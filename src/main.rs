#![no_std]
#![no_main]

use byteorder::ByteOrder;
use byteorder::LittleEndian;
use panic_halt as _;

use riscv::register::mhpmcounter10::write;
use riscv_rt::entry;

use gd32vf103xx_hal as hal;
use hal::pac as pac;

use hal::prelude::*;
use hal::gpio::{Input, Floating, gpioa::*, gpioc::*, State, PushPull, Output};
use hal::eclic::{EclicExt, Level, LevelPriorityBits};
use hal::serial;

use pac::{Interrupt, ECLIC};

use core::convert::TryFrom;
use core::fmt::Write;

use embedded_hal::digital::v2::{StatefulOutputPin, OutputPin, ToggleableOutputPin};

use synopsys_usb_otg::{UsbBus, UsbPeripheral};

use usb_device::prelude::*;

use usb_setup::*;

use core::ptr;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp.RCU.configure()
        .ext_hf_clock(8.mhz())
        .sysclk(96.mhz())
        .freeze();

    assert!(rcu.clocks.usbclk_valid());

    // pin setup

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);

    let mut _red = gpioc.pc13.into_push_pull_output_with_state(State::Low);
    let mut green = gpioa.pa1.into_push_pull_output_with_state(State::High);
    let mut blue = gpioa.pa2.into_push_pull_output_with_state(State::High);

    // UART setup
    let mut afio = dp.AFIO.constrain(&mut rcu);

    let pin_tx = gpioa.pa9;
    let pin_rx = gpioa.pa10;

    let serial = serial::Serial::new(
        dp.USART0,
        (pin_tx, pin_rx),
        serial::Config::default().baudrate(19_200.bps()),
        &mut afio,
        &mut rcu,
    );

    let (mut tx, _) = serial.split();

    writeln!(tx, "halló heimur!\r").ok();

    // ECLIC stuff
    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    // Use 3 bits for level, 1 for priority
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);

    let usbfs_global = unsafe { &*pac::USBFS_GLOBAL::ptr() };
    let usbfs_device = unsafe { &*pac::USBFS_DEVICE::ptr() };
    let rcu_regs = unsafe { &*pac::RCU::ptr() };

    // Enable USB peripheral
    rcu_regs.ahben.modify(|_, w| w.usbfsen().set_bit());

    // Reset USB peripheral
    rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
    rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());

    //
    // usb core initialization
    //

    // force device mode
    // disable additional OTG stuff
    usbfs_global.gusbcs.modify(|_, w| {
        w.fdm().set_bit()
         .fhm().clear_bit()
         .hnpcen().clear_bit()
         .srpcen().clear_bit()
    });

    // TxFIFO interrupt when completely empty
    // global interrupt enable
    usbfs_global.gahbcs.modify(|_, w| {
        w.txfth().set_bit()
         .ginten().set_bit()
    });

    // set device speed
    usbfs_device.dcfg.modify(|_, w| {
        unsafe {
            w.ds().bits(0b11) // full speed
        }
    });

    // enable interrupts
    usbfs_global.ginten.modify(|_, w| {
        w.rstie().set_bit() // usb reset
         .enumfie().set_bit() // enumeration done
         .sofie().set_bit() // usb sof
         .spie().set_bit() // usb suspend
         .espie().set_bit() // early usb suspend
         .rxfneie().set_bit()
    });

    writeln!(tx, "gintf: {:#034b}\r", usbfs_global.gintf.read().bits()).ok();

    // power on phy etc
    usbfs_global.gccfg.modify(|_, w| {
        w.vbusig().set_bit() // monitor vbus - disable this because longan nano is weird
         .sofoen().set_bit() // enable SOF output pin
         .vbusacen().set_bit() // VBUS A-device Comparer enable
         .vbusbcen().set_bit() // VBUS B-device Comparer enable
         .pwron().set_bit() // USB PHY power on
    });

    // wait for usb reset interrupt
    while usbfs_global.gintf.read().rst().bit_is_clear() {}
    green.set_low().ok();

    // wait for usb enum done interrupt
    while usbfs_global.gintf.read().enumf().bit_is_clear() {}
    blue.set_low().ok();

    // enable endpoint 0
    // set maximum packet size for endpoint 0
    usbfs_device.diep0ctl.modify(|_, w| {
        unsafe {
            w.epen().set_bit()
            .mpl().bits(0b11) // 8 bytes
        }
    });

    // enable additional interrupts
    usbfs_global.ginten.modify(|_, w| {
        w.rxfneie().set_bit() // Receive FIFO non-empty interrupt
    });

    // configure OUT 0 endpoint transfer length
    usbfs_device.doep0len.modify(|_, w| {
        unsafe {
            w.stpcnt().bits(3)
        }
    });

    // set FIFO sizes
    usbfs_global.grflen.modify(|_, w| unsafe { w.rxfd().bits(12) });
    usbfs_global.diep0tflen_mut().modify(|_, w| unsafe { w.iep0txfd().bits(12) });

    let mut usbdata = [0_u8; 1024];
    let mut di = 0_usize;

    loop {
        if usbfs_global.gintf.read().rxfneif().bit_is_set() {
            
            // mask Receive FIFO non-empty interrupt
            usbfs_global.ginten.modify(|_, w| { w.rxfneie().clear_bit() });
            
            match RxStatus::read(usbfs_global) {
                Ok(s) => {
                    writeln!(tx, "grstatp status: {:?}\r", s).ok();

                    // pop more data if necessary
                    if s.byte_count > 0 {
                        let npops = if s.byte_count%4 == 0 {
                            s.byte_count/4
                        } else {
                            (s.byte_count/4)+1
                        };

                        for _ in 0..npops {
                            let data = usbfs_global.grstatp_device().read().bits();
                            writeln!(tx, "grstatp word: {:#010X}\r", data).ok();

                            let bytes = data.to_ne_bytes();
                            usbdata[di..di+bytes.len()].copy_from_slice(&bytes);
                            di += 4;
                        }

                        writeln!(tx, "grstatp received data: {:02X?}\r", &usbdata[..di]).ok();
                    }

                    // packet matching stuff
                    match s.received_packet_status {
                        ReceivedPacketStatus::SetupReceived => {
                            match SetupPacket::parse(&usbdata[..di]) {
                                Ok(packet) => {
                                    writeln!(tx, "setup parse: {:04X?}\r", packet).ok();
                                    process_setup(&usbfs_global, &usbfs_device, &packet);
                                }
                                Err(e) => {
                                    writeln!(tx, "setup parse: error \"{}\"\r", e).ok();
                                }
                            }
                        }
                        _ => {}
                    }

                    // clear usb_data
                    usbdata.fill(0);
                    di = 0;
                }

                Err(v) => {
                    writeln!(tx, "grstatp error: {:#010X}\r", v).ok();
                }
            };

            // unmask Receive FIFO non-empty interrupt
            usbfs_global.ginten.modify(|_, w| { w.rxfneie().set_bit() });

            writeln!(tx, "---\r").ok();
        }
    }
}

const DESCRIPTOR: [u8; 18] = [
    // Device
    0x12,                       // bLength (Size of this descriptor in bytes) 
    0x01,                       // bDescriptorType (DEVICE Descriptor Type) 
    0x00, 0x02,                 // bcdUSB (USB Specification Release Number in Binary-Coded Decimal) ("2.0") (512) 
    0x00,                       // bDeviceClass (Class code (assigned by the USB-IF).) ("Defined at interface level") 
    0x00,                       // bDeviceSubClass (Subclass code (assigned by the USB-IF).) 
    0x00,                       // bDeviceProtocol (Protocol code (assigned by the USB-IF).) 
    0x40,                       // bMaxPacketSize0 (Maximum packet size for endpoint zero) ("64") 
    0xC0, 0x16,                 // idVendor (Vendor ID (assigned by the USB-IF)) (0) 
    0xDD, 0x27,                 // idProduct (Product ID (assigned by the manufacturer)) (0) 
    0x00, 0x00,                 // bcdDevice (Device release number in binary-coded decimal) (0) 
    0x01,                       // iManufacturer 
    0x02,                       // iProduct 
    0x03,                       // iSerialNumber 
    0x01,                       // bNumConfigurations (Number of possible configurations) 
];  

fn process_setup(usbfs_global: &pac::usbfs_global::RegisterBlock, usbfs_device: &pac::usbfs_device::RegisterBlock, packet: &SetupPacket) {
    match (&packet.request_type, &packet.request) {
        (_, DeviceRequest::GetDescriptor) => {
            ep0in_setup_tx(usbfs_global, usbfs_device, &DESCRIPTOR);
            ep0in_enable(usbfs_global, usbfs_device);
            ep0in_tx(usbfs_global, usbfs_device, &DESCRIPTOR);
        }
        (_, _) => {}
    }
}

fn ep0in_enable(usbfs_global: &pac::usbfs_global::RegisterBlock, usbfs_device: &pac::usbfs_device::RegisterBlock) {
    usbfs_device.diep0ctl.modify(|_, w| {
        unsafe {
            w.epen().set_bit() // endpoint enable
             .mpl().bits(0) // 64 bytes
             .txfnum().bits(0) // fifo 0
             .cnak().clear_bit() // disable NAK
        }
    });
}

fn ep0in_setup_tx(usbfs_global: &pac::usbfs_global::RegisterBlock, usbfs_device: &pac::usbfs_device::RegisterBlock, buffer: &[u8]) {
    usbfs_device.diep0len.modify(|_, w| {
        unsafe {
            w.tlen().bits(buffer.len() as u8)
             .pcnt().bits(1)
        }
    });
}

fn ep0in_tx(usbfs_global: &pac::usbfs_global::RegisterBlock, usbfs_device: &pac::usbfs_device::RegisterBlock, buffer: &[u8]) {
    let fifo = (0x50000000 + 0x1000) as *mut u32;
    
    for chunk in buffer.rchunks(4) {
        let mut w = LittleEndian::read_u32(chunk);

        if chunk.len() != 4 {
            w <<= 4 - chunk.len();
        }

        unsafe {
            ptr::write_volatile(fifo, w);
        }
    }
}

use numeric_enum_macro::numeric_enum;

numeric_enum! {
    #[repr(u8)]
    #[derive(Debug)]
    enum DataPID {
        Data0 = 0b00,
        Data1 = 0b10,
        Data2 = 0b01,
        MData = 0b11,
    }
}

numeric_enum! {
    #[repr(u8)]
    #[derive(Debug)]
    enum ReceivedPacketStatus {
        GlobalOutNAK = 0b0001,
        OutReceived = 0b0010,
        OutTxComplete = 0b0011,
        SetupComplete = 0b0100,
        SetupReceived = 0b0110,
    }
}

#[derive(Debug)]
struct RxStatus {
    received_packet_status: ReceivedPacketStatus,
    data_pid: DataPID,
    byte_count: u16,
    endpoint_num: u8,
}

impl RxStatus {
    fn read(usbfs_global: &pac::usbfs_global::RegisterBlock) -> Result<Self, u32> {
        let reg = usbfs_global.grstatp_device().read();

        let received_packet_status = match ReceivedPacketStatus::try_from(reg.rpckst().bits()) {
            Ok(rps) => rps,
            Err(_) => return Err(reg.bits()),
        };

        let data_pid = match DataPID::try_from(reg.dpid().bits()) {
            Ok(dp) => dp,
            Err(_) => return Err(reg.bits()),
        };

        Ok(Self {
            received_packet_status,
            data_pid,
            byte_count: reg.bcount().bits(),
            endpoint_num: reg.epnum().bits(),
        })
    }
}

mod usb_setup {
    use core::{convert::TryFrom, marker::PhantomData};

    use numeric_enum_macro::numeric_enum;

    use byteorder::{ByteOrder, LittleEndian};

    // USB Setup bmRequestType D7 Data Direction
    numeric_enum! {
        #[repr(u8)]
        #[derive(Debug)]
        pub enum DataDirection {
            HostToDevice = 0,
            DeviceToHost = 1,
        }
    }

    // USB Setup bmRequestType D6:5 Type
    numeric_enum! {
        #[repr(u8)]
        #[derive(Debug)]
        pub enum Type {
            Standard = 0,
            Class = 1,
            Vendor = 2,
            Reserved = 3,
        }
    }

    // USB Setup bmRequestType D4:0 Recipient
    numeric_enum! {
        #[repr(u8)]
        #[derive(Debug)]
        pub enum Recipient {
            Device = 0,
            Interface = 1,
            Endpoint = 2,
            Other = 3,
        }
    }

    // USB Setup bmRequestType
    #[derive(Debug)]
    pub struct RequestType {
        pub data_direction: DataDirection,
        pub request_type: Type,
        pub recipient: Recipient,
    }

    impl RequestType {
        pub fn from_u8(byte: u8) -> Result<Self, ()> {
            let data_direction = match DataDirection::try_from(byte >> 7) {
                Ok(v) => v,
                Err(_) => return Err(()),
            };

            let request_type = match Type::try_from((byte >> 5) & 3) {
                Ok(v) => v,
                Err(_) => return Err(()),
            };

            let recipient = match Recipient::try_from(byte & 31) {
                Ok(v) => v,
                Err(_) => return Err(()),
            };
            
            Ok(Self {
                data_direction,
                request_type,
                recipient,
            })
        }
    }

    // USB Setup bRequest for device requests
    numeric_enum! {
        #[repr(u8)]
        #[derive(Debug)]
        pub enum DeviceRequest {
            GetStatus = 0,
            ClearFeature = 1,
            SetFeature = 3,
            SetAddress = 5,
            GetDescriptor = 6,
            SetDescriptor = 7,
            GetConfiguration = 8,
            SetConfiguration = 9,
        }
    }

    #[derive(Debug)]
    pub struct SetupPacket {
        pub request_type: RequestType,
        pub request: DeviceRequest,
        pub value: u16,
        pub index: u16,
        pub length: u16,
    }

    impl SetupPacket {
        pub fn parse(data: &[u8]) -> Result<Self, &str> {
            if data.len() >= 8 {
                let request_type = match RequestType::from_u8(data[0]){
                    Ok(v) => v,
                    Err(_) => return Err("parsing request type"),
                };

                let request = match DeviceRequest::try_from(data[1]) {
                    Ok(v) => v,
                    Err(_) => return Err("parsing request"),
                };

                Ok(Self {
                    request_type,
                    request,
                    value: LittleEndian::read_u16(&data[2..4]),
                    index: LittleEndian::read_u16(&data[4..6]),
                    length: LittleEndian::read_u16(&data[6..]),
                })
            } else {
                Err("wrong len")
            }
        }
    }
}