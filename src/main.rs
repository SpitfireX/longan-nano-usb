#![no_std]
#![no_main]

use panic_halt as _;

use riscv_rt::entry;

use gd32vf103xx_hal as hal;
use hal::pac as pac;

use hal::prelude::*;
use hal::gpio::{Input, Floating, gpioa::*, gpioc::*, State, PushPull, Output};
use hal::eclic::{EclicExt, Level, LevelPriorityBits};
use hal::serial;

use pac::{Interrupt, ECLIC};

use core::fmt::Write;

use embedded_hal::digital::v2::{StatefulOutputPin, OutputPin, ToggleableOutputPin};

use synopsys_usb_otg::{UsbBus, UsbPeripheral};

use usb_device::prelude::*;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Configure clocks
    let mut rcu = dp.RCU.configure()
        .ext_hf_clock(8.mhz())
        .sysclk(96.mhz())
        .freeze();

    assert!(rcu.clocks.usbclk_valid());

    let mut delay = hal::delay::McycleDelay::new(&rcu.clocks);

    // pin setup

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpioc = dp.GPIOC.split(&mut rcu);

    let mut red = gpioc.pc13.into_push_pull_output_with_state(State::Low);
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
    let rcu_regs = unsafe { &*pac::RCU::ptr() };

    // Enable USB peripheral
    rcu_regs.ahben.modify(|_, w| w.usbfsen().set_bit());

    // Reset USB peripheral
    rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().set_bit());
    rcu_regs.ahbrst.modify(|_, w| w.usbfsrst().clear_bit());

    loop {}
}