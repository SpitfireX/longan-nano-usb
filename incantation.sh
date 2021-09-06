#!/bin/bash

riscv64-elf-objcopy -O binary target/riscv32imac-unknown-none-elf/release/longan-nano-usb firmware.bin
dfu-util -a 0 -s 0x08000000:leave -D firmware.bin
