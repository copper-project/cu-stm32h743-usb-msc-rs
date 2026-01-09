# STM32H743 USB Mass Storage Device

A minimal, no-std Rust firmware that turns an STM32H743 microcontroller into a USB mass storage device, exposing an SD card as a standard USB drive.

Originally developed for the [Copper](https://github.com/copper-project/copper-rs) flight control framework to retrieve telemetry logs from flight controllers, but useful for any STM32H743-based project needing USB storage access.

## Features

- **Plug-and-play** — Works with any OS (Windows, Linux, macOS) without drivers
- **Full SCSI support** — Implements the complete USB Mass Storage Class (Bulk-Only Transport) protocol
- **4-bit SDMMC** — Fast SD card access via hardware SDMMC1 peripheral
- **Minimal footprint** — ~5KB RAM, optimized for size with LTO
- **Safe Rust** — No heap allocation, memory-safe embedded code

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **MCU** | STM32H743 (tested on MicoAir H743 flight controller) |
| **USB** | OTG2_HS pins: PA11 (DM), PA12 (DP) |
| **SD Card** | SDMMC1: PC12 (CLK), PD2 (CMD), PC8-PC11 (D0-D3) |
| **LED** | PE6 (optional, blinks during operation) |

## Building

### Prerequisites

```bash
# Install Rust embedded toolchain
rustup target add thumbv7em-none-eabihf

# Install probe-rs for flashing (optional)
cargo install probe-rs-tools

# Install just command runner (optional)
cargo install just
```

### Compile

```bash
cargo build --release
```

### Flash

Using probe-rs (SWD debugger):
```bash
cargo run --release
```

Using DFU bootloader:
```bash
just dfu
# or manually:
llvm-objcopy -O binary target/thumbv7em-none-eabihf/release/sdcard-usb-msc sdcard-usb-msc.bin
dfu-util -a 0 -s 0x08000000:leave -D sdcard-usb-msc.bin
```

## Usage

1. Insert an SD card into the board
2. Flash the firmware
3. Connect USB to your computer
4. The SD card appears as a removable drive

The device identifies as:
- **Vendor:** COPPER
- **Product:** SD Card Reader
- **VID:PID:** 1209:0001

## Use Case: Copper-rs Log Retrieval

When running [Copper](https://github.com/copper-project/copper-rs) on a flight controller, telemetry and flight logs are stored on the SD card. This firmware provides a simple way to extract those logs:

1. Power down the flight controller
2. Flash this USB MSC firmware
3. Connect to your computer and copy the log files
4. Re-flash your flight control firmware

No need for SD card adapters or removing the card from the board.

## Adapting to Your Hardware

To use with a different STM32H743 board, modify:

1. **Pin assignments** in `main.rs`:
   - SDMMC1 pins (CLK, CMD, D0-D3)
   - USB OTG pins
   - LED pin (optional)

2. **Memory layout** in `build.rs` if your board has different flash/RAM:
   ```rust
   FLASH : ORIGIN = 0x08000000, LENGTH = 2M
   RAM   : ORIGIN = 0x24000000, LENGTH = 512K
   ```

3. **Clock configuration** if using different oscillator setup

## Technical Details

- **USB Mode:** Full-Speed (12 Mbps) via internal PHY
- **SD Mode:** 4-bit SDMMC at 25 MHz
- **Block Size:** 512 bytes
- **Supported Commands:** TestUnitReady, Inquiry, ReadCapacity10/16, RequestSense, ModeSense6/10, Read, Write, ReadFormatCapacities

## Dependencies

| Crate | Purpose |
|-------|---------|
| `stm32h7xx-hal` | Hardware abstraction layer |
| `usb-device` | USB device framework |
| `usbd-storage` | Mass Storage Class implementation |
| `synopsys-usb-otg` | USB OTG controller driver |
| `defmt` + `defmt-rtt` | Debug logging via RTT |

## License

MIT OR Apache-2.0
