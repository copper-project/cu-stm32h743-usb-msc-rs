# SD Card USB MSC Build Commands
# Requires: cargo-binutils (cargo install cargo-binutils && rustup component add llvm-tools)

# Default recipe - build release binary
default: build

# Build release binary
build:
    cargo build --release

# Create binary file from ELF
bin: build
    llvm-objcopy -O binary target/thumbv7em-none-eabihf/release/sdcard-usb-msc target/thumbv7em-none-eabihf/release/sdcard-usb-msc.bin
    @echo "Binary created: target/thumbv7em-none-eabihf/release/sdcard-usb-msc.bin"

# Create DFU file for STM32 bootloader
# The STM32H743 DFU bootloader expects a specific format
dfu: bin
    @echo "Creating DFU file..."
    @# Option 1: If dfu-suffix is available (from dfu-util package)
    @if command -v dfu-suffix > /dev/null 2>&1; then \
        cp target/thumbv7em-none-eabihf/release/sdcard-usb-msc.bin target/thumbv7em-none-eabihf/release/sdcard-usb-msc.dfu && \
        dfu-suffix -v 0x0483 -p 0xdf11 -a target/thumbv7em-none-eabihf/release/sdcard-usb-msc.dfu && \
        echo "DFU file created: target/thumbv7em-none-eabihf/release/sdcard-usb-msc.dfu"; \
    else \
        echo "dfu-suffix not found. The .bin file can be used directly with dfu-util:"; \
        echo "  dfu-util -a 0 -s 0x08000000:leave -D target/thumbv7em-none-eabihf/release/sdcard-usb-msc.bin"; \
    fi

# Flash using probe-rs
flash: build
    cargo run --release

# Flash using dfu-util (device must be in DFU mode)
flash-dfu: bin
    dfu-util -a 0 -s 0x08000000:leave -D target/thumbv7em-none-eabihf/release/sdcard-usb-msc.bin

# Print binary size
size: build
    llvm-size target/thumbv7em-none-eabihf/release/sdcard-usb-msc

# Clean build artifacts
clean:
    cargo clean

# Check for required tools
check-tools:
    @echo "Checking required tools..."
    @command -v cargo-objcopy > /dev/null 2>&1 || echo "WARNING: cargo-binutils not installed (cargo install cargo-binutils)"
    @command -v probe-rs > /dev/null 2>&1 || echo "WARNING: probe-rs not installed"
    @command -v dfu-util > /dev/null 2>&1 || echo "WARNING: dfu-util not installed"
    @echo "Tool check complete"
