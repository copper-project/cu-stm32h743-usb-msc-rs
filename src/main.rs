//! USB Mass Storage device exposing SD card as a block device
//!
//! This firmware exposes the SD card connected via SDMMC1 as a USB Mass Storage
//! device using the SCSI transparent command set and Bulk-Only Transport.
//!
//! Target: STM32H743VIT6 (MicoAir H743 board)
//! USB: PA11 (DM) / PA12 (DP) -> USB2 (OTG2_HS)
//! SD Card: SDMMC1 4-bit

#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::sdmmc::{SdCard, Sdmmc};
use stm32h7xx_hal::usb_hs::{UsbBus, USB2};
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*};

use usb_device::prelude::*;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};

use core::borrow::BorrowMut;

/// Block size for SD card (standard 512 bytes)
const BLOCK_SIZE: usize = 512;

/// USB endpoint memory
static mut EP_MEMORY: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

/// USB I/O buffer for mass storage - needs to be large enough for multi-block transfers
/// Using 4KB (8 blocks) to match common host request sizes
static mut USB_BUF: [u8; 4096] = [0u8; 4096];

/// Wrapper for static buffer that implements BorrowMut<[u8]>
struct StaticBuffer(&'static mut [u8]);

impl BorrowMut<[u8]> for StaticBuffer {
    fn borrow_mut(&mut self) -> &mut [u8] {
        self.0
    }
}

impl core::borrow::Borrow<[u8]> for StaticBuffer {
    fn borrow(&self) -> &[u8] {
        self.0
    }
}

/// Global SDMMC instance for use in main loop
static mut SDMMC: Option<Sdmmc<pac::SDMMC1, SdCard>> = None;

/// SD card block count
static mut BLOCK_COUNT: u32 = 0;

/// Read a block from the SD card
fn sd_read_block(lba: u32, buf: &mut [u8; 512]) -> Result<(), ()> {
    unsafe {
        if let Some(ref mut sdmmc) = SDMMC {
            sdmmc.read_block(lba, buf).map_err(|e| {
                error!("SD read error at LBA {}: {:?}", lba, Debug2Format(&e));
            })
        } else {
            Err(())
        }
    }
}

/// Write a block to the SD card
fn sd_write_block(lba: u32, buf: &[u8; 512]) -> Result<(), ()> {
    unsafe {
        if let Some(ref mut sdmmc) = SDMMC {
            sdmmc.write_block(lba, buf).map_err(|e| {
                error!("SD write error at LBA {}: {:?}", lba, Debug2Format(&e));
            })
        } else {
            Err(())
        }
    }
}

#[entry]
fn main() -> ! {
    info!("SD Card USB Mass Storage starting...");

    // Take peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Configure power and clocks
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(400.MHz())
        .pll1_q_ck(100.MHz())
        .freeze(vos, &dp.SYSCFG);

    // HSI48 is always enabled on H7, verify and configure for USB
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must be running for USB");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);
    info!("HSI48 clock configured for USB");

    // Configure GPIO ports
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure status LED (PE6)
    let mut led = gpioe.pe6.into_push_pull_output();
    led.set_low();

    // Configure SDMMC1 pins
    let sdmmc_clk = gpioc.pc12.into_alternate::<12>().speed(Speed::VeryHigh);
    let sdmmc_cmd = gpiod.pd2.into_alternate::<12>().speed(Speed::VeryHigh);
    let sdmmc_d0 = gpioc.pc8.into_alternate::<12>().speed(Speed::VeryHigh);
    let sdmmc_d1 = gpioc.pc9.into_alternate::<12>().speed(Speed::VeryHigh);
    let sdmmc_d2 = gpioc.pc10.into_alternate::<12>().speed(Speed::VeryHigh);
    let sdmmc_d3 = gpioc.pc11.into_alternate::<12>().speed(Speed::VeryHigh);

    info!("Initializing SDMMC1...");

    // Initialize SDMMC peripheral
    let mut sdmmc: Sdmmc<_, SdCard> = dp.SDMMC1.sdmmc(
        (sdmmc_clk, sdmmc_cmd, sdmmc_d0, sdmmc_d1, sdmmc_d2, sdmmc_d3),
        ccdr.peripheral.SDMMC1,
        &ccdr.clocks,
    );

    // Initialize SD card at 25MHz
    if let Err(e) = sdmmc.init(25.MHz()) {
        error!("SDMMC init failed: {:?}", Debug2Format(&e));
        loop {
            led.toggle();
            cortex_m::asm::delay(4_000_000);
        }
    }

    // Get card info
    let card = match sdmmc.card() {
        Ok(c) => c,
        Err(e) => {
            error!("No SD card detected: {:?}", Debug2Format(&e));
            loop {
                led.toggle();
                cortex_m::asm::delay(4_000_000);
            }
        }
    };

    // Calculate block count from card size
    let card_size = card.size();
    let block_count = (card_size / 512) as u32;
    let capacity_mb = card_size / (1024 * 1024);
    info!("SD card: {} bytes, {} blocks ({} MB)", card_size, block_count, capacity_mb);

    // Store SDMMC and block count in static for use in main loop
    unsafe {
        BLOCK_COUNT = block_count;
        SDMMC = Some(sdmmc);
    }

    // Configure USB2 pins (PA11 = DM, PA12 = DP) with Alternate<10>
    let usb_dm = gpioa.pa11.into_alternate::<10>();
    let usb_dp = gpioa.pa12.into_alternate::<10>();

    info!("Initializing USB2...");

    // Create USB2 peripheral (OTG2_HS with internal FS PHY)
    let usb = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        usb_dm,
        usb_dp,
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    // Initialize EP_MEMORY to zero
    let ep_mem = unsafe {
        let buf: &mut [MaybeUninit<u32>; 1024] =
            &mut *(core::ptr::addr_of_mut!(EP_MEMORY) as *mut _);
        for value in buf.iter_mut() {
            value.as_mut_ptr().write(0);
        }
        EP_MEMORY.assume_init_mut()
    };

    // Create USB bus
    let usb_bus = UsbBus::new(usb, ep_mem);

    // Create SCSI Mass Storage class
    // packet_size = 64 for FS, max_lun = 0 (single LUN)
    // Use StaticBuffer wrapper to satisfy BorrowMut<[u8]> bound
    let usb_buf = StaticBuffer(unsafe { &mut USB_BUF });
    let mut msc = Scsi::new(&usb_bus, 64, 0, usb_buf)
        .expect("Failed to create SCSI device");

    // Create USB device
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x0001))
        .strings(&[StringDescriptors::default()
            .manufacturer("Copper Project")
            .product("SD Card Reader")
            .serial_number("00000001")])
        .expect("Failed to set USB strings")
        .device_class(0x00)
        .max_packet_size_0(64)
        .expect("Invalid max_packet_size")
        .build();

    info!("USB Mass Storage device ready");
    led.set_high(); // LED on = ready

    // Transfer state for multi-block operations
    #[derive(Default)]
    struct TransferState {
        /// Current LBA being processed
        lba: u32,
        /// Blocks remaining to transfer
        blocks_remaining: u16,
        /// Bytes sent/received in current block
        block_offset: usize,
        /// Is a transfer in progress?
        active: bool,
        /// Is this a write operation?
        is_write: bool,
    }

    let mut transfer = TransferState::default();
    let mut block_buf = [0u8; BLOCK_SIZE];

    // Main loop
    loop {
        // Poll USB
        if !usb_dev.poll(&mut [&mut msc]) {
            continue;
        }

        // Process SCSI commands
        let _ = msc.poll(|mut cmd| {
            let block_count = unsafe { BLOCK_COUNT };

            match cmd.kind {
                ScsiCommand::TestUnitReady => {
                    debug!("SCSI: TestUnitReady");
                    cmd.pass();
                }

                ScsiCommand::Inquiry { .. } => {
                    debug!("SCSI: Inquiry");
                    // Standard inquiry data (36 bytes)
                    let inquiry_data = [
                        0x00, // Peripheral device type: SBC direct access
                        0x80, // RMB: Removable
                        0x04, // Version: SPC-2
                        0x02, // Response format
                        0x1F, // Additional length (31 bytes follow)
                        0x00, 0x00, 0x00, // Flags
                        // Vendor ID (8 bytes, space padded)
                        b'C', b'O', b'P', b'P', b'E', b'R', b' ', b' ',
                        // Product ID (16 bytes, space padded)
                        b'S', b'D', b' ', b'C', b'a', b'r', b'd', b' ',
                        b'R', b'e', b'a', b'd', b'e', b'r', b' ', b' ',
                        // Product revision (4 bytes)
                        b'1', b'.', b'0', b'0',
                    ];
                    let _ = cmd.try_write_data_all(&inquiry_data);
                    cmd.pass();
                }

                ScsiCommand::ReadCapacity10 => {
                    debug!("SCSI: ReadCapacity10");
                    let last_lba = block_count.saturating_sub(1);
                    let mut data = [0u8; 8];
                    data[0..4].copy_from_slice(&last_lba.to_be_bytes());
                    data[4..8].copy_from_slice(&(BLOCK_SIZE as u32).to_be_bytes());
                    let _ = cmd.try_write_data_all(&data);
                    cmd.pass();
                }

                ScsiCommand::ReadCapacity16 { .. } => {
                    debug!("SCSI: ReadCapacity16");
                    let last_lba = (block_count as u64).saturating_sub(1);
                    let mut data = [0u8; 32];
                    data[0..8].copy_from_slice(&last_lba.to_be_bytes());
                    data[8..12].copy_from_slice(&(BLOCK_SIZE as u32).to_be_bytes());
                    let _ = cmd.try_write_data_all(&data);
                    cmd.pass();
                }

                ScsiCommand::RequestSense { .. } => {
                    debug!("SCSI: RequestSense");
                    // Fixed format sense data (18 bytes)
                    let sense_data = [
                        0x70, // Response code: current errors, fixed format
                        0x00, // Obsolete
                        0x00, // Sense key: NO SENSE
                        0x00, 0x00, 0x00, 0x00, // Information
                        0x0A, // Additional sense length (10 bytes follow)
                        0x00, 0x00, 0x00, 0x00, // Command specific
                        0x00, // ASC: No additional sense
                        0x00, // ASCQ
                        0x00, 0x00, 0x00, 0x00, // FRU + SKSV
                    ];
                    let _ = cmd.try_write_data_all(&sense_data);
                    cmd.pass();
                }

                ScsiCommand::ModeSense6 { .. } => {
                    debug!("SCSI: ModeSense6");
                    // Minimal mode parameter header (4 bytes)
                    let mode_data = [
                        0x03, // Mode data length (3 bytes follow)
                        0x00, // Medium type: default
                        0x00, // Device-specific: not write-protected, no cache
                        0x00, // Block descriptor length: 0
                    ];
                    let _ = cmd.try_write_data_all(&mode_data);
                    cmd.pass();
                }

                ScsiCommand::ModeSense10 { .. } => {
                    debug!("SCSI: ModeSense10");
                    // Mode parameter header (8 bytes)
                    let mode_data = [
                        0x00, 0x06, // Mode data length (6 bytes follow)
                        0x00, // Medium type: default
                        0x00, // Device-specific: not write-protected
                        0x00, 0x00, // Reserved
                        0x00, 0x00, // Block descriptor length: 0
                    ];
                    let _ = cmd.try_write_data_all(&mode_data);
                    cmd.pass();
                }

                ScsiCommand::Read { lba, len } => {
                    if !transfer.active {
                        // Start new read transfer
                        debug!("SCSI: Read lba={} len={}", lba, len);
                        transfer.lba = lba;
                        transfer.blocks_remaining = len;
                        transfer.block_offset = BLOCK_SIZE; // Force read on first iteration
                        transfer.active = true;
                        transfer.is_write = false;
                    }

                    // Process read transfer
                    while transfer.blocks_remaining > 0 || transfer.block_offset < BLOCK_SIZE {
                        // Need to read a new block from SD?
                        if transfer.block_offset >= BLOCK_SIZE {
                            if transfer.blocks_remaining == 0 {
                                break;
                            }
                            // Read next block from SD card
                            if sd_read_block(transfer.lba, &mut block_buf).is_err() {
                                transfer.active = false;
                                cmd.fail();
                                return;
                            }
                            transfer.lba += 1;
                            transfer.blocks_remaining -= 1;
                            transfer.block_offset = 0;
                        }

                        // Try to write data to USB
                        match cmd.write_data(&block_buf[transfer.block_offset..]) {
                            Ok(written) => {
                                transfer.block_offset += written;
                            }
                            Err(_) => {
                                // USB buffer full, will continue on next poll
                                return;
                            }
                        }
                    }

                    // Transfer complete
                    transfer.active = false;
                    cmd.pass();
                }

                ScsiCommand::Write { lba, len } => {
                    if !transfer.active {
                        // Start new write transfer
                        debug!("SCSI: Write lba={} len={}", lba, len);
                        transfer.lba = lba;
                        transfer.blocks_remaining = len;
                        transfer.block_offset = 0;
                        transfer.active = true;
                        transfer.is_write = true;
                    }

                    // Process write transfer
                    while transfer.blocks_remaining > 0 {
                        // Read data from USB into block buffer
                        match cmd.read_data(&mut block_buf[transfer.block_offset..]) {
                            Ok(read) => {
                                transfer.block_offset += read;
                            }
                            Err(_) => {
                                // No more data available, will continue on next poll
                                return;
                            }
                        }

                        // Got a complete block?
                        if transfer.block_offset >= BLOCK_SIZE {
                            // Write to SD card
                            if sd_write_block(transfer.lba, &block_buf).is_err() {
                                transfer.active = false;
                                cmd.fail();
                                return;
                            }
                            transfer.lba += 1;
                            transfer.blocks_remaining -= 1;
                            transfer.block_offset = 0;
                        }
                    }

                    // Transfer complete
                    transfer.active = false;
                    cmd.pass();
                }

                ScsiCommand::ReadFormatCapacities { .. } => {
                    debug!("SCSI: ReadFormatCapacities");
                    // Capacity list (12 bytes)
                    let mut data = [0u8; 12];
                    // Capacity list header
                    data[0..4].copy_from_slice(&[0, 0, 0, 8]); // Capacity list length (8 bytes follow)
                    // Current/maximum capacity descriptor
                    data[4..8].copy_from_slice(&block_count.to_be_bytes()); // Number of blocks
                    data[8] = 0x02; // Descriptor type: Formatted media
                    // Block length (3 bytes, big-endian)
                    data[9] = 0x00;
                    data[10] = 0x02; // 512 = 0x000200
                    data[11] = 0x00;
                    let _ = cmd.try_write_data_all(&data);
                    cmd.pass();
                }

                ScsiCommand::Unknown { cmd: opcode } => {
                    warn!("SCSI: Unknown command 0x{:02x}", opcode);
                    cmd.fail();
                }

                // Handle any future variants (non_exhaustive enum)
                _ => {
                    warn!("SCSI: Unhandled command");
                    cmd.fail();
                }
            }
        });

        // Blink LED periodically to show activity
        static mut COUNTER: u32 = 0;
        unsafe {
            COUNTER = COUNTER.wrapping_add(1);
            if COUNTER % 500_000 == 0 {
                led.toggle();
            }
        }
    }
}
