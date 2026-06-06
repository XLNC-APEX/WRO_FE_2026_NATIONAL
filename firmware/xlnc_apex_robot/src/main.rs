#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::dbg;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    bind_interrupts, block::ImageDef, gpio::{Level, Output}, peripherals::{DMA_CH0, DMA_CH1}, spi::{self, CsPin, Spi}
};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());
    let mut spi_config = spi::Config::default();
    spi_config.polarity = spi::Polarity::IdleHigh;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    dbg!("Ya amongaus!");
    let mut cs_pin = Output::new(p.PIN_13, Level::High);
    let mut spi_bus = Spi::new(
        p.SPI1, p.PIN_14, p.PIN_15, p.PIN_12, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config,
    );
    let tx: [u8; 4] = [
        0xae, // first byte of no_checksum_sync (little endian -> least-significant byte first)
        0xc1, // second byte of no_checksum_sync
        0x0e, // this is the version request type
        0x00, // data_length is 0
    ];
    dbg!("Will do comunicaions");
    // spi_bus.write(&tx).await.expect("Cannot write");
    let mut rx = [0_u8; 35];
    // spi_bus.read(&mut rx).await.expect("Cannot read");
    cs_pin.set_low();
    spi_bus.transfer(&mut rx, &tx).await.expect("Cannot transfer");
    dbg!(rx);

    loop {
        Timer::after_millis(100).await;
    }
}

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"xlnc_apex_robot"),
    embassy_rp::binary_info::rp_program_description!(c"your program description"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// End of file
