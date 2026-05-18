#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::{dbg, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{bind_interrupts, block::ImageDef, i2c, peripherals};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;
use ssd1306::{I2CDisplayInterface, Ssd1306Async, prelude::*};

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    I2C0_IRQ => i2c::InterruptHandler<peripherals::I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());
    let i2c_bus = i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c::Config::default());
    let mut display = Ssd1306Async::new(
        I2CDisplayInterface::new(i2c_bus),
        DisplaySize128x32,
        DisplayRotation::Rotate0,
    )
    .into_terminal_mode();
    display.init().await.expect("Init failed");
    display.clear().await.expect("Clear failed");
    display
        .write_str("excellen ape \n\t\\_-X-_/")
        .await
        .expect("Write failed");
    dbg!("DA"); // does not print to cargo embed
    info!("bb");
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
