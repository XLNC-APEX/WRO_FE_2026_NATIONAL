#![no_std]
#![no_main]

use defmt::dbg;
use embassy_executor::Spawner;
// use embassy_rp::i2c::;
use embassy_rp as hal;
use embassy_time::Timer;
use hal::block::ImageDef;
use hal::i2c::{I2c, InterruptHandler};
// use hal::interrupt::Interrupt::I2C0_IRQ;

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;
use vl53l0x::VL53L0x;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

mod constants;
// use constants::*;

embassy_rp::bind_interrupts!(struct Irqs {
    I2C0_IRQ => InterruptHandler<hal::peripherals::I2C0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut bus =
        hal::i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, hal::i2c::Config::default());
    let mut dist = VL53L0x::new(&mut bus).expect("tof");
    dist.set_address(0x52).expect("set address");
    dbg!(dist.read_range_mm().expect("distance in mm"));

    loop {
        Timer::after_millis(100).await;
    }
}

// #[embassy_executor::task]
// async fn init_distance_sensors(i2c: I2c) {

// }

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
