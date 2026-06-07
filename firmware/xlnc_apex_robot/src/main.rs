#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::dbg;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    bind_interrupts,
    block::ImageDef,
    gpio::{Input, Pull},
    i2c::{self, I2c},
    peripherals,
};
use sparkfun_otos::SparkfunOTOS;

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;

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
    let i2c_bus = I2c::new_async(p.I2C0, p.PIN_9, p.PIN_8, Irqs, Default::default());
    let irq_pin = Input::new(p.PIN_10, Pull::None);
    let mut otos = SparkfunOTOS::new(i2c_bus, irq_pin);
    otos.init().await.expect("Init otos failed");
    let ver = otos.get_version().await.expect("Unable to get_version");
    dbg!(ver);
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
