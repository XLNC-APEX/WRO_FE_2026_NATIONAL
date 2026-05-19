#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::{dbg, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    bind_interrupts,
    block::ImageDef,
    gpio::{Input, Output},
    i2c, peripherals,
};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;
use vl53l0x::VL53L0x;

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
    info!("bb");
    // TODO: probably move all or some of this logic to the driver crate
    let mut xshut_left = Output::new(p.PIN_2, hal::gpio::Level::Low);
    info!("xshut");

    // let mut xshut_right = Output::new(p.PIN_4, hal::gpio::Level::Low);
    let mut irq_left = Input::new(p.PIN_3, hal::gpio::Pull::Up);
    // let mut irq_right = Input::new(p.PIN_5, hal::gpio::Pull::Up);
    info!("irq_left");

    xshut_left.set_high();
    info!("xshut high");
    Timer::after_micros(1250).await; // t_boot is 1.2ms max
    info!("waited");
    let mut left_dist = VL53L0x::new(i2c_bus).await.expect("Tof create failed");
    info!("left_dist");
    left_dist
        .start_continuous(10)
        .await
        .expect("Cannot start continuous");
    info!("cont started");
    for i in 0..100 {
        let range = left_dist
            .read_range_mm(&mut irq_left)
            .await
            .expect("Couldn't read range in mm: try inches");
        info!("{} range {}", i, range);
    }
    dbg!("DA"); // does not print to cargo embed
    left_dist
        .stop_continuous()
        .await
        .expect("Cannot stop continuous");
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
