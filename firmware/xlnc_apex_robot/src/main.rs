#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::block::ImageDef;
use xlnc_apex_robot::{init, motor_play};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = hal::init(Default::default());
    let mut devices = init(p).await;
    info!("Intialized! Press btn2 to start.");
    devices.btn2.wait_for_low().await;
    // devices.servo.set_pos_deg(90.0).expect("bbbbb");
    // Timer::after_millis(2000).await;
    // devices.servo.set_pos_deg(-90.0).unwrap();
    // info!("Servo movings complete");
    info!("Starting motor");
    spawner.spawn(motor_play(devices.motor).unwrap());

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
