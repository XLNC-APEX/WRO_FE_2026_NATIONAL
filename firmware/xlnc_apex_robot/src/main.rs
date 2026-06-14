#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::block::ImageDef;
use xlnc_apex_robot::{init, motor_and_servo_play, motor_play, otos_print, play_song};

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
    Timer::after_millis(600).await;
    info!("{}", devices.voltage.get().await.unwrap());
    info!("Intialized! Press btn2 to start. Then btn1 to reset.");
    devices.btn2.wait_for_low().await;
    spawner.spawn(play_song(devices.buzzer).unwrap());
    devices.servo.set_pos_deg(0.0).unwrap();
    Timer::after_millis(500).await; // wait until it stays still
    drop(devices.servo); // drop servo so it doesn't servo
    Timer::after_millis(2000).await; // wait until it stays still
    info!("Servo movings complete\n");
    info!("Resetting and calibrating OTOS\n");
    devices.otos.reset_tracking().await.unwrap();
    devices.otos.calibrate_imu(255).await.unwrap();
    info!("Done! Launching task to log otos data.\n");
    spawner.spawn(otos_print(devices.otos).unwrap());
    // info!("Starting motor");
    // spawner.spawn(motor_play(devices.motor).unwrap());
    // spawner.spawn(motor_and_servo_play(devices.motor, devices.servo).unwrap());

    devices.btn1.wait_for_low().await;
    devices.watchdog.trigger_reset();

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
