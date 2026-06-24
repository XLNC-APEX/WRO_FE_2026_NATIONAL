#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::block::ImageDef;
use xlnc_apex_robot::{
    btn_reset, init, /*motor_and_servo_play, motor_play, otos_print, play_song,*/
};

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
    info!("{}", devices.voltage.get().await.unwrap());
    info!("Intialized! Press btn2 to start. Then btn1 to reset.");
    devices.btn2.wait_for_low().await;
    spawner.spawn(btn_reset(devices.btn1, devices.watchdog).unwrap());

    devices.otos.reset_tracking().await.unwrap();
    devices.otos.calibrate_imu(255).await.unwrap();

    loop {
        devices.btn2.wait_for_falling_edge().await;
        let pos = devices.otos.get_pos().await.unwrap();
        info!("{}", pos);
        Timer::after_millis(100).await;
    }
    //Points to go through, measured irl by moving manually:
    //Pose { x: 0.0, y: 0.0, h: -0.0022050973 }
    //Pose { x: 0.054626465, y: -1.095581, h: -0.044773065 }
    //Pose { x: 2.414856, y: -1.1758423, h: 1.5301459 }
    //Pose { x: 2.3162842, y: -2.5476074, h: -0.08321846 }
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
