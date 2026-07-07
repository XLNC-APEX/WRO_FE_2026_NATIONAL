#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use core::f32::{self, consts::FRAC_PI_6};

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::block::ImageDef;
use nalgebra::Point2;
use tb6612fng::DriveCommand::Backward;
use xlnc_apex_robot::{
    ApexCar, PurePursuit, PurePursuitConfig, beeper_task, btn_reset, init, pure_pursuit,
};

// Panic Handler
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
    spawner.spawn(beeper_task(devices.buzzer).unwrap()); // beep();
    spawner.spawn(btn_reset(devices.btn1, devices.watchdog).unwrap());

    devices.otos.reset_tracking().await.unwrap();
    devices.otos.calibrate_imu(255).await.unwrap();
    let ppconf = PurePursuitConfig {
        kl: 0.8,
        min_l: 0.1,
        max_l: 0.5,
        l_drv: 0.096,
        max_steer_rad: FRAC_PI_6,
    };
    let car = ApexCar::new(devices.servo, devices.otos);
    // static PATH: &[Point2<f32>] = &[
    //     Point2::new(0.0, 0.0),
    //     Point2::new(0.054626465, -1.095581),
    //     Point2::new(2.414856, -1.1758423),
    //     Point2::new(2.3162842, -2.5476074),
    // ];
    static PATH: &[Point2<f32>] = &[
        Point2::new(0.0, 0.0),
        Point2::new(1.095581, 0.054626465),
        Point2::new(1.1758423, 2.414856),
        Point2::new(2.5476074, 2.3162842),
    ];
    let pp = PurePursuit::new(car, PATH, ppconf);
    devices.motor.drive(Backward(50)).unwrap();
    spawner.spawn(pure_pursuit(pp).unwrap());

    loop {
        // devices.btn2.wait_for_falling_edge().await;
        // let pos = devices.otos.get_pos().await.unwrap();
        // info!("{}", pos);
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
