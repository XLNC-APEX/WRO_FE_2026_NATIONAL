#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::{dbg, info};
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
    Timer::after_millis(600).await;
    dbg!(devices.voltage.get().await.unwrap());

    let mut tofs = [devices.tof_left, devices.tof_center, devices.tof_right];

    info!("Intialized! Press btn2 to start.");
    // avg = [138, 91, 110]
    // avg = [231, 183, 199]
    // avg = [193, 144, 171] 140mm ~ish

    // 135.15 − [193  144  171] = [−1157/20  −177/20  −717/20] = [−57.85  −8.85  −35.85]
    // [-57850  -8850  -35850] left center right
    loop {
        devices.btn2.wait_for_low().await;
        // let (left, center, right) = (devices.tof_left, devices.tof_center, devices.tof_right);
        // for tof in &mut tofs {
        //     tof.start_continuous(0).await.unwrap();
        //     info!("{}", &tof.read_range_mm().await.unwrap());
        //     tof.stop_continuous().await.unwrap();
        // }
        for tof in &mut tofs {
            tof.start_continuous(0).await.unwrap();
            // info!("{}", &tof.read_range_mm().await.unwrap());
            // tof.stop_continuous().await.unwrap();
        }
        let mut avg = [0u16; 3];
        for i in 0..3 {
            devices.btn2.wait_for_low().await;
            for _ in 0..32 {
                avg[i] += &tofs[i].read_range_mm().await.unwrap();
                // info!(
                //     "{} {} {}",
                //     &tofs[0].read_single_range_mm().await.unwrap(),
                //     &tofs[1].read_single_range_mm().await.unwrap(),
                //     &tofs[2].read_single_range_mm().await.unwrap()
                // );
                // info!(
                //     "{} {} {}",
                //     &tofs[0].read_range_mm().await.unwrap(),
                //     &tofs[1].read_range_mm().await.unwrap(),
                //     &tofs[2].read_range_mm().await.unwrap()
                // );

                // for tof in &mut tofs {
                //     info!("{}", &tof.read_single_range_mm().await.unwrap());
                // }
            }
            avg[i] /= 32;
            dbg!(avg[i]);
        }

        dbg!(avg);

        for tof in &mut tofs {
            // tof.start_continuous(0).await.unwrap();
            // info!("{}", &tof.read_range_mm().await.unwrap());
            tof.stop_continuous().await.unwrap();
        }

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
