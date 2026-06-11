#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::{dbg, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::block::ImageDef;
use pixy2::packets::Signature;
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
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());
    let mut devices = init(p).await;
    // Timer::after_millis(600).await;
    // info!("Intialized! Press btn2 to start.");
    // info!("Voltage: {}", devices.voltage.get().await.unwrap());
    // devices.btn2.wait_for_low().await;
    let ver = devices.pixy2.get_version().await.unwrap();
    info!("{}", &ver);
    Timer::after_millis(600).await;

    loop {
        let blocks = devices
            .pixy2
            .get_blocks(Signature::SIG_ALL, 10)
            .await
            .expect("Get blocks");
        info!("Got {} blocks", blocks.len());
        // Works too: for block in block
        dbg!(&blocks);
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
