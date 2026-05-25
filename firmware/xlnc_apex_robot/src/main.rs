#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::{dbg, info, println};
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use hal::{
    bind_interrupts,
    block::ImageDef,
    gpio::{Input, Level, Output, Pull},
    i2c,
    peripherals::{self, I2C0},
};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;
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
    static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::I2c<'static, I2C0, i2c::Async>>> =
        StaticCell::new();
    let i2c_bus = i2c::I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c::Config::default());
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c_bus));
    info!("bb");
    dbg!("DA");
    defmt::debug!("DA");
    println!("DA"); //

    let mut left_dist = VL53L0x::new(
        I2cDevice::new(i2c_bus),
        Input::new(p.PIN_3, Pull::Up),
        Output::new(p.PIN_2, Level::Low),
    )
    .init(Delay)
    .await
    .expect("Init left_dist");
    info!("inited");
    left_dist.set_address(0x67).await.expect("Address to 0x67 not set(((");

    let mut right_dist = VL53L0x::new(
        I2cDevice::new(i2c_bus),
        Input::new(p.PIN_5, Pull::Up),
        Output::new(p.PIN_4, Level::Low),
    )
    .init_with_address(0x52, Delay)
    .await
    .expect("Init right_dist");

    left_dist
        .start_continuous(100)
        .await
        .expect("Cannot start continuous");
    right_dist
        .start_continuous(100)
        .await
        .expect("Cannot start continuous");
    info!("cont started");

    for i in 0..100 {
        let l = left_dist
            .read_range_mm()
            .await
            .expect("Couldn't read range");
        let r = right_dist
            .read_range_mm()
            .await
            .expect("Couldn't read range");
        info!("{}: {} {}", i, l, r);
    }
    left_dist
        .stop_continuous()
        .await
        .expect("Cannot stop continuous");
    right_dist
        .stop_continuous()
        .await
        .expect("Cannot stop continuous");
    info!("bb");

    loop {
        Timer::after_millis(100).await;
    }
}

// type Dist = VL53L0x<I2cDevice<'static, NoopRawMutex, i2c::I2c<'static, I2C0, i2c::Async>>, Input<'static>, Output<'static>>;

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
