#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    adc::{self, Adc, Channel},
    bind_interrupts,
    block::ImageDef,
    gpio::Pull,
};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());
    let mut adc_pin = Channel::new_pin(p.PIN_26, Pull::None);
    let mut adc = Adc::new(p.ADC, Irqs, Default::default());
    let mut v_adc: u16;
    let mut v: f32;
    let mut avg: f32 = 0.0;
    let mut n: u32 = 1;

    Timer::after_millis(1000).await; // Because of the cap the first like 300ms voltage is too low. Should we remove it? Or repace with smaller.
    //TODO: maybe test without cap somehow?

    // Does incremental averaging
    // The immediate result is like 0.02V noisy.
    loop {
        v_adc = adc.read(&mut adc_pin).await.expect("Adc failure");
        v = v_adc as f32 * (5.7 * (3.205 / 4095.0)); // Max accuracy tuned value.
        // 3.205 is pico voltage(tuned that, multimeter is not precise),
        // 5.7 is coefficient of voltage divider,
        // 4095.0 is max value of the 12bit adc
        avg += (v - avg) / n as f32;
        info!("{} {}", v, avg);
        n += 1;
        // Timer::after_millis(100).await;
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
