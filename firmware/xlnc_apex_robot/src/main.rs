#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    block::ImageDef,
    pwm::{self, Pwm, SetDutyCycle as _},
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
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());
    const PWM_DIV_INT: u8 = 64;
    const PWM_TOP: u16 = 46_874;
    let mut servo_config: pwm::Config = Default::default();
    servo_config.top = PWM_TOP;
    servo_config.divider = PWM_DIV_INT.into();

    let mut servo = Pwm::new_output_a(p.PWM_SLICE3, p.PIN_22, servo_config);

    loop {
        // Move servo to 0° position (5 % duty cycle = 50/1000)
        servo
            .set_duty_cycle_fraction(50, 1000)
            .expect("invalid min duty cycle");

        Timer::after_millis(1500).await;

        // 90° position (7.5 % duty cycle)
        servo
            .set_duty_cycle_fraction(75, 1000)
            .expect("invalid half duty cycle");

        Timer::after_millis(1500).await;

        // 180° position (10% duty cycle)
        servo
            .set_duty_cycle_fraction(100, 1000)
            .expect("invalid max duty cycle");

        Timer::after_millis(1500).await;

        // 90° position (7.5 % duty cycle)
        servo
            .set_duty_cycle_fraction(75, 1000)
            .expect("invalid half duty cycle");

        Timer::after_millis(1500).await;
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
