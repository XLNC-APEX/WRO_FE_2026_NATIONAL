#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use embassy_executor::Spawner;
use embassy_futures::select::{self, Either, select};
use embassy_time::Timer;
use hal::{
    block::ImageDef,
    gpio::{Input, Pull},
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

    let mut b1 = Input::new(p.PIN_11, Pull::Up);
    let mut b2 = Input::new(p.PIN_27, Pull::Up);

    let mut is_b1: bool;
    match select(b1.wait_for_low(), b2.wait_for_low()).await {
        Either::First(_) => {
            servo
                .set_duty_cycle_fraction(50, 1000)
                .expect("invalid max duty cycle");
            is_b1 = true;
        }
        Either::Second(_) => {
            servo
                .set_duty_cycle_fraction(100, 1000)
                .expect("invalid max duty cycle");
            is_b1 = false;
        }
    }

    loop {
        match select(b1.wait_for_low(), b2.wait_for_low()).await {
            Either::First(_) => {
                if !is_b1 {
                    servo
                        .set_duty_cycle_fraction(50, 1000)
                        .expect("invalid max duty cycle");
                    is_b1 = true;
                }
            }
            Either::Second(_) => {
                if is_b1 {
                    servo
                        .set_duty_cycle_fraction(100, 1000)
                        .expect("invalid max duty cycle");
                    is_b1 = false;
                }
            }
        }
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
