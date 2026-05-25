#![no_std]
#![no_main]

extern crate embassy_rp as hal;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::{
    bind_interrupts,
    block::ImageDef,
    gpio::{Level, Output},
    peripherals::PIO0,
    pio::Pio,
    pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram},
    pwm::{self, Pwm},
};

//Panic Handler
use panic_probe as _;
// Defmt Logging
use defmt_rtt as _;
use tb6612fng::Motor;

/// Tell the Boot ROM about our application
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = hal::block::ImageDef::secure_exe();

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => hal::pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = hal::init(Default::default());
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    let prg = PioEncoderProgram::new(&mut common);
    let mut encoder = PioEncoder::new(&mut common, sm0, p.PIN_8, p.PIN_9, &prg);

    let mut pwm_config = pwm::Config::default();
    pwm_config.top = 1499;
    // info!("{}", pwm_config.top);
    // loop {
    //     Timer::after_millis(100).await;
    // }
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE6, p.PIN_13, pwm_config);
    let ain2 = Output::new(p.PIN_15, Level::Low);
    let ain1 = Output::new(p.PIN_14, Level::Low);
    let _stby = Output::new(p.PIN_12, Level::High);
    let motor = Motor::new(ain1, ain2, motor_pwm).expect("Motor creation failed");

    spawner.spawn(motor_play(motor).expect("Spawn task failed"));

    let mut c = 0;
    loop {
        c += match encoder.read().await {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        info!("{}", c);
    }
}

#[embassy_executor::task]
async fn motor_play(mut motor: Motor<Output<'static>, Output<'static>, Pwm<'static>>) {
    // motor
    //     .drive(tb6612fng::DriveCommand::Forward(20))
    //     .expect("Drive motor");
    // Timer::after_millis(4000).await;
    loop {
        info!("Forward!");
        motor
            .drive(tb6612fng::DriveCommand::Forward(100))
            .expect("Drive motor");
        Timer::after_millis(2000).await;
        info!("Backward!");
        motor
            .drive(tb6612fng::DriveCommand::Backward(100))
            .expect("Drive motor");
        Timer::after_millis(2000).await;
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
