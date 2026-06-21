#![no_std]

extern crate embassy_rp as hal;
use core::f32::{
    self,
    consts::{FRAC_PI_2, PI},
};

use defmt::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::{
    Peri, Peripherals,
    adc::{self, Adc, Channel},
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c},
    peripherals::{DMA_CH0, DMA_CH1, I2C0, I2C1, PIN_22, PWM_SLICE3, SPI1},
    pwm::{self, Pwm, SetDutyCycle},
    spi::{self, Spi},
    watchdog::Watchdog,
};
use map_range::MapRange;
use pixy2::Pixy2;
use sparkfun_otos::SparkfunOTOS;
use static_cell::StaticCell;
use tb6612fng::Motor;
use vl53l0x::VL53L0x;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

pub async fn init(p: Peripherals) -> Devices {
    static I2C1_BUS: StaticCell<Mutex<NoopRawMutex, I2c<'static, I2C1, i2c::Async>>> =
        StaticCell::new();
    let i2c1_bus = I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, i2c::Config::default());
    let i2c1_bus = I2C1_BUS.init(Mutex::new(i2c1_bus));

    let xshut_right = Output::new(p.PIN_2, Level::Low);
    let xshut_left = Output::new(p.PIN_4, Level::Low);
    let xshut_front = Output::new(p.PIN_0, Level::Low);

    let tof_right = VL53L0x::new(
        I2cDevice::new(i2c1_bus),
        Input::new(p.PIN_3, Pull::Up),
        xshut_right,
    )
    .init_with_address(0x52, Delay)
    .await
    .expect("Init right_dist");

    let tof_front = VL53L0x::new(
        I2cDevice::new(i2c1_bus),
        Input::new(p.PIN_5, Pull::Up),
        xshut_front,
    )
    .init_with_address(0x67, Delay)
    .await
    .expect("Init center_dist");

    let tof_left = VL53L0x::new(
        I2cDevice::new(i2c1_bus),
        Input::new(p.PIN_1, Pull::Up),
        xshut_left,
    )
    .init(Delay)
    .await
    .expect("Init left_dist");

    let i2c0_bus = I2c::new_async(p.I2C0, p.PIN_9, p.PIN_8, Irqs, Default::default());
    let mut otos = SparkfunOTOS::new(i2c0_bus, Input::new(p.PIN_10, Pull::None));
    otos.init().await.expect("Init otos failed");

    let mut spi_config = spi::Config::default();
    spi_config.polarity = spi::Polarity::IdleHigh;
    spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    spi_config.frequency = 8_000_000; // 8MHz is max safe beatiful value. Then zeros appear sometimes.
    let spi_bus = Spi::new(
        p.SPI1, p.PIN_14, p.PIN_15, p.PIN_12, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config,
    );
    let spi_dev = ExclusiveDevice::new(spi_bus, Output::new(p.PIN_13, Level::High), Delay)
        .expect("ExclusiveDevice creating failed");
    let mut pixy2 = Pixy2::new(spi_dev);
    pixy2.init().await.expect("Pixy2 init failure");

    let mut pwm_config = pwm::Config::default();
    pwm_config.top = 1499; //100kHz (TODO: Recheck this)
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE2, p.PIN_21, pwm_config);
    let bin1 = Output::new(p.PIN_19, Level::Low);
    let bin2 = Output::new(p.PIN_20, Level::Low);
    let motor_stby = Output::new(p.PIN_18, Level::High);
    let motor = Motor::new(bin1, bin2, motor_pwm).expect("Motor creation failed");

    let adc_pin = Channel::new_pin(p.PIN_26, Pull::None);
    let adc = Adc::new(p.ADC, Irqs, Default::default());
    let voltage = Voltage::new(adc, adc_pin);

    let servo = Servo::new(p.PWM_SLICE3, p.PIN_22);

    let btn1 = Input::new(p.PIN_11, Pull::Up);
    let btn2 = Input::new(p.PIN_27, Pull::Up);

    // Set up the watchdog driver - needed by the clock setup code
    let watchdog = Watchdog::new(p.WATCHDOG);

    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = PWM_DIV_INT.into();
    pwm_config.top = PWM_TOP;

    let buzzer = Pwm::new_output_a(p.PWM_SLICE6, p.PIN_28, pwm_config);

    Devices {
        pixy2,
        otos,
        tof_left,
        tof_front,
        tof_right,
        motor,
        motor_stby,
        voltage,
        servo,
        btn1,
        btn2,
        watchdog,
        buzzer,
    }
}

#[embassy_executor::task]
pub async fn btn_reset(mut btn: Input<'static>, mut watchdog: Watchdog) {
    btn.wait_for_low().await;
    watchdog.trigger_reset();
}

#[embassy_executor::task]
pub async fn motor_play(mut motor: XlncMotor) {
    loop {
        info!("Forward!");
        motor
            .drive(tb6612fng::DriveCommand::Forward(100))
            .expect("Drive motor");
        Timer::after_millis(4000).await;
        info!("Backward!");
        motor
            .drive(tb6612fng::DriveCommand::Backward(100))
            .expect("Drive motor");
        Timer::after_millis(4000).await;
    }
}

#[embassy_executor::task]
pub async fn motor_and_servo_play(mut motor: XlncMotor, mut servo: Servo) {
    loop {
        info!("Forward!");
        servo.set_pos_deg(30.0).unwrap();
        motor
            .drive(tb6612fng::DriveCommand::Forward(100))
            .expect("Drive motor");
        Timer::after_millis(500).await;
        servo.set_pos_deg(-30.0).unwrap();
        Timer::after_millis(500).await;
        info!("Backward!");
        servo.set_pos_deg(30.0).unwrap();
        motor
            .drive(tb6612fng::DriveCommand::Backward(100))
            .expect("Drive motor");
        Timer::after_millis(500).await;
        servo.set_pos_deg(-30.0).unwrap();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
pub async fn otos_print(mut otos: XlncOTOS) {
    loop {
        let mut pos = otos.get_pos().await.unwrap();
        pos.h *= 180.0 / PI;
        info!("{}", pos);
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
pub async fn play_song(mut buzzer: Pwm<'static>) {
    let mut pwm_config = pwm::Config::default();
    pwm_config.divider = PWM_DIV_INT.into();
    let song = tinytones::Tone::new(
        tinytones::songs::ode_to_joy::TEMPO,
        tinytones::songs::ode_to_joy::MELODY,
    );
    for (note, duration_type) in song.iter() {
        let top = get_top(note.freq_f64(), PWM_DIV_INT);
        pwm_config.top = top;
        buzzer.set_config(&pwm_config);

        let pause_duration = duration_type / 10; // 10% of note_duration

        buzzer
            .set_duty_cycle_percent(50)
            .expect("50 is valid duty percentage"); // Set duty cycle to 50% to play the note

        Timer::after_millis(duration_type - pause_duration).await; // Play 90%

        buzzer
            .set_duty_cycle_percent(0)
            .expect("50 is valid duty percentage"); // Stop tone
        Timer::after_millis(pause_duration).await; // Pause for 10%
    }
}

type Tof = VL53L0x<
    I2cDevice<'static, NoopRawMutex, I2c<'static, I2C1, i2c::Async>>,
    Input<'static>,
    Output<'static>,
>;
type XlncMotor = Motor<Output<'static>, Output<'static>, Pwm<'static>>;
type XlncPixy2 = Pixy2<ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, Delay>>;
type XlncOTOS = SparkfunOTOS<I2c<'static, I2C0, i2c::Async>, Input<'static>>;

pub struct Devices {
    pub pixy2: XlncPixy2,
    pub otos: XlncOTOS,
    pub tof_left: Tof,
    pub tof_front: Tof,
    pub tof_right: Tof,
    pub motor: XlncMotor,
    pub motor_stby: Output<'static>, // Has to be there so it won't be dropped on scope end.
    pub voltage: Voltage,
    pub servo: Servo,
    pub btn1: Input<'static>,
    pub btn2: Input<'static>,
    pub watchdog: Watchdog,
    pub buzzer: Pwm<'static>,
}

pub struct Voltage {
    adc: Adc<'static, adc::Async>,
    adc_pin: Channel<'static>,
}

impl Voltage {
    pub fn new(adc: Adc<'static, adc::Async>, adc_pin: Channel<'static>) -> Self {
        Voltage { adc, adc_pin }
    }
    pub async fn get(&mut self) -> Result<f32, adc::Error> {
        let raw = self.adc.read(&mut self.adc_pin).await?;
        Ok(raw as f32 * (5.7 * (3.205 / 4095.0)))
    }
}

pub struct Servo {
    pwm: Pwm<'static>,
}

impl Servo {
    pub fn new(pwm_slice: Peri<'static, PWM_SLICE3>, pin: Peri<'static, PIN_22>) -> Self {
        const PWM_DIV_INT: u8 = 64;
        const PWM_TOP: u16 = 46_874;
        let mut servo_config: pwm::Config = Default::default();
        servo_config.top = PWM_TOP;
        servo_config.divider = PWM_DIV_INT.into();
        Servo {
            pwm: Pwm::new_output_a(pwm_slice, pin, servo_config),
        }
    }
    pub fn set_pos_deg(&mut self, pos: f32) -> Result<(), pwm::PwmError> {
        self.set_pos_raw(pos.map_range(90.0..-90.0, 1180.0..7536.0) as u16)
    }
    pub fn set_pos_rad(&mut self, pos: f32) -> Result<(), pwm::PwmError> {
        self.set_pos_raw(pos.map_range(FRAC_PI_2..-FRAC_PI_2, 1180.0..7536.0) as u16)
    }
    /// 18..=115 / 1000 safe range 180degree
    pub fn set_pos_raw(&mut self, pos: u16) -> Result<(), pwm::PwmError> {
        // self.pwm.set_duty_cycle_fraction(pos, 1000)
        //scaled to max range is 1179.63==7536.525. Maybe adds more precision
        self.pwm.set_duty_cycle_fraction(pos, u16::MAX)
    }
}

pub const fn get_top(freq: f64, div_int: u8) -> u16 {
    assert!(div_int != 0, "Divider must not be 0");

    let result = 150_000_000. / (freq * div_int as f64);

    assert!(result >= 1.0, "Frequency too high");
    assert!(
        result <= 65535.0,
        "Frequency too low: TOP exceeds 65534 max"
    );

    result as u16 - 1
}

pub const PWM_DIV_INT: u8 = 64;
pub const PWM_TOP: u16 = get_top(440., PWM_DIV_INT);
