#![no_std]

extern crate embassy_rp as hal;
use core::{
    f32::{
        self,
        consts::{FRAC_PI_2, PI},
    },
    str::from_utf8,
};

use cyw43::{JoinOptions, aligned_bytes};
use cyw43_pio::{DEFAULT_CLOCK_DIVIDER, PioSpi};
use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_net::{StackResources, tcp::TcpSocket};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, mutex::Mutex};
use embassy_time::{Delay, Duration, Timer};
use embedded_io_async::Write;
// use embedded_hal_bus::spi::ExclusiveDevice;
use hal::{
    Peri, Peripherals,
    adc::{self, Adc, Channel},
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c},
    peripherals::{DMA_CH0, DMA_CH1, DMA_CH2, I2C0, I2C1, PIN_22, PIO0, PWM_SLICE3},
    pio::{self, Pio},
    pwm::{self, Pwm, SetDutyCycle},
    watchdog::Watchdog,
};
use map_range::MapRange;
// use pixy2::Pixy2;
use sparkfun_otos::SparkfunOTOS;
use static_cell::StaticCell;
use tb6612fng::Motor;
use vl53l0x::VL53L0x;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>, dma::InterruptHandler<DMA_CH2>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

const WIFI_NETWORK: &str = "ssid"; // change to your network SSID
const WIFI_PASSWORD: &str = "pwd"; // change to your network password

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

pub async fn init(p: Peripherals, sp: &Spawner) -> Devices {
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

    // let mut spi_config = spi::Config::default();
    // spi_config.polarity = spi::Polarity::IdleHigh;
    // spi_config.phase = spi::Phase::CaptureOnSecondTransition;
    // spi_config.frequency = 8_000_000; // 8MHz is max safe beatiful value. Then zeros appear sometimes.
    // let spi_bus = Spi::new(
    //     p.SPI1, p.PIN_14, p.PIN_15, p.PIN_12, p.DMA_CH0, p.DMA_CH1, Irqs, spi_config,
    // );
    // let spi_dev = ExclusiveDevice::new(spi_bus, Output::new(p.PIN_13, Level::High), Delay)
    //     .expect("ExclusiveDevice creating failed");
    // let mut pixy2 = Pixy2::new(spi_dev);
    // pixy2.init().await.expect("Pixy2 init failure");

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

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH2, Irqs),
    );

    cyw43_init(pwr, spi, sp).await;

    Devices {
        // pixy2,
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

async fn cyw43_init(pwr: Output<'static>, spi: PioSpi<'static, PIO0, 0>, sp: &Spawner) {
    let mut rng = hal::clocks::RoscRng;

    let fw = aligned_bytes!("../assets/cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../assets/cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../assets/cyw43-firmware/nvram_rp2040.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --binary-format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs download 43439A0_clm.bin --binary-format bin --chip RP2040 --base-address 0x10140000
    //let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };
    //let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    sp.spawn(cyw43_task(runner).unwrap());

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let config = embassy_net::Config::dhcpv4(Default::default());
    //let config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    sp.spawn(net_task(runner).unwrap());

    while let Err(err) = control
        .join(WIFI_NETWORK, JoinOptions::new(WIFI_PASSWORD.as_bytes()))
        .await
    {
        info!("join failed: {:?}", err);
    }

    info!("waiting for link...");
    stack.wait_link_up().await;

    info!("waiting for DHCP...");
    stack.wait_config_up().await;

    // And now we can use it!
    info!("Stack is up!");

    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];
    let mut buf = [0; 4096];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        control.gpio_set(0, false).await;
        info!("Listening on TCP:1234...");
        if let Err(e) = socket.accept(1234).await {
            warn!("accept error: {:?}", e);
            continue;
        }

        info!("Received connection from {:?}", socket.remote_endpoint());
        control.gpio_set(0, true).await;

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };

            info!("rxd {}", from_utf8(&buf[..n]).unwrap());

            match socket.write_all(&buf[..n]).await {
                Ok(()) => {}
                Err(e) => {
                    warn!("write error: {:?}", e);
                    break;
                }
            };
        }
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
// type XlncPixy2 = Pixy2<ExclusiveDevice<Spi<'static, SPI1, spi::Async>, Output<'static>, Delay>>;
type XlncOTOS = SparkfunOTOS<I2c<'static, I2C0, i2c::Async>, Input<'static>>;

pub struct Devices {
    // pub pixy2: XlncPixy2,
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
