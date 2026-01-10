#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use defmt::{info, Debug2Format};
use embedded_hal_compat::eh0_2::blocking::delay::DelayMs;
use embedded_hal_compat::{eh0_2, eh1_0, ReverseCompat};
use esp_ahrs_probe::ahrs::AhrsUkf;
use esp_ahrs_probe::config::MagCalibration;
use esp_ahrs_probe::hmc5883l::Hmc5883l;
use esp_ahrs_probe::sensors::Gy86Module;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::Io;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use mpu6050::Mpu6050;
use nalgebra::Vector3;
use shared_bus::BusManagerSimple;

const GRAVITY: f32 = 9.807;

enum Magnetometer<I2C> {
    Hmc(Hmc5883l<I2C>),
}

struct DelayMsCompat<T> {
    inner: T,
}

impl<T> DelayMsCompat<T> {
    fn new(inner: T) -> Self {
        Self { inner }
    }
}

impl<T> eh0_2::blocking::delay::DelayMs<u8> for DelayMsCompat<T>
where
    T: eh1_0::delay::DelayNs,
{
    fn delay_ms(&mut self, ms: u8) {
        self.inner.delay_ms(ms as u32);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    info!("PANIC: {:?}", info);
    loop {}
}

#[alloc_error_handler]
fn alloc_error(layout: core::alloc::Layout) -> ! {
    info!("ALLOC ERROR: {:?}", Debug2Format(&layout));
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

    esp_alloc::heap_allocator!(size: 32 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    info!("ESP32-C6 AHRS Probe starting...");

    // Initialize GPIO
    let _io = Io::new(peripherals.IO_MUX);

    let i2c_config = I2cConfig::default();
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .unwrap()
        .with_sda(peripherals.GPIO18)
        .with_scl(peripherals.GPIO19);

    let mut sensors = Gy86Module::new(peripherals.GPIO22, peripherals.GPIO21, peripherals.GPIO20);

    sensors.power_on();
    info!("GY-86 power enabled");

    info!("Waiting for module startup (500ms)...");
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(500) {}

    info!("Power stabilization complete");

    let bus = BusManagerSimple::new(i2c.reverse());

    let mut mpu_delay = DelayMsCompat::new(Delay::new());
    let mut mpu = Mpu6050::new(bus.acquire_i2c());
    if let Err(err) = mpu.init(&mut mpu_delay) {
        info!("MPU init failed: {:?}", Debug2Format(&err));
        loop {}
    }
    if let Err(err) = mpu.write_byte(0x6A, 0x00) {
        info!("MPU user ctrl reset failed: {:?}", Debug2Format(&err));
        loop {}
    }
    if let Err(err) = mpu.write_byte(0x37, 0x02) {
        info!("MPU bypass enable failed: {:?}", Debug2Format(&err));
        loop {}
    }
    mpu_delay.delay_ms(10);

    let mut mag = Hmc5883l::new(bus.acquire_i2c());
    let mut mag = match mag.init(&mut mpu_delay) {
        Ok(()) => {
            info!("HMC5883L detected");
            Magnetometer::Hmc(mag)
        }
        Err(err) => {
            info!("HMC5883L init failed ({:?})", Debug2Format(&err));
            loop {}
        }
    };

    let mag_cal = MagCalibration::new();
    let mut ahrs = AhrsUkf::new();
    let mut last_tick = Instant::now();
    let interval = Duration::from_millis(10);

    loop {
        let elapsed = last_tick.elapsed();
        if elapsed < interval {
            continue;
        }
        last_tick = Instant::now();
        let dt = (elapsed.as_micros() as f32) * 1e-6;

        let accel_g = match mpu.get_acc() {
            Ok(accel) => accel,
            Err(err) => {
                info!("MPU accel read failed: {:?}", Debug2Format(&err));
                continue;
            }
        };
        let gyro_raw = match mpu.get_gyro() {
            Ok(gyro) => gyro,
            Err(err) => {
                info!("MPU gyro read failed: {:?}", Debug2Format(&err));
                continue;
            }
        };
        let mag_field = match &mut mag {
            Magnetometer::Hmc(dev) => match dev.read() {
                Ok(mag) => mag,
                Err(err) => {
                    info!("HMC5883L read failed: {:?}", Debug2Format(&err));
                    continue;
                }
            },
        };
        let mag_field = mag_cal.apply(mag_field);

        let accel = Vector3::new(
            accel_g.x * GRAVITY,
            accel_g.y * GRAVITY,
            accel_g.z * GRAVITY,
        );
        let gyro = Vector3::new(gyro_raw.x, gyro_raw.y, gyro_raw.z);

        if let Err(err) = ahrs.predict(dt, Some(gyro)) {
            info!("UKF predict failed: {:?}", Debug2Format(&err));
        }
        match ahrs.update(accel, mag_field) {
            Ok(true) => {}
            Ok(false) => {
                info!("Skipping UKF update: sensor norm is too small");
            }
            Err(err) => {
                info!("UKF update failed: {:?}", Debug2Format(&err));
            }
        }

        let attitude = ahrs.attitude();
        let quat = attitude.as_ref();

        info!(
            "Quat w={:?} x={:?} y={:?} z={:?} dt={=f32}",
            quat.coords.w, quat.coords.x, quat.coords.y, quat.coords.z, dt
        );
    }
}
