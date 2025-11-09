#![no_std]
#![no_main]

use defmt::info;
use esp_ahrs_probe::sensors::Gy86Module;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Io;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::main;
use esp_hal::time::{Duration, Instant};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    info!("PANIC: {:?}", info);
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    rtt_target::rtt_init_defmt!();

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

    let mut sensors = Gy86Module::new(
        peripherals.GPIO22,
        peripherals.GPIO21,
        peripherals.GPIO20,
        i2c,
    );

    sensors.power_on();
    info!("GY-86 power enabled");

    info!("Waiting for module startup (500ms)...");
    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(500) {}

    info!("Power stabilization complete");
    sensors.probe();

    loop {
        sensors.power_on();
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
        sensors.power_off();
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }
}
