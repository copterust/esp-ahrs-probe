use defmt::info;
use esp_hal::gpio::{Output, OutputConfig, OutputPin};
use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;

struct Gy86Pins<'d> {
    vcc_in: Output<'d>,
    v3_3: Output<'d>,
    gnd: Output<'d>,
}

pub struct Gy86Module<'d> {
    pins: Gy86Pins<'d>,
    i2c: I2c<'d, Blocking>,
}

impl<'d> Gy86Module<'d> {
    pub fn new(
        vcc_in_pin: impl OutputPin + 'd,
        v3_3_pin: impl OutputPin + 'd,
        gnd_pin: impl OutputPin + 'd,
        i2c: I2c<'d, Blocking>,
    ) -> Self {
        let vcc_in = Output::new(
            vcc_in_pin,
            esp_hal::gpio::Level::Low,
            OutputConfig::default(),
        );
        let v3_3 = Output::new(v3_3_pin, esp_hal::gpio::Level::Low, OutputConfig::default());
        let gnd = Output::new(gnd_pin, esp_hal::gpio::Level::Low, OutputConfig::default());

        let pins = Gy86Pins { vcc_in, v3_3, gnd };
        Gy86Module { pins, i2c }
    }

    pub fn power_on(&mut self) {
        self.pins.gnd.set_low();
        self.pins.v3_3.set_high();
        self.pins.vcc_in.set_high();
    }

    pub fn power_off(&mut self) {
        self.pins.vcc_in.set_low();
        self.pins.v3_3.set_low();
        self.pins.gnd.set_low();
    }

    pub fn probe(&mut self) {
        let mpu_addr = 0x68u8;
        let whoami_reg = 0x75u8;
        let mut buf = [0u8; 1];
        let res = self.i2c.write_read(mpu_addr, &[whoami_reg], &mut buf);
        match res {
            Ok(_) => info!("MPU6500 WHO_AM_I: 0x{:02x}", buf[0]),
            Err(_) => info!("No response from MPU6500"),
        }

        let baro_addr = 0x77u8;
        let prom_cmd = 0xA0u8;
        let mut buf = [0u8; 2];
        let res = self.i2c.write_read(baro_addr, &[prom_cmd], &mut buf);
        match res {
            Ok(_) => info!("MS5611 PROM0: 0x{:02x}{:02x}", buf[0], buf[1]),
            Err(_) => info!("No response from barometer"),
        }
    }
}
