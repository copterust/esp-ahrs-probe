use esp_hal::gpio::{Output, OutputConfig, OutputPin};

struct Gy86Pins<'d> {
    vcc_in: Output<'d>,
    v3_3: Output<'d>,
    gnd: Output<'d>,
}

pub struct Gy86Module<'d> {
    pins: Gy86Pins<'d>,
}

impl<'d> Gy86Module<'d> {
    pub fn new(
        vcc_in_pin: impl OutputPin + 'd,
        v3_3_pin: impl OutputPin + 'd,
        gnd_pin: impl OutputPin + 'd,
    ) -> Self {
        let vcc_in = Output::new(
            vcc_in_pin,
            esp_hal::gpio::Level::Low,
            OutputConfig::default(),
        );
        let v3_3 = Output::new(v3_3_pin, esp_hal::gpio::Level::Low, OutputConfig::default());
        let gnd = Output::new(gnd_pin, esp_hal::gpio::Level::Low, OutputConfig::default());

        let pins = Gy86Pins { vcc_in, v3_3, gnd };
        Gy86Module { pins }
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
}
