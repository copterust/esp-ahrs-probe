use embedded_hal_compat::eh0_2::blocking::delay::DelayMs;
use embedded_hal_compat::eh0_2::blocking::i2c::{Write, WriteRead};
use nalgebra::Vector3;

const HMC5883L_ADDRESS: u8 = 0x1E;
const REG_CONFIG_A: u8 = 0x00;
const REG_CONFIG_B: u8 = 0x01;
const REG_MODE: u8 = 0x02;
const REG_DATA_X_MSB: u8 = 0x03;

const CONFIG_A_AVG_8_15HZ: u8 = 0x70;
const CONFIG_B_GAIN_1_3_GA: u8 = 0x20;
const MODE_CONTINUOUS: u8 = 0x00;

const SCALE_UT_PER_LSB: f32 = 100.0 / 1090.0;

pub struct Hmc5883l<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Hmc5883l<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: HMC5883L_ADDRESS,
        }
    }
}

impl<I2C, E> Hmc5883l<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), E> {
        self.write_reg(REG_CONFIG_A, CONFIG_A_AVG_8_15HZ)?;
        self.write_reg(REG_CONFIG_B, CONFIG_B_GAIN_1_3_GA)?;
        self.write_reg(REG_MODE, MODE_CONTINUOUS)?;
        delay.delay_ms(10);
        Ok(())
    }

    pub fn read(&mut self) -> Result<Vector3<f32>, E> {
        let mut buf = [0u8; 6];
        self.read_regs(REG_DATA_X_MSB, &mut buf)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let z = i16::from_be_bytes([buf[2], buf[3]]) as f32;
        let y = i16::from_be_bytes([buf[4], buf[5]]) as f32;

        Ok(Vector3::new(
            x * SCALE_UT_PER_LSB,
            y * SCALE_UT_PER_LSB,
            z * SCALE_UT_PER_LSB,
        ))
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, val])
    }

    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.address, &[reg], buf)
    }
}
