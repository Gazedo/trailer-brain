use core::fmt::Debug;

use defmt::{error, info};
use embedded_hal::blocking::i2c::{WriteRead, Write};
use lis2dh12::{Lis2dh12, SlaveAddr,  RawAccelerometer, I16x3, Error};
use micromath::F32Ext;
use rp_pico::hal::i2c::Error;

static SENS:f32 = 0.002;

static PI:f32 = 3.14159;

// here the "magic" conversion is generated
#[derive(Debug, Clone)]
pub struct Orientation {
    roll: f32,
    pitch: f32,
}
impl Default for Orientation {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
        }
    }
}
impl Orientation {
    pub fn new(roll: f32, pitch: f32) -> Self {
        Self { roll, pitch }
    }
    pub fn get_roll(&self) -> f32 {
        self.roll
    }
    pub fn get_pitch(&self) -> f32 {
        self.pitch
    }
}
impl From<I16x3> for Orientation{
    fn from(value: I16x3) -> Self {
        let x = (value.x >> 4) as f32 * SENS;
        let y = (value.y >> 4) as f32 * SENS;
        let z = (value.z >> 4) as f32 * SENS;
        
        let pitch = ((F32Ext::atan2(y, z) * 180_f32) / PI) + 180_f32;
        let roll = ((F32Ext::atan2(x, z) * 180_f32) / PI) + 180_f32;
        Self { roll, pitch}
    }
}
pub struct Imu<I2C> {
    dev: Lis2dh12<I2C>,
    prev_value: Orientation,
    k: f32,
}

impl<I2C, E> Imu<I2C>
where
I2C: WriteRead<Error = E> + Write<Error = E>,
E: Debug
{
    pub fn new(i2c:I2C) -> Result<Self, Error<E>> {
        let mut dev = match Lis2dh12::new(i2c, SlaveAddr::Alternative(false)){
            Ok(dev) => dev,
            Err(e) => {
                error!("Failed to create lisdh device");
                match e{
                    Lis2dh12::Error => error!("Error in lisdh device"),
                    Error => error!("Error in i2c bus")
                }
                return Err(e);
            }
        };
        dev.reset().unwrap();
        dev.enable_axis((true, true, true)).unwrap();
        dev.set_odr(lis2dh12::Odr::Hz25).unwrap();

        Ok(Self{dev, prev_value:Orientation::default(), k:0.5})
    }
    pub fn get_angles(&mut self) -> Orientation {
        let raw_angles = match self.dev.accel_raw() {
            Ok(vals) => vals,
            Err(_) => panic!("Could not read IMU"),
        };
        let calc_angles:Orientation = raw_angles.into();
        let angles = Orientation::new(
            (calc_angles.roll * (1_f32 - self.k)) + (self.prev_value.roll * self.k),
            (calc_angles.pitch * (1_f32 - self.k)) + (self.prev_value.pitch * self.k),
        );
        self.prev_value = angles.clone();
        info!("Got angles as roll:{} pitch:{}", angles.roll, angles.pitch);
        return angles;
    }
    pub fn available(&mut self) -> bool{
        match self.dev.get_stored_samples(){
            Ok(val) => if val > 0 {return true;} else {return false},
            Err(_) => {
                error!("Failed to get sample numbers");
                return false;
            }
        }
    }
}
