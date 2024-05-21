use defmt::{error, info};
use embedded_hal::adc::Channel;
use embedded_hal::digital::{InputPin, OutputPin};
use rp_pico::hal::adc::AdcPin;
use rp_pico::hal::gpio::AnyPin;

#[derive(Debug)]
pub struct BtsSwitch {
    en_pin_a: OutputPin,
    en_pin_b: OutputPin,
    d_selpin: OutputPin,
    d_enpin: OutputPin,
    i_spin: AdcPin<AnyPin>,
    state: bool,
}

#[derive(Debug, Copy, Clone)]
pub enum BtsError{

}

impl BtsSwitch {
    pub fn new(
        en_pin_a: OutputPin,
        en_pin_b: OutputPin,
        d_selpin: OutputPin,
        d_enpin: OutputPin,
        i_spin: AdcPin<AnyPin>,
    ) {
        Self{en_pin_a, en_pin_b, d_selpin, d_enpin, i_spin, state:false}
    }
    pub fn toggle(){

    }
    pub fn getCurrent(){}
    pub fn getStatus() -> Result<Ok, BtsError>{
        Err()
    }
}
