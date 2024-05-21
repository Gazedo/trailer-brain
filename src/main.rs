#![no_std]
#![no_main]

//#Todo
// Fix IMU to it smooths
// Move to using and reusing a single string buffer?

use core::{
    f32::consts::PI,
    fmt::{Debug, Write},
};

use cortex_m::delay::Delay;

use display_interface::DisplayError;
use embedded_hal::digital::v2::InputPin;

// Debug and panic stuff
use defmt::{error, info, warn};
use defmt_rtt as _;
use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Point, Size},
    mono_font::{ascii::FONT_6X10, iso_8859_14::FONT_4X6, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    primitives::Rectangle,
    text::{renderer::TextRenderer, Baseline, Text},
    Drawable,
};
use fugit::RateExtU32;
use heapless::Vec;
use mpu6050::Mpu6050;
use panic_halt as _;
use pcf857x::{OutputPin, Pcf8575};
use rp2040_hal::gpio;
use rp_pico::{
    hal::{
        clocks,
        // multicore::{Multicore, Stack},
        // uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock,
        Sio,
        Watchdog,
        I2C,
    },
    pac::{CorePeripherals, Peripherals},
};
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};

// use crate::renogy::SmartBattery;

// mod imu;
// mod renogy;
// mod bts7090;

// static mut CORE1_STACK: Stack<4096> = Stack::new();

#[derive(Debug)]
struct Acceleration {
    roll: f32,
    pitch: f32,
}

static ACC_ZERO: Acceleration = Acceleration { roll: -0.2, pitch: -178.5 };

// static Q_IMU: Q4<Acceleration> = Q4::new();
// static Q_KITCHEN: Q4<bool> = Q4::new();
// static Q_STOVE: Q4<bool> = Q4::new();
// static Q_WATER: Q4<bool> = Q4::new();
// static Q_FRIDGE: Q4<bool> = Q4::new();

// Display Helpers
/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
struct FmtBuf {
    buf: [u8; 24],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self { buf: [0; 24], ptr: 0 }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() { rest_len } else { s.len() };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
type Display = Ssd1306<
    I2CInterface<
        I2C<
            rp_pico::pac::I2C1,
            (
                gpio::Pin<gpio::bank0::Gpio18, gpio::FunctionI2c, gpio::PullDown>,
                gpio::Pin<gpio::bank0::Gpio19, gpio::FunctionI2c, gpio::PullDown>,
            ),
        >,
    >,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;
type TextStyle<'a> = embedded_graphics::mono_font::MonoTextStyle<'a, BinaryColor>;

fn parse_de(e: DisplayError) -> usize {
    match e {
        DisplayError::InvalidFormatError => 0,
        DisplayError::BusWriteError => 1,
        DisplayError::DCError => 2,
        DisplayError::CSError => 3,
        DisplayError::DataFormatNotImplemented => 4,
        DisplayError::RSError => 5,
        DisplayError::OutOfBoundsError => 6,
        _ => todo!(),
    }
}

fn flush(display: &mut Display) {
    display.flush().unwrap_or_else(|e| {
        error!("Failed to write to screen with {}!", parse_de(e));
        // display.clear_buffer();
    });
}

// fn core1_task<UART>(clock_freq: u32, mut display: Display, uart: UART) {
// fn core1_task(clock_freq: u32, mut display: Display) {
//     // Resources
//     // uart0
//     // i2c1
//     // Check Battery
//     // Update GUI if needed

//     // use defmt::info;
//     // use defmt_rtt as _;
//     let core = unsafe { CorePeripherals::steal() };
//     let mut delay = Delay::new(core.SYST, clock_freq);
//     info!("Running Core 1");
//     let text_style =
//         MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build();
//     display.clear(BinaryColor::Off).unwrap();
//     Text::with_baseline("Started second core", Point::new(0, 16), text_style, Baseline::Top)
//         .draw(&mut display)
//         .unwrap();
//     flush(&mut display);

//     display.clear(BinaryColor::Off).unwrap_or_else(|e| error!("Failed to clear screen with error {:?}", parse_de(e)));
//     let mut str_buff = FmtBuf::new();
//     info!("Starting Core 1 loop");
//     loop {
//         while let Some(state) = Q_KITCHEN.dequeue() {
//             info!("Got Kitchen {}", state);
//             str_buff.reset();
//             write!(str_buff, "Kitchen: {}", state).unwrap();
//             // imu_str.write_str("IMU r {}  p {}", acc.roll);
//             display
//                 .fill_solid(&Rectangle::new(Point::zero(), Size::new(128, 10)), BinaryColor::Off)
//                 .unwrap();
//             Text::with_baseline(str_buff.as_str(), Point::zero(), text_style, Baseline::Top)
//                 .draw(&mut display)
//                 .unwrap();
//             flush(&mut display);
//         }
//         while let Some(state) = Q_STOVE.dequeue() {
//             info!("Got Stove {}", state);
//             str_buff.reset();
//             write!(str_buff, "Stove: {}", state).unwrap();
//             // imu_str.write_str("IMU r {}  p {}", acc.roll);
//             display
//                 .fill_solid(
//                     &Rectangle::new(Point::new(0, 16), Size::new(128, 10)),
//                     BinaryColor::Off,
//                 )
//                 .unwrap();
//             Text::with_baseline(str_buff.as_str(), Point::new(0, 16), text_style, Baseline::Top)
//                 .draw(&mut display)
//                 .unwrap();
//             flush(&mut display);
//         }
//         while let Some(state) = Q_WATER.dequeue() {
//             info!("Got Water {}", state);
//             str_buff.reset();
//             write!(str_buff, "Water: {}", state).unwrap();
//             // imu_str.write_str("IMU r {}  p {}", acc.roll);
//             display
//                 .fill_solid(
//                     &Rectangle::new(Point::new(0, 32), Size::new(128, 10)),
//                     BinaryColor::Off,
//                 )
//                 .unwrap();
//             Text::with_baseline(str_buff.as_str(), Point::new(0, 32), text_style, Baseline::Top)
//                 .draw(&mut display)
//                 .unwrap();
//             flush(&mut display);
//         }
//         while let Some(acc) = Q_IMU.dequeue() {
//             info!("Got {}x{} in core 1", acc.roll, acc.pitch);
//             str_buff.reset();
//             write!(str_buff, "IMU r{0:.2}  p{1:.2}", acc.roll, acc.pitch).unwrap_or_else(|_| warn!("Failed to fill in string buffer"));
//             // imu_str.write_str("IMU r {}  p {}", acc.roll);
//             display
//                 .fill_solid(
//                     &Rectangle::new(Point::new(0, 48), Size::new(128, 10)),
//                     BinaryColor::Off,
//                 )
//                 .unwrap();
//             Text::with_baseline(str_buff.as_str(), Point::new(0, 48), text_style, Baseline::Top)
//                 .draw(&mut display)
//                 .unwrap();
//             flush(&mut display);
//         }
//         delay.delay_ms(25);
//     }
// }

#[rp_pico::entry]
fn main() -> ! {
    // Setup

    info!("Starting Setup");
    // -------- Setup peripherials --------
    let mut pac = Peripherals::take().unwrap();
    let core = CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Multicore set up
    info!("Starting Multicore");
    let sio = Sio::new(pac.SIO);
    // let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    // let cores = mc.cores();
    // let core1 = &mut cores[1];

    let clocks = clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    // Starting i2c
    info!("Setting up I2C");
    let sda_pin: gpio::Pin<_, gpio::FunctionI2C, gpio::PullUp> = pins.gpio4.reconfigure();
    let scl_pin: gpio::Pin<_, gpio::FunctionI2C, gpio::PullUp> = pins.gpio5.reconfigure();
    let i2c =
        I2C::i2c0(pac.I2C0, sda_pin, scl_pin, 100.kHz(), &mut pac.RESETS, &clocks.system_clock);
    let i2c: &'static _ = shared_bus::new_cortexm!(I2C<rp_pico::pac::I2C0, (gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionI2c, gpio::PullUp>)> = i2c).unwrap();
    // let i2c = shared_bus::CortexMMutex::new(i2c);

    // Create the I²C display interface:
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio18.into_function::<gpio::FunctionI2C>();
    let scl_pin = pins.gpio19.into_function::<gpio::FunctionI2C>();
    info!("Setting up I2C Disp");
    let i2c_disp =
        I2C::i2c1(pac.I2C1, sda_pin, scl_pin, 150.kHz(), &mut pac.RESETS, &clocks.peripheral_clock);
    let interface = ssd1306::I2CDisplayInterface::new(i2c_disp);
    let mut display: Display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display
        .init()
        .unwrap_or_else(|e| error!("Failed to init display with error {:?}", parse_de(e)));
    info!("Screen Started");
    let text_style =
        MonoTextStyleBuilder::new().font(&FONT_6X10).text_color(BinaryColor::On).build();

    Text::with_baseline("TrailerBrain v5!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    flush(&mut display);
    // Buttons
    let but0 = pins.gpio20.into_pull_up_input();
    let but1 = pins.gpio9.into_pull_up_input();
    let but2 = pins.gpio8.into_pull_up_input();
    let mut kitchen_button = TrailerButton::new(but0);
    let mut stove_button = TrailerButton::new(but1);
    let mut water_button = TrailerButton::new(but2);
    // Set up PFC
    let address = pcf857x::SlaveAddr::Alternative(true, true, true);
    let mut expander = Pcf8575::new(i2c.acquire_i2c(), address);
    let bits = 0b0000000000000000;
    let bits = 0b1111111111111111;
    if let Err(_) = expander.set(bits) {
        error!("Failed to set expander bits")
    }
    let pcf_pins = expander.split();
    let mut kitchen = TrailerOutput::new(pcf_pins.p17);
    let mut stove = TrailerOutput::new(pcf_pins.p14);
    let mut water = TrailerOutput::new(pcf_pins.p3);
    // Q_KITCHEN.enqueue(kitchen.state).unwrap_or_else(|_| warn!("Failed to enqueue"));
    // Q_STOVE.enqueue(stove.state).unwrap_or_else(|_| warn!("Failed to enqueue"));
    // Q_WATER.enqueue(water.state).unwrap_or_else(|_| warn!("Failed to enqueue"));
    Text::with_baseline("Buttons: Success", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    flush(&mut display);
    // UART
    // info!("Setting up uart for modbus rtu");
    // let uart = rp2040_hal::
    // let uart_pins = (
    //     // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
    //     pins.gpio0.into_function(),
    //     // UART RX (characters received by RP2040) on pin 2 (GPIO1)
    //     pins.gpio1.into_function(),
    // );
    // let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
    //     .enable(
    //         UartConfig::new(9600.Hz(), DataBits::Six, None, StopBits::One),
    //         clocks.peripheral_clock.freq(),
    //     )
    //     .unwrap();

    // let re = pins.gpio15.into_push_pull_output();
    // let tx_pin: gpio::Pin<_, gpio::FunctionUart, gpio::PullNone> = pins.gpio0.reconfigure();
    // let rx_pin: gpio::Pin<_, gpio::FunctionUart, gpio::PullNone> = pins.gpio1.reconfigure();
    // let uart = UartPeripheral::new(pac.UART0, (tx_pin, rx_pin), &mut pac.RESETS)
    //     .enable(
    //         UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
    //         clocks.peripheral_clock.freq(),
    //     )
    //     .unwrap();
    // let mut ren = SmartBattery::new(uart, re, 0x30, delay);
    // let soc = ren.get_soc();
    // info!("Battery is at {}", soc);
    // let str_soc:heapless::String<25>;
    // write!(str_soc, "Found Battery! SOC {}", soc);
    Text::with_baseline("Battery: Failed", Point::new(0, 32), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    flush(&mut display);

    // I2c Scan
    // let mut bus = i2c.acquire_i2c();
    // let test_ack = [0x00];
    // for i in 0..127 {
    //     if let Ok(_) = bus.write(i, &test_ack) {
    //         info!("i2c_scan: Found device at {:#04x}", i);
    //     } else {
    //         // info!("i2c_scan: No device at {}", i)
    //     }
    // }
    // devices at:
    // 0x27 pfc8575
    // 0x33 MAX11614
    // 0x68 MPU6050
    // 0x77 bme280
    // let mut dev = match Lis2dh12::new(i2c.acquire_i2c(), SlaveAddr::Default){
    //     Ok(dev) => dev,
    //     Err(e) => {
    //         panic!("Failed to create lisdh device on Alternate false with error {:?}", e);
    //     }};
    info!("Set up imu success");
    let mut mpu = Mpu6050::new(i2c.acquire_i2c());
    match mpu.init(&mut delay) {
        Ok(_) => {
            Text::with_baseline("IMU: Success", Point::new(0, 48), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            flush(&mut display);
        }
        Err(_) => {
            Text::with_baseline("IMU: Failed", Point::new(0, 48), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            flush(&mut display);
        }
    }

    // delay.delay_ms(250);
    // let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
    //     core1_task(clocks.system_clock.freq().to_Hz(), display)
    // });
    // display.clear(BinaryColor::Off).unwrap();
    // Text::with_baseline("Started second core", Point::new(0, 16), text_style, Baseline::Top)
    //     .draw(&mut display)
    //     .unwrap();
    // flush(&mut display);

    info!("Starting core0 loop");
    let mut mpu_cnt = 0;
    let mut acc_roll = Vec::<f32, 50>::new();
    let mut acc_pitch = Vec::<f32, 50>::new();

    init_screen_layout(&mut display, text_style);
    loop {
        // resources:
        // adc
        // gpio pins
        // i2c_sens

        // Check Buttons
        if kitchen_button.active() {
            info!("Got toggle message for Kitchen");
            let state = kitchen.actuate();
            // Q_KITCHEN.enqueue(state).unwrap_or_else(|_| warn!("Failed to queue kitchen state"));
            write_output(&mut display, "Kitchen", state, 0, text_style);
        }
        if stove_button.active() {
            let state = stove.actuate();
            // Q_STOVE.enqueue(state).unwrap_or_else(|_| warn!("Failed to queue stove state"));
            write_output(&mut display, "Stove", state, 1, text_style);
        }
        if water_button.active() {
            let state = water.actuate();
            // Q_WATER.enqueue(state).unwrap_or_else(|_| warn!("Failed to queue Water state"));
            write_output(&mut display, "Water", state, 2, text_style);
        }
        // Check IMU
        if (mpu_cnt % 25) == 0 {
            let raw_angles = mpu.get_acc_angles().unwrap();
            acc_roll
                .push((raw_angles[0] * 2.0 * 180.0 / PI) + ACC_ZERO.roll)
                .unwrap_or_else(|_| error!("Failed to add roll to moving average"));
            acc_pitch
                .push((raw_angles[1] * 2.0 * 180.0 / PI) + ACC_ZERO.pitch)
                .unwrap_or_else(|_| error!("Failed to add pitch to moving average"));
        }
        if mpu_cnt > 500 {
            let acc = Acceleration { roll: average(&acc_roll), pitch: average(&acc_pitch) };
            info!("Sending imu and clearing buffs with current average {} {}", acc.roll, acc.pitch);
            // Q_IMU
            //     .enqueue(angle)
            //     .unwrap_or_else(|_| warn!("Failed to enqueue new acc value"));
            write_imu(&mut display, acc, text_style);
            acc_roll.clear();
            acc_pitch.clear();
            mpu_cnt = 0;
        } else {
            mpu_cnt += 1;
        }
        // Check Water
        // if water_cnt > 500{
        //     water_en.set
        // }
        // Actuate RGBLeds

        delay.delay_ms(1);
    }
}
fn write_imu(display: &mut Display, acc: Acceleration, text_style: TextStyle) {
    let mut str_buff = FmtBuf::new();
    write!(str_buff, "IMU   {0:.2}   {1:.2}", acc.roll, acc.pitch)
        .unwrap_or_else(|_| warn!("Failed to fill in string buffer"));
    // imu_str.write_str("IMU r {}  p {}", acc.roll);
    display
        .fill_solid(&Rectangle::new(Point::new(0, 55), Size::new(128, 10)), BinaryColor::Off)
        .unwrap();
    Text::with_baseline(str_buff.as_str(), Point::new(0, 55), text_style, Baseline::Top)
        .draw(display)
        .unwrap();
    flush(display);
}
fn write_output(
    display: &mut Display,
    output: &str,
    state: bool,
    row: i32,
    // str_buff: &mut FmtBuf,
    text_style: TextStyle,
) {
    let mut str_buff = FmtBuf::new();
    // str_buff.reset();
    let y_start: i32 = (row * 12) + 8;
    write!(str_buff, "{: <7}:{:>6}", output, state).unwrap();
    // imu_str.write_str("IMU r {}  p {}", acc.roll);
    let tl = Point::new(0, y_start);
    let clear_size =
        text_style.measure_string(str_buff.as_str(), tl, Baseline::Top).bounding_box.size;
    display.fill_solid(&Rectangle::new(tl, clear_size), BinaryColor::Off).unwrap();
    Text::with_baseline(str_buff.as_str(), tl, text_style, Baseline::Top).draw(display).unwrap();
    flush(display);
}
fn init_screen_layout(display: &mut Display, text_style: TextStyle) {
    display
        .clear(BinaryColor::Off)
        .unwrap_or_else(|e| error!("Failed to clear screen with error {:?}", parse_de(e)));
    let title_text_style =
        MonoTextStyleBuilder::new().font(&FONT_4X6).text_color(BinaryColor::On).build();
    Text::with_baseline("TrailerBrain v5.1", Point::zero(), title_text_style, Baseline::Top)
        .draw(display)
        .unwrap();
    let state = false;
    write_output(display, "Kitchen", state, 0, text_style);
    write_output(display, "Stove", state, 1, text_style);
    write_output(display, "Water", state, 2, text_style);
    write_output(display, "Water level", state, 3, text_style);
}
fn average(vals: &Vec<f32, 50>) -> f32 {
    let mut sum: f32 = 0.0;
    for i in vals {
        sum += i;
    }
    return sum / vals.len() as f32;
}
struct TrailerButton<P: InputPin> {
    button: P,
    debounce_cd: u32,
    max_cd: u32,
}
impl<P: InputPin> TrailerButton<P> {
    fn new(button: P) -> Self {
        Self { button, debounce_cd: 0, max_cd: 250 }
    }
    fn active(&mut self) -> bool {
        if self.debounce_cd > 0 {
            self.debounce_cd -= 1;
            // info!("Reducing cd to {}", self.debounce_cd);
            return false;
        }
        if self.button.is_low().unwrap_or_else(|_| {
            error!("Failed to read input pin");
            false
        }) {
            self.debounce_cd = self.max_cd;
            return true;
        }
        return false;
    }
}
struct TrailerOutput<T: OutputPin> {
    state: bool,
    pcf_pin: T,
}
impl<T> TrailerOutput<T>
where
    T: OutputPin,
{
    fn new(pin: T) -> Self {
        Self { state: false, pcf_pin: pin }
    }
    fn actuate(&mut self) -> bool {
        if self.state {
            if let Err(_) = self.pcf_pin.set_low() {
                error!("Unable to set pin low");
            }
            self.state = false;
        } else {
            if let Err(_) = self.pcf_pin.set_high() {
                error!("Unable to set pin high");
            }
            self.state = true;
        }
        return self.state;
    }
}
