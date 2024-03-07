#![no_std]
#![no_main]

extern crate alloc;

use core::fmt::Debug;

use alloc::boxed::Box;
use cortex_m::{delay::Delay, singleton};

use embedded_hal::{digital::v2::OutputPin, blocking::i2c::{WriteRead, Write}};
// Debug and panic stuff
use defmt::{error, info};
use defmt_rtt as _;
use fugit::RateExtU32;
use heapless::mpmc::Q4;
use panic_halt as _;
use pcf857x::Pcf8575;
use rp_pico::{
    hal::{
        clocks,
        dma::DMAExt,
        gpio,
        gpio::PinState,
        multicore::{Multicore, Stack},
        Clock, Sio, Spi, Timer, Watchdog, I2C, uart::{UartPeripheral, UartConfig, DataBits, StopBits},
    },
    pac::{CorePeripherals, Peripherals},
};
use shared_bus::BusManagerCortexM;
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, RepaintBufferType},
        WindowEvent,
    },
    ComponentHandle,
};

use crate::{
    pico_slint::PicoPlatform,
    spi::{DMATransfer, Pix666, SPIInterfaceNoCSDma, ILI9488},
};

// mod imu;
mod pico_slint;
mod renogy;
mod spi;
mod xpt2046;

slint::include_modules!();

const _DIS_HEIGHT: usize = 320;
const DIS_WIDTH: usize = 480;

// 10.4KB left after this
const HEAP_SIZE: usize = 100 * 1024;
static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[global_allocator]
static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();

enum UICmd {
    ToggleLight(Output, bool),
    RGBLight(u8, (u8, u8, u8)),
    ZeroIMU,
}
#[derive(Debug)]
struct BatInfo {
    soc: f32,
    cur: f32,
    temp: f32,
}
#[derive(Debug)]
struct ImuInfo {
    roll: f32,
    pitch: f32,
}
enum UIMsg {
    Battery(BatInfo),
    IMU(ImuInfo),
}
static Q_CMD: Q4<UICmd> = Q4::new();
static Q_MSG: Q4<UIMsg> = Q4::new();

fn core1_task<I2C, UART, E>(i: usize, mut delay: Delay, i2c:&BusManagerCortexM<I2C>, uart:UART) 
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug
{
    use defmt::*;
    use defmt_rtt as _;
    info!("Running Core 1 with input val {}", i);
    // let mut soc = 0.0;
    // let mut cur = 0.0;
    // let mut temp = 0.0;
    // let mut roll = -10.0;
    // let mut pitch = -10.0;
    
    let address = pcf857x::SlaveAddr::Alternative(true, true, true);
    let mut expander = Pcf8575::new(i2c.acquire_i2c(), address);
    let bits = 0xff;
    if let Err(_) = expander.set(bits){
        error!("Failed to set expander bits")
    }
    let mut pins = expander.split();
    
    // let mut imu = Imu::new(i2c.acquire_i2c()).unwrap();
    let bat = renogy::SmartBattery::new(uart, 0x30);
    loop {
        // Check for commands from UI
        if let Some(new_msg) = Q_CMD.dequeue() {
            match new_msg {
                UICmd::ToggleLight(name, val) => match name {
                    Output::Kitchen => {
                        info!("Got toggle message for Kitchen with val {}", val);
                        if let Err(_) = pins.p14.set_state(PinState::from(val)){
                            error!("Unable to set kitchen");
                        }
                        // let bits = match val{
                        //     true => 0x00,
                        //     false => 0xff,
                        // };
                        // if let Err(e) = expander.set(bits){
                        //     error!("Failed to set expander bits")
                        // }
                    }
                    Output::Stove => {
                        info!("Got toggle message for Stove with val {}", val);
                        // let _ = pins.p4.set_state(PinState::from(val));
                        if let Err(_) = pins.p17.set_state(PinState::from(val)){
                            error!("Unable to set Stove");
                        }
                    
                    },
                    Output::Fridge => {
                        info!("Got toggle message for Fridge with val {}", val);
                        // let _ = pins.p3.set_state(PinState::from(val));
                        if let Err(_) = pins.p0.set_state(PinState::from(val)){
                            error!("Unable to set Fridge");
                        }
                    },
                    Output::Water => {
                        info!("Got toggle message for Water with val {}", val);
                        // let _ = pins.p0.set_state(PinState::from(val));
                        if let Err(_) = pins.p3.set_state(PinState::from(val)){
                            error!("Unable to set Water");
                        }
                    },
                },
                UICmd::RGBLight(name, val) => {
                    info!("Set the rgbled {} to {}{}{}", name, val.0, val.1, val.2)
                }
                UICmd::ZeroIMU => info!("Zero the imu"),
            }
        }
        // Check hardware and send messages to the UI

        // soc += 1.0;
        // cur += 0.1;
        // temp += 0.1;
        // if soc >= 100.0 {
        //     soc = 0.0;
        // }
        // if cur >= 30.0 {
        //     cur = 0.0;
        // }
        // if temp >= 35.0 {
        //     temp = 0.0;
        // }
        // let bat_val = BatInfo { soc, cur, temp};
        // if let Err(_) = Q_MSG.enqueue(UIMsg::Battery(bat_val)) {
        //     error!("Failed to enqueue new Battery info message");
        // }
        // if imu.available(){
        //     let angles = imu.get_angles();
        //     let imu_msg = ImuInfo {roll:angles.get_roll(), pitch:angles.get_pitch()};
        //     if let Err(_) = Q_MSG.enqueue(UIMsg::IMU(imu_msg)){
        //         error!("Failed to enqueue imu message");
        //     }
        // }
        // roll += 0.01;
        // pitch += 0.01;
        // if roll >= 10.0 && pitch >= 10.0 {
        //     roll = -10.0;
        //     pitch = -10.0;
        // }

        // let imu_val = ImuInfo { roll, pitch};
        // if let Err(_) = Q_MSG.enqueue(UIMsg::IMU(imu_val)) {
        //     error!("Failed to enqueue new Battery info message");
        // }
        delay.delay_us(500);
    }
}

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
    let mut sio = Sio::new(pac.SIO);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

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
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) }
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Setup Spi buses
    info!("Setting up SPI Pins");
    let spi0_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio18.reconfigure();
    let spi0_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio19.reconfigure();
    let spi0_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio20.reconfigure();
    let spi1_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio10.reconfigure();
    let spi1_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio11.reconfigure();
    let spi1_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio12.reconfigure();

    // Create the SPI driver instance for the Display device
    info!("Setting up SPI Busses");
    let spi0 = Spi::<_, _, _, 8>::new(pac.SPI0, (spi0_mosi, spi0_miso, spi0_sclk));
    let spi0 = spi0.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500_000.Hz(),
        &embedded_hal::spi::MODE_0,
    );
    let spi1 = Spi::<_, _, _, 8>::new(pac.SPI1, (spi1_mosi, spi1_miso, spi1_sclk));
    let spi1 = spi1.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        3_000_000.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let spi = shared_bus::BusManagerSimple::new(spi1);
    
    // STarting i2c
    info!("Setting up I2C");
    let sda_pin: gpio::Pin<_, gpio::FunctionI2C, gpio::PullUp> = pins.gpio4.reconfigure();
    let scl_pin: gpio::Pin<_, gpio::FunctionI2C, gpio::PullUp> = pins.gpio5.reconfigure();
    let i2c = I2C::i2c0(pac.I2C0, sda_pin, scl_pin, 400.kHz(), &mut pac.RESETS, &clocks.system_clock);
    let i2c: &'static _ = shared_bus::new_cortexm!(I2C<rp_pico::pac::I2C0, (gpio::Pin<gpio::bank0::Gpio4, gpio::FunctionI2c, gpio::PullUp>, gpio::Pin<gpio::bank0::Gpio5, gpio::FunctionI2c, gpio::PullUp>)> = i2c).unwrap();
    // let i2c = shared_bus::CortexMMutex::new(i2c); 

    // UART
    info!("Setting up uart for modbus rtu");
    let tx_pin: gpio::Pin<_, gpio::FunctionUart, gpio::PullDown> = pins.gpio0.reconfigure();
    let rx_pin: gpio::Pin<_, gpio::FunctionUart, gpio::PullDown> = pins.gpio1.reconfigure();
    // let uart = rp2040_hal::
    let uart = UartPeripheral::new(pac.UART0, (tx_pin, rx_pin), &mut pac.RESETS)
    .enable(
        UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
        clocks.peripheral_clock.freq(),
    ).unwrap();
    
    info!("Setting up Display");
    let mut bl = pins.gpio6.into_push_pull_output_in_state(PinState::High);
    // bl.set_high().unwrap();
    let mut rst = pins.gpio14.into_push_pull_output_in_state(PinState::High);
    // rst.set_high().unwrap();
    let dc = pins.gpio16.into_push_pull_output();
    let dma = pac.DMA.split(&mut pac.RESETS);
    let write_buf = singleton!(: [Pix666; DIS_WIDTH] = [Pix666::default(); DIS_WIDTH]).unwrap();
    let dma = DMATransfer::Idle(dma.ch0, write_buf, spi0);
    let di = SPIInterfaceNoCSDma::new(dc, dma);

    info!("Starting Display");
    // let mut display = Builder::ili9341_rgb565(di)
    //     .with_color_order(ColorOrder::Bgr)
    //     .with_orientation(Orientation::Landscape(false))
    //     .with_display_size(DIS_WIDTH as u16, DIS_HEIGHT as u16)
    //     .with_framebuffer_size(DIS_WIDTH as u16, DIS_HEIGHT as u16)
    //     .init(&mut delay, Some(rst))
    //     .unwrap();
    delay.delay_us(100);
    let mut display = ILI9488::new(di, rst, &mut delay);
    // display.clear(Rgb666::BLACK).unwrap();

    info!("Set up DMA to display");

    info!("Setting up SPI1 Peripherials");
    let _cs_nrf = pins.gpio13.into_push_pull_output_in_state(PinState::High);
    let _cs_sd = pins.gpio8.into_push_pull_output_in_state(PinState::High);
    let cs_tp = pins.gpio9.into_push_pull_output_in_state(PinState::High);

    info!("Starting Touch driver");
    let touch_irq = pins.gpio17.into_pull_up_input();
    let mut touch = xpt2046::XPT2046::new(touch_irq, cs_tp, spi.acquire_spi()).unwrap();

    info!("Starting Slint UI");
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    info!("Setting Slint Platform");
    slint::platform::set_platform(Box::new(PicoPlatform { window: window.clone(), timer }))
        .unwrap();

    info!("Setting up queue's");

    info!("Setting Slint Window");
    let ui = AppWindow::new().expect("Failed to load UI");
    info!("Setting up pub/sub");
    ui.global::<Logic>().on_send_toggle(|name, val| {
        let msg = UICmd::ToggleLight(name, val);
        let n = match name {
            Output::Kitchen => "Kitchen",
            Output::Stove => "Stove",
            Output::Fridge => "Fridge",
            Output::Water => "Water",
        };
        info!("For [{:?}] new val is {:?}, enqueueing", n, val);

        if let Err(_) = Q_CMD.enqueue(msg) {
            error!("Failed to enqueue newest UI message");
        }
    });
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task(5, delay, i2c, uart));

    let mut last_touch = None;
    info!("Starting Loop!");
    loop {
        slint::platform::update_timers_and_animations();
        if let Some(new_msg) = Q_MSG.dequeue() {
            match new_msg {
                UIMsg::Battery(bat) => {
                    info!("Got new battery message soc [{}] amp [{}] temp [{}]", bat.soc, bat.cur, bat.temp);
                    ui.set_battery_percentage(bat.soc);
                    ui.set_battery_amp(bat.cur);
                    ui.set_battery_temperature(bat.temp);
                }
                UIMsg::IMU(imu) => {
                    info!("Got new imu message roll [{}] pitch [{}]", imu.roll, imu.pitch);
                    ui.set_imu_roll(imu.roll);
                    ui.set_imu_pitch(imu.pitch);
                }
            }
        }

        window.draw_if_needed(|render| {
            // Will also send pixels to the display and increment the buffer location
            render.render_by_line(&mut display);
        });
        // handle touch event
        let button = slint::platform::PointerEventButton::Left;
        if let Some(event) = touch
            .read()
            .map_err(|_| ())
            .unwrap()
            .map(|point| {
                let position =
                    slint::PhysicalPosition::new((point.0 * 480.) as _, (point.1 * 320.) as _)
                        .to_logical(window.scale_factor());
                match last_touch.replace(position) {
                    Some(_) => WindowEvent::PointerMoved { position },
                    None => WindowEvent::PointerPressed { position, button },
                }
            })
            .or_else(|| {
                last_touch.take().map(|position| WindowEvent::PointerReleased { position, button })
            })
        {
            window.dispatch_event(event);
            // Don't go to sleep after a touch event that forces a redraw
            continue;
        }

        if window.has_active_animations() {
            continue;
        }
    }
}
