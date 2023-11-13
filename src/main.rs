#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use cortex_m::singleton;

use embedded_hal::digital::v2::OutputPin;
// Debug and panic stuff
use panic_halt as _;
use defmt_rtt as _;
use defmt::info;
use fugit::RateExtU32;
use rp_pico::{hal::{Spi, gpio, Timer, gpio::PinState, Watchdog, Sio, multicore::{Multicore, Stack}, clocks, Clock, dma::DMAExt}, pac::{Peripherals, CorePeripherals}};
use slint::platform::{software_renderer::{MinimalSoftwareWindow, RepaintBufferType}, WindowEvent};

use crate::{pico_slint::PicoPlatform, spi::{ILI9488, SPIInterfaceNoCSDma, DMATransfer, Pix666}};

mod spi;
mod xpt2046;
mod pico_slint;

slint::include_modules!();


// Static Display Buffer
// 153600 bytes or 153.6 KB
// 110.4 KB left
const _DIS_HEIGHT:usize = 320;
const DIS_WIDTH:usize = 480;

// 10.4KB left after this
const HEAP_SIZE:usize = 100*1024;
static mut HEAP: [u8; HEAP_SIZE] = [0;HEAP_SIZE];

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[global_allocator]
static ALLOCATOR:embedded_alloc::Heap = embedded_alloc::Heap::empty();

fn core1_task() {
    use defmt::*;
    use defmt_rtt as _;
    loop {
        info!("Running Core 1");
        loop {}
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
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);

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
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
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

    info!("Setting up Display");
    let mut bl = pins.gpio6.into_push_pull_output_in_state(PinState::High);
    bl.set_high().unwrap();
    let rst = pins.gpio14.into_push_pull_output();
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
    let mut display = ILI9488::new(di, rst, delay);
    
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
    slint::platform::set_platform(Box::new(PicoPlatform {
        window: window.clone(),
        timer
    })).unwrap();
    info!("Setting Slint Window");
    let _ui = AppWindow::new().expect("Failed to load UI");

    let mut last_touch = None;
    info!("Starting Loop!");
    loop{
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|render| {
            // type TargetPixel = Rgb666;
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
                    slint::PhysicalPosition::new((point.0 * 320.) as _, (point.1 * 240.) as _)
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

