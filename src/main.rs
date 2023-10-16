#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;
slint::include_modules!();

// fn core1(){
//     // Work with hardware
//     setup_imu(q_s.imu.sender());
//     setup_water(q_s.water.sender());
//     setup_battery();
//     setup_dcout();
//     setup_rgb();
//     setup_env();
//     loop{
//         // Will poll the hardware that doesn't have interupts and tell trigger a UI update
//         if (let Some(data_updates) = checkHw()){
//             // data_updates is a linked list or vec of hw updates
//             ui.update(data_updates);
//         }
//         for msg in q_hwcmd.recv(){
//             processHWCmd(msg);
//         }
//     }
// }

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    // let ui_handle = ui.as_weak();
    // ui.on_request_increase_value(move || {
    //     let ui = ui_handle.unwrap();
    //     ui.set_battery_percentage(ui.get_battery_percentage() + 0.5);
    // });
    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}

#[cfg(not(feature = "simulator"))]
fn core1_task() {
    use defmt::*;
    use defmt_rtt as _;
    loop {
        info!("Running Core 1");
        loop {}
    }
}

#[cfg(not(feature = "simulator"))]
use rp_pico::hal::multicore::Stack;
#[cfg(not(feature = "simulator"))]
static mut CORE1_STACK: Stack<4096> = Stack::new();

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    use embedded_graphics_core::{
        pixelcolor::Rgb565,
        prelude::{DrawTarget, OriginDimensions, RgbColor},
    };
    use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
    // Pull in any important traits
    use defmt::*;
    use defmt_rtt as _;
    use fugit::RateExtU32;
    use mipidsi::{Builder, Orientation};
    use panic_halt as _;
    use rp_pico::{
        hal::{self, gpio, multicore::Multicore, Clock, Sio},
        pac,
    };
    use slint::{platform::WindowEvent, Color};

    // -------- Setup Allocator --------
    info!("Setting up allocator");
    const HEAP_SIZE: usize = 100 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) };

    // -------- Setup peripherials --------
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Multicore set up
    let mut sio = Sio::new(pac.SIO);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);

    let clocks = hal::clocks::init_clocks_and_plls(
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();
    // let _spi_miso = pins.gpio12.into_mode::<hal::gpio::FunctionSpi>();

    // let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI1);

    // Set up our SPI pins into the correct mode
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio18.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio19.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio20.reconfigure();
    // let spi_cs = pins.gpio5.into_push_pull_output();

    // Create the SPI driver instance for the Display device
    let spi0 = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
    let spi0 = spi0.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        62_500_000.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut bl = pins.gpio6.into_push_pull_output();
    bl.set_high().unwrap();
    let rst = pins.gpio14.into_push_pull_output();
    let dc = pins.gpio16.into_push_pull_output();
    // let cs = pins.gpio9.into_push_pull_output();
    let di = display_interface_spi::SPIInterfaceNoCS::new(spi0, dc);
    let mut display = Builder::ili9341_rgb565(di)
        .with_color_order(mipidsi::ColorOrder::Bgr)
        .with_orientation(Orientation::Landscape(false))
        // .with_display_size(480, 320)
        .with_framebuffer_size(0, 0)
        .init(&mut delay, Some(rst))
        .unwrap();
    let size = display.size();

    // Set up our SPI pins into the correct mode
    let spi_sclk: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio10.reconfigure();
    let spi_mosi: gpio::Pin<_, gpio::FunctionSpi, gpio::PullNone> = pins.gpio11.reconfigure();
    let spi_miso: gpio::Pin<_, gpio::FunctionSpi, gpio::PullUp> = pins.gpio12.reconfigure();
    let spi_nrf_cs = pins.gpio13.into_push_pull_output();
    let spi_sd_cs = pins.gpio8.into_push_pull_output();
    let spi_tp_cs = pins.gpio9.into_push_pull_output();

    // Create the SPI driver instance for the SPI0 device
    let spi1 = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (spi_mosi, spi_miso, spi_sclk));
    let spi1 = spi1.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        3_000_000.Hz(),
        &embedded_hal::spi::MODE_3,
    );
    let spi = shared_bus::BusManagerSimple::new(spi1);

    // touch screen
    let touch_irq = pins.gpio17.into_pull_up_input();
    let mut touch = xpt2046::XPT2046::new(touch_irq, spi_tp_cs, spi.acquire_spi()).unwrap();

    // -------- Setup the Slint backend --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
        timer: hal::Timer,
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(
            &self,
        ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError>
        {
            Ok(self.window.clone())
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(
                self.timer.get_counter().duration_since_epoch().to_micros(),
            )
        }
    }

    // -------- Configure the UI --------
    // (need to be done after the call to slint::platform::set_platform)
    let _ui = create_slint_app();

    // -------- Event loop --------
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); 480];
    let mut last_touch = None;
    info!("Entering Loop");
    loop {
        info!("Updating loop");
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    // NOTE! this is not an efficient way to send pixel to the screen, but it is kept simple on this template.
                    // It would be much faster to use the DMA to send pixel in parallel.
                    // See the example in https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/pico_st7789.rs
                    self.0
                        // .fill_contiguous(
                        //     &rect,
                        //     self.1[range.clone()].iter().map(|p| {
                        //         embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                        //     }),
                        // )
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                                // embedded_graphics_core::pixelcolor::Rgb565::from(p.red())
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }
            renderer.render_by_line(DisplayWrapper(&mut display, &mut line));
            // renderer.render(buffer, pixel_stride)
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

        // TODO: we could save battery here by going to sleep up to
        //   slint::platform::duration_until_next_timer_update()
        // or until the next touch interrupt, whatever comes first
        // cortex_m::asm::wfe();
    }
}

#[cfg(not(feature = "simulator"))]
mod xpt2046 {
    use defmt::info;
    use embedded_hal::blocking::spi::Transfer;
    use embedded_hal::digital::v2::{InputPin, OutputPin};

    pub struct XPT2046<IRQ: InputPin + 'static, CS: OutputPin, SPI: Transfer<u8>> {
        irq: IRQ,
        cs: CS,
        spi: SPI,
        pressed: bool,
    }

    impl<PinE, IRQ: InputPin<Error = PinE>, CS: OutputPin<Error = PinE>, SPI: Transfer<u8>>
        XPT2046<IRQ, CS, SPI>
    {
        pub fn new(irq: IRQ, mut cs: CS, spi: SPI) -> Result<Self, PinE> {
            cs.set_high()?;
            Ok(Self { irq, cs, spi, pressed: false })
        }

        pub fn read(&mut self) -> Result<Option<(f32, f32)>, Error<PinE, SPI::Error>> {
            info!("Reading touch");
            const PRESS_THRESHOLD: i32 = -25_000;
            const RELEASE_THRESHOLD: i32 = -30_000;
            let threshold = if self.pressed { RELEASE_THRESHOLD } else { PRESS_THRESHOLD };
            self.pressed = false;

            if self.irq.is_low().map_err(|e| Error::Pin(e))? {
                const CMD_X_READ: u8 = 0b10010000;
                const CMD_Y_READ: u8 = 0b11010000;
                const CMD_Z1_READ: u8 = 0b10110000;
                const CMD_Z2_READ: u8 = 0b11000000;

                // These numbers were measured approximately.
                const MIN_X: u32 = 1900;
                const MAX_X: u32 = 30300;
                const MIN_Y: u32 = 2300;
                const MAX_Y: u32 = 30300;

                self.cs.set_low().map_err(|e| Error::Pin(e))?;

                macro_rules! xchg {
                    ($byte:expr) => {
                        match self
                            .spi
                            .transfer(&mut [$byte, 0, 0])
                            .map_err(|e| Error::Transfer(e))?
                        {
                            [_, h, l] => ((*h as u32) << 8) | (*l as u32),
                            _ => return Err(Error::InternalError),
                        }
                    };
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                if z < threshold {
                    xchg!(0);
                    self.cs.set_high().map_err(|e| Error::Pin(e))?;
                    return Ok(None);
                }

                xchg!(CMD_X_READ | 1); // Dummy read, first read is a outlier

                let mut point = (0u32, 0u32);
                for _ in 0..10 {
                    let y = xchg!(CMD_Y_READ);
                    let x = xchg!(CMD_X_READ);
                    point.0 += i16::MAX as u32 - x;
                    point.1 += y;
                }

                let z1 = xchg!(CMD_Z1_READ);
                let z2 = xchg!(CMD_Z2_READ);
                let z = z1 as i32 - z2 as i32;

                xchg!(0);
                self.cs.set_high().map_err(|e| Error::Pin(e))?;

                if z < RELEASE_THRESHOLD {
                    return Ok(None);
                }

                point.0 /= 10;
                point.1 /= 10;
                self.pressed = true;
                info!("Got Touch {:?}, {:?}", point.0, point.1);
                Ok(Some((
                    point.0.saturating_sub(MIN_X) as f32 / (MAX_X - MIN_X) as f32,
                    point.1.saturating_sub(MIN_Y) as f32 / (MAX_Y - MIN_Y) as f32,
                )))
            } else {
                Ok(None)
            }
        }
    }

    pub enum Error<PinE, TransferE> {
        Pin(PinE),
        Transfer(TransferE),
        InternalError,
    }
}
