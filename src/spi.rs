// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use core::{u8, mem};

use cortex_m::delay::Delay;
use defmt::{info, debug, error};
use embedded_hal as hal;
use hal::{digital::v2::OutputPin, spi::FullDuplex};
use embedded_hal::blocking::spi;

use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use rp_pico::hal::dma::{single_buffer::{Config, Transfer}, SingleChannel, WriteTarget, Pace};
use slint::{platform::software_renderer::LineBufferProvider, Rgb8Pixel};

use embedded_hal::blocking::delay::DelayUs;
use crate::{spi::dcs::{EnterNormalMode, ExitSleepMode, PixelFormat, SetAddressMode, SetDisplayOn, SetPixelFormat, Dcs}, DIS_WIDTH};

mod dcs;
mod options;

pub type Pix666 = Rgb8Pixel;
pub enum DMATransfer<TO: WriteTarget, CH: SingleChannel> {
    Idle(CH, &'static mut [Pix666; DIS_WIDTH], TO),
    Running(Transfer<CH, PartialReadBuffer, TO>),
}

impl<TO: WriteTarget<TransmittedWord = u8> + FullDuplex<u8>, CH: SingleChannel>
    DMATransfer<TO, CH>
{
    fn wait(self) -> (CH, &'static mut [Pix666; DIS_WIDTH], TO) {
        match self {
            DMATransfer::Idle(a, b, c) => (a, b, c),
            DMATransfer::Running(dma) => {
                let (a, b, to) = dma.wait();
                // After the DMA operated, we need to empty the receive FIFO, otherwise the touch screen
                // driver will pick wrong values. Continue to read as long as we don't get a Err(WouldBlock)
                // while !to.read().is_err() {}
                (a, b.0, to)
            }
        }
    }
}
pub struct PartialReadBuffer(&'static mut [Pix666; DIS_WIDTH], core::ops::Range<usize>);
unsafe impl embedded_dma::ReadBuffer for PartialReadBuffer {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const <Self as embedded_dma::ReadBuffer>::Word, usize) {
        let act_slice = &self.0[self.1.clone()];
        (act_slice.as_ptr() as *const u8, act_slice.len() * core::mem::size_of::<Pix666>())
    }
}

// Should implement DrawTarget using dma command
pub struct ILI9488<DI:DmaWriteOnlyDataCommand, RST: OutputPin>{
    di: Option<DI>,
    rst: Option<RST>,
    // cur_buf: usize,
    buf: [Pix666; DIS_WIDTH],
    delay: Delay,
    // bufs: Vec<Ili9488Buf>,
    // dcs: Dcs<DI>
}

impl<DI, RST> ILI9488<DI, RST>
where 
    DI: WriteOnlyDataCommand + DmaWriteOnlyDataCommand,
    RST: OutputPin
    // Dcs<DI>: DcsCommand
{
    pub fn new(di:DI, rst:RST, delay: Delay) -> Self {
        let mut ili = Self {di:Some(di), delay, rst:Some(rst), buf: [Pix666::default();DIS_WIDTH] };
        ili.init_screen();
        ili
    }
    fn init_screen(&mut self) {
        if let Some(di) = self.di.take(){
            let mut dcs = Dcs::write_only(di);
            self.hard_reset();
            info!("Initting the common display");
            init_common(&mut dcs, &mut self.delay).unwrap();
            
            info!("Success! Releasing dcs");
            self.di = Some(dcs.release())
        }
    }
    pub fn write_cur_buf(&mut self, range:core::ops::Range<usize>, line:usize){
        let (sx, ex) = (range.start, range.end);
        let sy = line;
        self.set_address_window(sx, sy, ex, sy);
        if let Some(mut di) = self.di.take(){
            di.write_partial_data(&mut self.buf, range).unwrap()
        }

    }
    // Sets the address window for the display.
    fn set_address_window(&mut self, sx: usize, sy: usize, ex: usize, ey: usize)  {
        if let Some(di) = self.di.take(){
            let mut dcs = Dcs::write_only(di);
            dcs.write_command(dcs::SetColumnAddress::new(sx as u16, ex as u16)).unwrap();
            dcs.write_command(dcs::SetPageAddress::new(sy as u16, ey as u16)).unwrap();
        }
    }
    fn hard_reset(&mut self){
        if let Some(mut rst) = mem::take(&mut self.rst){
            let _ = rst.set_low();
            self.delay.delay_us(10);
            let _ = rst.set_high();
            self.rst = Some(rst);
        }
    }
}
fn init_common<DELAY, DI>(
    dcs: &mut Dcs<DI>,
    delay: &mut DELAY,
) -> Result<SetAddressMode, DisplayError>
where
    DELAY: DelayUs<u32>,
    DI: WriteOnlyDataCommand,
{
    let madctl = SetAddressMode::default()
        .with_color_order(options::ColorOrder::Bgr)
        .with_orientation(options::Orientation::Landscape(false));
    let pixel_format = PixelFormat::with_all(
        dcs::BitsPerPixel::TwentyFour
    );
    info!("Exitting sleep mode");
    dcs.write_command(ExitSleepMode)?; // turn off sleep
    info!("Setting pixel format");
    dcs.write_command(SetPixelFormat::new(pixel_format))?; // pixel format
    dcs.write_command(madctl)?; // left -> right, bottom -> top RGB
    dcs.write_raw(0xB6, &[0b0000_0010, 0x02, 0x3B])?; // DFC
    dcs.write_command(EnterNormalMode)?; // turn to normal mode
    dcs.write_command(SetDisplayOn)?; // turn on display

    // DISPON requires some time otherwise we risk SPI data issues
    delay.delay_us(120_000);

    Ok(madctl)
}

impl<DI, RST> LineBufferProvider for &mut ILI9488<DI, RST>
where
    DI: WriteOnlyDataCommand + DmaWriteOnlyDataCommand,
    RST: OutputPin
{
    fn process_line(
            &mut self,
            line: usize,
            range: core::ops::Range<usize>,
            render_fn: impl FnOnce(&mut [Self::TargetPixel]),
        ) {
        render_fn(&mut self.buf[range.clone()]);
        self.write_cur_buf(range, line);
    }

    type TargetPixel = Pix666;
}

pub trait DmaWriteOnlyDataCommand
{
    fn write_partial_data(&mut self, buf: &mut [Pix666; DIS_WIDTH], range: core::ops::Range<usize>) -> Result<(), DisplayError>;
}

pub struct SPIInterfaceNoCSDma<DC, TO, CH>
where
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord = u8> + FullDuplex<u8>,
    DC: OutputPin,
{
    dma: Option<DMATransfer<TO, CH>>,
    dc: DC,
}
impl<DC, TO, CH> SPIInterfaceNoCSDma<DC, TO, CH>
where
    CH: SingleChannel,
    TO: WriteTarget<TransmittedWord=u8> + FullDuplex<u8> + spi::Transfer<u8>,
    DC: OutputPin,
{
    /// Create new SPI interface for communication with a display driver
    pub fn new(dc:DC, dma: DMATransfer<TO, CH>) -> Self {
        Self { dc, dma: Some(dma)}
    }
}
impl <DC, TO, CH> DmaWriteOnlyDataCommand for SPIInterfaceNoCSDma<DC, TO, CH>
where 
    TO: WriteTarget<TransmittedWord=u8> + FullDuplex<u8> + spi::Transfer<u8>,
    CH: SingleChannel,
    DC: OutputPin
{
    fn write_partial_data(&mut self, dis_buf: &mut [Pix666;DIS_WIDTH], range: core::ops::Range<usize>) -> Result<(),DisplayError> {
        let (ch, buf, spi) = if let Some(dma) = mem::take(&mut self.dma){
            dma.wait()
        } else {
            panic!("DMA Doesn't hold expected data");
        };
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        core::mem::swap(dis_buf, buf);
        let mut pio = Config::new(ch, PartialReadBuffer(buf, range), spi);
        pio.pace(Pace::PreferSink);
        self.dma = Some(DMATransfer::Running(pio.start()));
        Ok(())
    }
}

impl <DC, TO, CH> WriteOnlyDataCommand for SPIInterfaceNoCSDma<DC, TO, CH>
where
TO: WriteTarget<TransmittedWord=u8> + FullDuplex<u8> + spi::Transfer<u8> + spi::Write<u8>,
CH: SingleChannel,
DC: OutputPin
{
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result<(), DisplayError> {
        // 1 = data, 0 = command
        info!("Setting dcs low");
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;

        info!("Getting spi bus from dma");
        let (ch, b, mut spi) = if let Some(dma) = mem::take(&mut self.dma){
            dma.wait()
        } else {
            panic!("DMA Doesn't hold expected data");
        };
        info!("Sending command u8's over bus");
        let e = send_u8(&mut spi, cmds);
        match e.clone(){
            Ok(_) => info!("Succeeded in writing commands"),
            Err(e) => match e {
                DisplayError::InvalidFormatError => error!("Invalid Format"),
                DisplayError::BusWriteError => error!("Bus write"),
                DisplayError::DCError => error!("DC Error"),
                DisplayError::CSError => error!("CS Error"),
                DisplayError::DataFormatNotImplemented => error!("Data format not implemented"),
                DisplayError::RSError => error!("RS Error"),
                DisplayError::OutOfBoundsError => error!("Out of Bounds"),
                _ => todo!(),
            }
        }
        info!("Restoring spi with dma, current value is {:?}", if self.dma.is_none(){"None"} else{"Not None"});
        self.dma = Some(DMATransfer::Idle(ch, b, spi));
        e
    }
    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        // 1 = data, 0 = command
        info!("Setting dcs high");
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        info!("Getting spi bus from dma");
        let (ch, b, mut spi) = if let Some(dma) = mem::take(&mut self.dma){
            dma.wait()
        } else {
            panic!("DMA Doesn't hold expected data");
        };
        info!("Sending data u8's over bus");
        let e = send_u8(&mut spi, buf);
        let _ = self.dma.insert(DMATransfer::Idle(ch, b, spi));
        e
    }
}

fn send_u8<SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>>(
    spi: &mut SPI,
    words: DataFormat<'_>,
) -> Result<(), DisplayError> {
    match words {
        DataFormat::U8Iter(iter) => {
            let mut buf = [0_u8; 32];
            let mut i = 0;

            info!("Using U8Iter to send");
            for v in iter.into_iter() {
                debug!("in spi send loop");
                buf[i] = v;
                i += 1;

                if i == buf.len() {
                    info!("First stage of spi transfer");
                    spi.transfer(&mut buf).map_err(|_| DisplayError::BusWriteError)?;
                    i = 0;
                }
            }

            if i > 0 {
                info!("Second stage of spi transfer");
                spi.transfer(&mut buf[..i]).map_err(|_| DisplayError::BusWriteError)?;
            }

            Ok(())
        }
        DataFormat::U16BEIter(iter) => {
            for mut v in iter.map(u16::to_be_bytes) {
                spi.transfer(&mut v).map_err(|_| DisplayError::BusWriteError)?;
            }

            Ok(())
        }
        DataFormat::U8(items) => {
            spi.write(items).map_err(|_| DisplayError::BusWriteError)?;
            Ok(())
        }
        _ => Err(DisplayError::DataFormatNotImplemented),
    }
}