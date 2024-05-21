use core::{u8, fmt::Debug};

use cortex_m::delay::Delay;
use defmt::{error, info};
use embedded_hal::digital::v2::OutputPin;
use rmodbus::{client::ModbusRequest, guess_response_frame_len, ModbusProto};
use heapless::Vec;
use panic_halt as _;
use rp_pico::{hal::uart::{Enabled, UartPeripheral, ValidUartPinout}, pac::UART0};

trait RenInterface {
    fn get_u32(&mut self, register: u16, size: usize) -> f32;
    fn set_write(&mut self);
    fn set_read(&mut self);
}

// fn  write_all<T>(port:&mut T, data: &[u8]) -> Result<u8, T::Error>
// where T: Write<u8>, <T as embedded_hal::serial::Write<u8>>::Error: Debug
// {
//     let mut written = 0;
//     for &i in data{
//         match port.write(i){
//             Ok(_) => info!("UART: Wrote {:#04x}",i),
//             Err(_) => error!("UART: Failed to write {:#04x}", i),
//         }
//         written += 1;
//     }
//     match port.flush() {
//         Ok(_) => info!("UART: Flushed"),
//         Err(_) => error!("UART: Failed to flush"),
//     }
//     Ok(written)
// }
// fn  read_n<PO: ValidUartPinout<UART0>>(port:&mut UartPeripheral<Enabled, UART0, PO>, n:usize, buf: &mut [u8])
// // where T: embedded_hal::serial::Read<u8>, <T as embedded_hal::serial::Read<u8>>::Error: Debug
// // where T: Read<u8> + UartDevice
// {
//     type Error = ReadErrorType;
//     for i in 0..n{
//         port.read_full_blocking(buffer)
//         let byte =  match port.read(){
//             Ok(b) => {
//                 info!("UART: Read {}", b);
//                 b
//             },
//             Err(e) => match e {
//                 Other(inner) => {
//                     match inner{}
//                 },
//                 WouldBlock => Err(WouldBlock),
//             },
//                 let err = match e{
//                     ReadErrorType::Overrun => 0,
//                     ReadErrorType::Break => 1,
//                     ReadErrorType::Parity => 2,
//                     ReadErrorType::Framing => 3
//                 };
//                 error!("UART: Failed to read e={}", err);
//             },
//         };
//         // {
//         //     Ok(b) => b,
//         //     Err(e) => {
//         //         error!("Failed to read from uart {:?}", e);
//         //         0x00
//         //     }
//         // };
//         info!("UART: Read {}", byte);
//         buf[i] = byte;
//     }
// }

// impl <UART, P> RenInterface for SmartBattery<UART, P>
impl <P, PO: ValidUartPinout<UART0>> RenInterface for SmartBattery<P, PO> 
where
P: OutputPin,
<P as OutputPin>::Error: Debug,
// UART: embedded_hal::serial::Write<u8> + embedded_hal::serial::Read<u8>,
// <UART as embedded_hal::serial::Write<u8>>::Error: Debug,
// <UART as embedded_hal::serial::Read<u8>>::Error: Debug,
{
    fn set_write(&mut self) {
        self.re.set_high().unwrap();
    }
    fn set_read(&mut self) {
        self.re.set_low().unwrap();
    }
    fn get_u32(&mut self, register: u16, size: usize) -> f32 {
        let mut mreq = ModbusRequest::new(self.address, ModbusProto::Rtu);
        // let mut request = Vec::new();
        // let mut request = [0_u8;32];
        let mut request = Vec::<u8,48>::new();
        let mut response = Vec::<u8,48>::new();
        // let mut response = [0_u8;32];
        // Send a request for data
        self.set_write();
        mreq.generate_get_inputs(register, size.try_into().unwrap(), &mut request).unwrap();
        // mreq.generate_get_holdings(register, size, &mut request)
        //     .unwrap();
        self.port.write_full_blocking(&request);
        self.port.set_fifos(true);
        // match write_all(&mut self.port,&request) {
        //     Ok(written) => info!("Wrote {} bytes to the battery at {}", written, self.address),
        //     Err(e) => panic!("Got serial error {:?}", e),
        // };
        // self.port.drain().unwrap();
        // info!("UART: Flushing port");
        // self.port.flush().expect("Failed to flush the serial port");

        // Get the header of the response
        info!("UART: Setting to read mode");
        self.delay.delay_ms(5);
        self.set_read();
        // self.delay.delay_ms(50);
        response.resize_default(6).unwrap();
        info!("UART: response size is {} Reading...", response.len());
        // info!("UART: read {}", self.port.read().unwrap());
        
        match self.port.read_full_blocking(&mut response){
            Ok(_) => info!("[REN] Successfully read initial response"),
            Err(e) => error!("[REN] Failed to read initial response with {}", e),
        }
        for i in 0..response.len(){
            info!("b{} = {:#04x}", i, response[i]);
        }
        // read_n(&mut self.port, 6, &mut response[0..5]);
        // for i in 0..5{
        //     let byte = match self.port.read(){
        //         Ok(b) => b,
        //         Err(e) => {
        //             error!("Failed to read from uart {:?}", e);
        //             0x00
        //         }
        //     };
        //     response[i] = byte;
        // }
        let len:usize = guess_response_frame_len(&response, ModbusProto::Rtu).expect("failed to get frame length").into();
        response.resize_default(len).unwrap();
        match self.port.read_full_blocking(&mut response[6..len]){
            Ok(_) => info!("[REN] Successfully read rest of response"),
            Err(e) => error!("[REN] Failed to read second response with {}", e),
        }
        // read_n(&mut self.port, len - 6, &mut response[6..len]);

        // for i in 6..len{
        //     let byte = match self.port.read(){
        //         Ok(b) => b,
        //         Err(e) => {
        //             error!("Failed to read from uart {:?}", e);
        //             0x00
        //         }
        //     };
        //     response[i] = byte;
        // }
        // info!("Got {:?} from serial port", response);

        mreq.parse_ok(&response).unwrap();


        let arr: [u8; 4] = match response[3..3+(size*2)].try_into() {
            Ok(ba) => ba,
            Err(_) => panic!("Expected a Vec of length {} but it was {}", 4, size * 2),
        };
        let data = f32::from_be_bytes(arr) as f32 / 1000.0;
        info!("Got new data as {:?}", data);
        return data;
    }
}

pub struct SmartBattery<P, PO: ValidUartPinout<UART0>> {
    port: UartPeripheral<Enabled, UART0, PO>,
    re: P,
    address: u8,
    total_capacity: f32,
    delay: Delay,
}

impl <P, PO: ValidUartPinout<UART0>> SmartBattery<P, PO> 
where
P: OutputPin,
<P as OutputPin>::Error: Debug,
// UART: embedded_hal::serial::Write<u8> + embedded_hal::serial::Read<u8>,
// <UART as embedded_hal::serial::Write<u8>>::Error: Debug,
// <UART as embedded_hal::serial::Read<u8>>::Error: Debug,
// where UART:blocking_wr
{
    pub fn new(uart:UartPeripheral<Enabled, UART0, PO>, re:P, address: u8, delay: Delay) -> Self {
        info!(
            "Setting up new battery at {:?}",
            address
        );
        let mut battery = Self {
            port:uart,
            re,
            address,
            total_capacity: 100.0,
            delay
        };
        // battery.port.set_write_mode(true).unwrap();
        // battery
        //     .port
        //     .set_read_mode(0, Duration::from_secs(5))
        //     .unwrap();

        // battery.port.set_hardware_flow_control(false);
        // battery.port.set_software_flow_control(true);
        // let hard = battery.port.hardware_flow_control();
        // let soft = battery.port.software_flow_control();
        // info!("Hardware flow control {} and software flow control {}", hard, soft);
        battery.total_capacity = battery.get_capacity();
        return battery;
    }

    pub fn get_soc(&mut self) -> f32 {
        // let temp = self.get_data(0x13b4, ResSize::);
        // let arr: [u8; 4] = match temp[3..7].try_into() {
        //     Ok(ba) => ba,
        //     Err(_) => panic!("Expected a Vec of length {} but it was {}", 4, temp.len()),
        // };
        // let soc = u32::from_be_bytes(arr) as f32 / 1000.0;
        let temp = self.get_u32(0x13b4, 2);
        let soc = temp as f32 / 1000.0;
        info!("Got new soc as {:?}", soc);
        soc
    }
    pub fn get_capacity(&mut self) -> f32 {
        // let temp = self.get_data(0x13b6, 2);
        // let arr: [u8; 4] = match temp[3..7].try_into() {
        //     Ok(ba) => ba,
        //     Err(_) => panic!("Expected a Vec of length {} but it was {}", 4, temp.len()),
        // };
        // let capacity = u32::from_be_bytes(arr) as f32 / 1000.0;
        // info!("Got new capacity as {:?}", capacity);
        // capacity
        let temp = self.get_u32(0x13b6, 2);
        let capacity = temp as f32 / 1000.0;
        info!("Got new capacity as {:?}", capacity);
        capacity
    }
    // pub fn get_current(&mut self) -> f32 {
    //     let temp = self.get_data(0x13b2, 1);
    //     let arr: [u8; 2] = match temp[3..5].try_into() {
    //         Ok(ba) => ba,
    //         Err(_) => panic!("Expected a Vec of length {} but it was {}", 2, temp.len()),
    //     };
    //     let current = i16::from_be_bytes(arr) as f32 / 100.0;
    //     info!("Got new current {:?}", current);
    //     current
    // }
    // pub fn get_voltage(&mut self) -> f32 {
    //     let temp = self.get_data(0x13b3, 1);
    //     let arr: [u8; 2] = match temp[3..5].try_into() {
    //         Ok(ba) => ba,
    //         Err(_) => panic!("Expected a Vec of length {} but it was {}", 2, temp.len()),
    //     };
    //     let voltage = u16::from_be_bytes(arr) as f32 / 10.0;
    //     info!("Got new voltage {:?}", voltage);
    //     voltage
    // }
}
