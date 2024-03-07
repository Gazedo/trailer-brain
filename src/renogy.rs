use core::{time::Duration, u8};

use defmt::info;
use rmodbus::{client::ModbusRequest, guess_response_frame_len, ModbusProto};

trait RenInterface {
    fn get_data(&mut self, register: u16, size: u16) -> u32;
}

impl <UART> RenInterface for SmartBattery<UART>
where
UART: embedded_hal::serial::Write<u8> + embedded_hal::serial::Read<u8>
{
    fn get_data(&mut self, register: u16, size: u16) -> u32 {
        let mut mreq = ModbusRequest::new(self.address, ModbusProto::Rtu);
        // let mut request = Vec::new();
        let mut request = [0_u8;32];
        let mut response = [0_u8;32];
        // Send a request for data
        mreq.generate_get_holdings(register, size, &request.iter_mut())
            .unwrap();
        match self.port.write(&request) {
            Ok(written) => info!("Wrote {} bytes to the battery at {}", written, self.address),
            Err(e) => panic!("Got serial error {}", e),
        };
        // self.port.drain().unwrap();

        // Get the header of the response
        // let mut buf = [0u8; 6];
        let mut timedout = true;
        // let mut response = Vec::new();
        while timedout {
            if timedout {
                info!("Timedout on serial, retrying");
            }
            let read_bytes = match self.port.read(){
                Ok(res) => ,
                Err(_) => todo!(),
            }
            // timedout = match self.port.read(&mut response) {
            //     Ok(bread) => {
            //         info!("Read {} bytes", bread);
            //         false
            //     }
            //     Err(e) => {
            //         info!("\nGot error {:?}\n", e);
            //         true
            //     }
            // };
            info!("Got {:?} from serial port", response);
            // response.extend_from_slice(&buf);
            // Get size of the rest of the message
            let len = guess_response_frame_len(&response, ModbusProto::Rtu).unwrap();
            if len > 6 {
                let mut rest = [0u8; (len - 6) as usize];
                timedout = match self.port.read(&mut rest) {
                    Ok(_) => false,
                    Err(e) => {
                        info!("\nGot error {:?}\n", e);
                        true
                    }
                };
                response.extend(rest);
            }
        }

        mreq.parse_ok(&response).unwrap();
        return response;
    }
}

pub struct SmartBattery<UART> {
    port: UART,
    address: u8,
    total_capacity: f32,
}

impl <UART> SmartBattery<UART> 
// where UART:blocking_wr
{
    pub fn new(uart:UART , address: u8) -> Self {
        info!(
            "Setting up new battery at {:?}",
            address
        );
        let mut battery = Self {
            port:uart,
            address,
            total_capacity: 100.0,
        };
        battery.port.set_write_mode(true).unwrap();
        battery
            .port
            .set_read_mode(0, Duration::from_secs(5))
            .unwrap();

        battery.port.set_hardware_flow_control(false);
        battery.port.set_software_flow_control(true);
        let hard = battery.port.hardware_flow_control();
        let soft = battery.port.software_flow_control();
        info!("Hardware flow control {} and software flow control {}", hard, soft);
        battery.total_capacity = battery.get_capacity();
        return battery;
    }

    pub fn get_soc(&mut self) -> f32 {
        let temp = self.get_data(0x13b4, 2);
        let arr: [u8; 4] = match temp[3..7].try_into() {
            Ok(ba) => ba,
            Err(_) => panic!("Expected a Vec of length {} but it was {}", 4, temp.len()),
        };
        let soc = u32::from_be_bytes(arr) as f32 / 1000.0;
        info!("Got new soc as {:?}", soc);
        soc
    }
    pub fn get_capacity(&mut self) -> f32 {
        let temp = self.get_data(0x13b6, 2);
        let arr: [u8; 4] = match temp[3..7].try_into() {
            Ok(ba) => ba,
            Err(_) => panic!("Expected a Vec of length {} but it was {}", 4, temp.len()),
        };
        let capacity = u32::from_be_bytes(arr) as f32 / 1000.0;
        info!("Got new capacity as {:?}", capacity);
        capacity
    }
    pub fn get_current(&mut self) -> f32 {
        let temp = self.get_data(0x13b2, 1);
        let arr: [u8; 2] = match temp[3..5].try_into() {
            Ok(ba) => ba,
            Err(_) => panic!("Expected a Vec of length {} but it was {}", 2, temp.len()),
        };
        let current = i16::from_be_bytes(arr) as f32 / 100.0;
        info!("Got new current {:?}", current);
        current
    }
    pub fn get_voltage(&mut self) -> f32 {
        let temp = self.get_data(0x13b3, 1);
        let arr: [u8; 2] = match temp[3..5].try_into() {
            Ok(ba) => ba,
            Err(_) => panic!("Expected a Vec of length {} but it was {}", 2, temp.len()),
        };
        let voltage = u16::from_be_bytes(arr) as f32 / 10.0;
        info!("Got new voltage {:?}", voltage);
        voltage
    }
}
