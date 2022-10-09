// src/main.rs

// This project implements a simple serial echo on UART2 using the `stm32f1xx-hal` crate

// std and main are not available for bare metal software
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use hal::serial::{Config, Serial, StopBits};
use embedded_hal::digital::v2::OutputPin;
use hal::{delay::Delay, pac, prelude::*};
use nb::block;
use stm32f1xx_hal as hal;
use heapless::String;
use core::fmt::Write;

type RS485 = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART3, (stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
    stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>;
type DirPin = stm32f1xx_hal::gpio::gpiob::PB12<stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::PushPull>>;

pub struct RS485Iface {
    iface: RS485,
    dirpin: DirPin,
    send_delay: stm32f1xx_hal::delay::Delay
}

impl RS485Iface {
    fn send_str(&mut self, str_to_send: &str) {
        self.dirpin.set_high().ok();

        let _ = str_to_send.as_bytes()
        .iter()
        .try_for_each(|c| block!(self.iface.write(*c)))
        .map_err(|_| core::fmt::Error);
        self.send_delay.delay_ms(3_u16);
        self.dirpin.set_low().ok();
    }

    fn read_next(&mut self) -> u8 {
        match block!(self.iface.read()) {
            Ok(x) => x,
            Err(_) => 0
        }
    }
}

#[entry]
fn main() -> ! {
    // Get access to device peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Get access to RCC, AFIO, FLASH and GPIOA
    let mut rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut flash = dp.FLASH.constrain();
    // let gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    // Freeze clocks
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Set up UART2 pins
    let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let rx = gpiob.pb11;

    let mut rs485i = RS485Iface {iface: Serial::usart3(
            dp.USART3,
            (tx, rx),
            &mut afio.mapr,
            Config::default()
                .baudrate(115200.bps())
                .stopbits(StopBits::STOP1)
                .parity_none(),
            clocks,
            &mut rcc.apb1,
        ),
        dirpin: gpiob.pb12.into_push_pull_output(&mut gpiob.crh),
        send_delay: Delay::new(cp.SYST, clocks)
    };

    rs485i.send_str("RS485 Started!\r\n");

    let mut recv_s: String<32> = String::from("");
    let mut send_s: String<32> = String::from("");

    let min_cmd_len = 2;

    loop {
        // receive_buff[0] = rs485i.read_next() as char;
        // let rec = rs485i.read_next();
        // if rec != 0 {
        //     recv_s.push(rec as char).unwrap();
        // }

        loop {
            let rec = rs485i.read_next();
            if rec != 0 {
                recv_s.push(rec as char).unwrap();
                // rs485i.send_str(&recv_s.as_str());
            } else {
                break;
            }

            if recv_s.len() >= min_cmd_len {
                // rs485i.send_str(&recv_s.as_str());
                break;
            }
        }

        match recv_s.find("S") {
            None => {},
            Some(i) => {
                if recv_s.len() > 1 {
                    let _ = write!(send_s,"RS485 S{} received!\r\n", &recv_s[i+1..]);
                    rs485i.send_str(send_s.as_str());
                    recv_s.clear();
                    send_s.clear()
                }
            }
        }

        if recv_s.len() >= 32 {
            recv_s.clear();
        }
    }
}

//openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
//<gdb> -x openocd.gdb target/thumbv7em-none-eabihf/debug
//set auto-load safe-path /
