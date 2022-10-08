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

type RS485 = stm32f1xx_hal::serial::Serial<stm32f1xx_hal::pac::USART3, (stm32f1xx_hal::gpio::gpiob::PB10<stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::PushPull>>,
    stm32f1xx_hal::gpio::gpiob::PB11<stm32f1xx_hal::gpio::Input<stm32f1xx_hal::gpio::Floating>>)>;

pub struct RS485Iface {
    iface: RS485
}

impl RS485Iface {
    fn send_str(&mut self, str_to_send: &str) {
        str_to_send.as_bytes()
        .iter()
        .try_for_each(|c| block!(self.iface.write(*c)))
        .map_err(|_| core::fmt::Error);
    }

    fn read_next(&mut self) -> u8 {
        block!(self.iface.read()).unwrap()
    }
}

// fn rs485_sendstr<F: FnMut(u8) -> Result<(), nb::Error<core::convert::Infallible>>>(str_to_send: &str, send_func: &mut F) {
//     str_to_send.as_bytes()
//     .iter()
//     .try_for_each(|c| nb::block!(send_func(*c)))
//     .map_err(|_| core::fmt::Error);
// }

// fn RS485_sendstr(ser: &mut RS485, str_to_send: &str) {
//     str_to_send.as_bytes()
//     .iter()
//     .try_for_each(|c| nb::block!(ser.write(*c)))
//     .map_err(|_| core::fmt::Error);
// }

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

    // Get UART2 instance
    // let mut serial = Serial::usart3(
    //     dp.USART3,
    //     (tx, rx),
    //     &mut afio.mapr,
    //     Config::default()
    //         .baudrate(115200.bps())
    //         .stopbits(StopBits::STOP1)
    //         .parity_none(),
    //     clocks,
    //     &mut rcc.apb1,
    // );

    let mut delay = Delay::new(cp.SYST, clocks);

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
    )};

    let mut rs485_dir = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

    rs485_dir.set_high().ok();
    // let value = 3;
    // writeln!(rs485_tx, "Button Press {:02}\r", value).unwrap();

    // "RS485 Started!\r\n".as_bytes()
    // .iter()
    // .try_for_each(|c| nb::block!(serial.write(*c)))
    // .map_err(|_| core::fmt::Error);

    // let mut send_func_ptr = |c| {serial.write(c)};
    // rs485_sendstr("RS485 Started!\r\n", &mut send_func_ptr);
    rs485i.send_str("RS485 Started!\r\n");

    // block!(serial.write('a' as u8)).ok();
    delay.delay_ms(3_u16);
    rs485_dir.set_low().ok();

    let mut receive_buff = [' '; 32];
    // let recv_next = || block!(serial.read()).unwrap() as char;

    loop {
        // Get byte from UART and send it back
        let received = rs485i.read_next() as char;

        if received == 'M' {
            receive_buff[0] = 'M';
            receive_buff[1] = rs485i.read_next() as char;
            if receive_buff[1] == '0' {
                rs485_dir.set_high().ok();
                // RS485_sendstr(&mut serial, "RS485 M0 received!\r\n");
                rs485i.send_str("RS485 M0 received!\r\n");
                delay.delay_ms(3_u16);
                rs485_dir.set_low().ok();
            }
        }

        // rs485_dir.set_high().ok();
        // rs485_sendstr(received as &str, &mut send_func_ptr);
        // RS485_sendstr(&mut serial, received);
        // delay.delay_ms(3_u16);
        // rs485_dir.set_low().ok();
    }
}

//openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg
//<gdb> -x openocd.gdb target/thumbv7em-none-eabihf/debug
//set auto-load safe-path /
