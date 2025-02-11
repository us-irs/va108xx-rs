//! UART example application. Sends a test string over a UART and then enters
//! echo mode.
//!
//! Instructions:
//!
//! 1. Tie a USB to UART converter with RX to PA9 and TX to PA8.
//! 2. Connect to the serial interface by using an application like Putty or picocom.
//!    You should set a "Hello World" print when the application starts. After that, everything
//!    typed on the console should be printed back by the echo application.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal_nb::{nb, serial::Read};
use embedded_io::Write as _;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{gpio::PinsA, pac, prelude::*, uart};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx UART example application--");

    let mut dp = pac::Peripherals::take().unwrap();

    let gpioa = PinsA::new(&mut dp.sysconfig, dp.porta);
    let tx = gpioa.pa9.into_funsel_2();
    let rx = gpioa.pa8.into_funsel_2();

    let uarta = uart::Uart::new_without_interrupt(
        &mut dp.sysconfig,
        50.MHz(),
        dp.uarta,
        (tx, rx),
        115200.Hz(),
    );
    let (mut tx, mut rx) = uarta.split();
    writeln!(tx, "Hello World\r").unwrap();
    loop {
        // Echo what is received on the serial link.
        match rx.read() {
            Ok(recv) => {
                nb::block!(embedded_hal_nb::serial::Write::write(&mut tx, recv))
                    .expect("TX send error");
            }
            Err(nb::Error::WouldBlock) => (),
            Err(nb::Error::Other(uart_error)) => {
                rprintln!("UART receive error {:?}", uart_error);
            }
        }
    }
}
