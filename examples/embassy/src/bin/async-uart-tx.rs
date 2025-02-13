//! Asynchronous UART transmission example application.
//!
//! This application receives sends 4 strings with different sizes permanently using UART A.
//! Ports PA8 and PA9 are used for this.
//!
//! Instructions:
//!
//! 1. Tie a USB to UART converter with RX to PA9 and TX to PA8 for UART A.
//! 2. Connect to the serial interface by using an application like Putty or picocom. You can
//!    can verify the correctness of the sent strings.
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use embedded_io_async::Write;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_embassy::embassy;
use va108xx_hal::{
    gpio::PinsA,
    pac::{self, interrupt},
    prelude::*,
    uart::{self, on_interrupt_uart_a_tx, TxAsync},
    InterruptConfig,
};

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

const STR_LIST: &[&str] = &[
    "Hello World\r\n",
    "Smoll\r\n",
    "A string which is larger than the FIFO size\r\n",
    "A really large string which is significantly larger than the FIFO size\r\n",
];

// main is itself an async function.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_init_print!();
    rprintln!("-- VA108xx Async UART TX Demo --");

    let mut dp = pac::Peripherals::take().unwrap();

    // Safety: Only called once here.
    unsafe {
        embassy::init(
            &mut dp.sysconfig,
            &dp.irqsel,
            SYSCLK_FREQ,
            dp.tim23,
            dp.tim22,
        );
    }

    let porta = PinsA::new(&mut dp.sysconfig, dp.porta);
    let mut led0 = porta.pa10.into_readable_push_pull_output();
    let mut led1 = porta.pa7.into_readable_push_pull_output();
    let mut led2 = porta.pa6.into_readable_push_pull_output();

    let tx = porta.pa9.into_funsel_2();
    let rx = porta.pa8.into_funsel_2();

    let uarta = uart::Uart::new_with_interrupt(
        &mut dp.sysconfig,
        50.MHz(),
        dp.uarta,
        (tx, rx),
        115200.Hz(),
        InterruptConfig::new(pac::Interrupt::OC2, true, true),
    );
    let (tx, _rx) = uarta.split();
    let mut async_tx = TxAsync::new(tx);
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut idx = 0;
    loop {
        rprintln!("Current time: {}", Instant::now().as_secs());
        led0.toggle().ok();
        led1.toggle().ok();
        led2.toggle().ok();
        let _written = async_tx
            .write(STR_LIST[idx].as_bytes())
            .await
            .expect("writing failed");
        idx += 1;
        if idx == STR_LIST.len() {
            idx = 0;
        }
        ticker.next().await;
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC2() {
    on_interrupt_uart_a_tx();
}
