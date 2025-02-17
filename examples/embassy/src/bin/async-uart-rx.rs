//! Asynchronous UART reception example application.
//!
//! This application receives data on two UARTs permanently using a ring buffer.
//! The ring buffer are read them asynchronously. UART A is received on ports PA8 and PA9.
//! UART B is received on ports PA2 and PA3.
//!
//! Instructions:
//!
//! 1. Tie a USB to UART converter with RX to PA9 and TX to PA8 for UART A.
//!    Tie a USB to UART converter with RX to PA3 and TX to PA2 for UART B.
//! 2. Connect to the serial interface by using an application like Putty or picocom. You can
//!    type something in the terminal and check if the data is echoed back. You can also check the
//!    RTT logs to see received data.
#![no_std]
#![no_main]
use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::Instant;
use embedded_io::Write;
use embedded_io_async::Read;
use heapless::spsc::{Consumer, Producer, Queue};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::PinsA,
    pac::{self, interrupt},
    prelude::*,
    uart::{
        self, on_interrupt_rx_overwriting,
        rx_asynch::{on_interrupt_rx, RxAsync},
        Bank, RxAsyncOverwriting, Tx,
    },
    InterruptConfig,
};

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

static QUEUE_UART_A: static_cell::ConstStaticCell<Queue<u8, 256>> =
    static_cell::ConstStaticCell::new(Queue::new());
static PRODUCER_UART_A: Mutex<RefCell<Option<Producer<u8, 256>>>> = Mutex::new(RefCell::new(None));

static QUEUE_UART_B: static_cell::ConstStaticCell<Queue<u8, 256>> =
    static_cell::ConstStaticCell::new(Queue::new());
static PRODUCER_UART_B: Mutex<RefCell<Option<Producer<u8, 256>>>> = Mutex::new(RefCell::new(None));
static CONSUMER_UART_B: Mutex<RefCell<Option<Consumer<u8, 256>>>> = Mutex::new(RefCell::new(None));

// main is itself an async function.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();
    rprintln!("-- VA108xx Async UART RX Demo --");

    let mut dp = pac::Peripherals::take().unwrap();

    // Safety: Only called once here.
    va108xx_embassy::init(
        &mut dp.sysconfig,
        &dp.irqsel,
        SYSCLK_FREQ,
        dp.tim23,
        dp.tim22,
    );

    let porta = PinsA::new(&mut dp.sysconfig, dp.porta);
    let mut led0 = porta.pa10.into_readable_push_pull_output();
    let mut led1 = porta.pa7.into_readable_push_pull_output();
    let mut led2 = porta.pa6.into_readable_push_pull_output();

    let tx_uart_a = porta.pa9.into_funsel_2();
    let rx_uart_a = porta.pa8.into_funsel_2();

    let uarta = uart::Uart::new_with_interrupt(
        &mut dp.sysconfig,
        50.MHz(),
        dp.uarta,
        (tx_uart_a, rx_uart_a),
        115200.Hz(),
        InterruptConfig::new(pac::Interrupt::OC2, true, true),
    );

    let tx_uart_b = porta.pa3.into_funsel_2();
    let rx_uart_b = porta.pa2.into_funsel_2();

    let uartb = uart::Uart::new_with_interrupt(
        &mut dp.sysconfig,
        50.MHz(),
        dp.uartb,
        (tx_uart_b, rx_uart_b),
        115200.Hz(),
        InterruptConfig::new(pac::Interrupt::OC3, true, true),
    );
    let (mut tx_uart_a, rx_uart_a) = uarta.split();
    let (tx_uart_b, rx_uart_b) = uartb.split();
    let (prod_uart_a, cons_uart_a) = QUEUE_UART_A.take().split();
    // Pass the producer to the interrupt handler.
    let (prod_uart_b, cons_uart_b) = QUEUE_UART_B.take().split();
    critical_section::with(|cs| {
        *PRODUCER_UART_A.borrow(cs).borrow_mut() = Some(prod_uart_a);
        *PRODUCER_UART_B.borrow(cs).borrow_mut() = Some(prod_uart_b);
        *CONSUMER_UART_B.borrow(cs).borrow_mut() = Some(cons_uart_b);
    });
    let mut async_rx_uart_a = RxAsync::new(rx_uart_a, cons_uart_a);
    let async_rx_uart_b = RxAsyncOverwriting::new(rx_uart_b, &CONSUMER_UART_B);
    spawner
        .spawn(uart_b_task(async_rx_uart_b, tx_uart_b))
        .unwrap();
    let mut buf = [0u8; 256];
    loop {
        rprintln!("Current time UART A: {}", Instant::now().as_secs());
        led0.toggle();
        led1.toggle();
        led2.toggle();
        let read_bytes = async_rx_uart_a.read(&mut buf).await.unwrap();
        let read_str = core::str::from_utf8(&buf[..read_bytes]).unwrap();
        rprintln!(
            "Read {} bytes asynchronously on UART A: {:?}",
            read_bytes,
            read_str
        );
        tx_uart_a.write_all(read_str.as_bytes()).unwrap();
    }
}

#[embassy_executor::task]
async fn uart_b_task(mut async_rx: RxAsyncOverwriting<pac::Uartb, 256>, mut tx: Tx<pac::Uartb>) {
    let mut buf = [0u8; 256];
    loop {
        rprintln!("Current time UART B: {}", Instant::now().as_secs());
        // Infallible asynchronous operation.
        let read_bytes = async_rx.read(&mut buf).await.unwrap();
        let read_str = core::str::from_utf8(&buf[..read_bytes]).unwrap();
        rprintln!(
            "Read {} bytes asynchronously on UART B: {:?}",
            read_bytes,
            read_str
        );
        tx.write_all(read_str.as_bytes()).unwrap();
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC2() {
    let mut prod =
        critical_section::with(|cs| PRODUCER_UART_A.borrow(cs).borrow_mut().take().unwrap());
    let errors = on_interrupt_rx(Bank::A, &mut prod);
    critical_section::with(|cs| *PRODUCER_UART_A.borrow(cs).borrow_mut() = Some(prod));
    // In a production app, we could use a channel to send the errors to the main task.
    if let Err(errors) = errors {
        rprintln!("UART A errors: {:?}", errors);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC3() {
    let mut prod =
        critical_section::with(|cs| PRODUCER_UART_B.borrow(cs).borrow_mut().take().unwrap());
    let errors = on_interrupt_rx_overwriting(Bank::B, &mut prod, &CONSUMER_UART_B);
    critical_section::with(|cs| *PRODUCER_UART_B.borrow(cs).borrow_mut() = Some(prod));
    // In a production app, we could use a channel to send the errors to the main task.
    if let Err(errors) = errors {
        rprintln!("UART B errors: {:?}", errors);
    }
}
