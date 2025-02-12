//! # Async UART reception functionality for the VA108xx family.
//!
//! This module provides the [RxAsync] and [RxAsyncSharedConsumer] struct which both implement the
//! [embedded_io_async::Read] trait.
//! This trait allows for asynchronous reception of data streams. Please note that this module does
//! not specify/declare the interrupt handlers which must be provided for async support to work.
//! However, it provides four interrupt handlers:
//!
//! - [on_interrupt_uart_a]
//! - [on_interrupt_uart_b]
//! - [on_interrupt_uart_a_overwriting]
//! - [on_interrupt_uart_b_overwriting]
//!
//! The first two are used for the [RxAsync] struct, while the latter two are used with the
//! [RxAsyncSharedConsumer] struct. The later two will overwrite old values in the used ring buffer.
//!
//! Error handling is performed in the user interrupt handler by checking the [AsyncUartErrors]
//! structure returned by the interrupt handlers.
//!
//! # Example
//!
//! - [Async UART RX example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/embassy/src/bin/async-uart-rx.rs)
use core::{cell::RefCell, convert::Infallible, future::Future, sync::atomic::Ordering};

use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embedded_io::ErrorType;
use heapless::spsc::Consumer;
use portable_atomic::AtomicBool;
use va108xx as pac;

use super::{Instance, Rx, RxError, UartErrors};

static UART_RX_WAKERS: [AtomicWaker; 2] = [const { AtomicWaker::new() }; 2];
static RX_READ_ACTIVE: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];
static RX_HAS_DATA: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

struct RxFuture {
    uart_idx: usize,
}

impl RxFuture {
    pub fn new<Uart: Instance>(_rx: &mut Rx<Uart>) -> Self {
        RX_READ_ACTIVE[Uart::IDX as usize].store(true, Ordering::Relaxed);
        Self {
            uart_idx: Uart::IDX as usize,
        }
    }
}

impl Future for RxFuture {
    type Output = Result<(), RxError>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        UART_RX_WAKERS[self.uart_idx].register(cx.waker());
        if RX_HAS_DATA[self.uart_idx].load(Ordering::Relaxed) {
            return core::task::Poll::Ready(Ok(()));
        }
        core::task::Poll::Pending
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AsyncUartErrors {
    /// Queue has overflowed, data might have been lost.
    pub queue_overflow: bool,
    /// UART errors.
    pub uart_errors: UartErrors,
}

fn on_interrupt_handle_rx_errors<Uart: Instance>(uart: &Uart) -> Option<UartErrors> {
    let rx_status = uart.rxstatus().read();
    if rx_status.rxovr().bit_is_set()
        || rx_status.rxfrm().bit_is_set()
        || rx_status.rxpar().bit_is_set()
    {
        let mut errors_val = UartErrors::default();

        if rx_status.rxovr().bit_is_set() {
            errors_val.overflow = true;
        }
        if rx_status.rxfrm().bit_is_set() {
            errors_val.framing = true;
        }
        if rx_status.rxpar().bit_is_set() {
            errors_val.parity = true;
        }
        return Some(errors_val);
    }
    None
}

fn on_interrupt_rx_common_post_processing<Uart: Instance>(
    uart: &Uart,
    rx_enabled: bool,
    read_some_data: bool,
    irq_end: u32,
) -> Option<UartErrors> {
    if read_some_data {
        RX_HAS_DATA[Uart::IDX as usize].store(true, Ordering::Relaxed);
        if RX_READ_ACTIVE[Uart::IDX as usize].load(Ordering::Relaxed) {
            UART_RX_WAKERS[Uart::IDX as usize].wake();
        }
    }

    let mut errors = None;
    // Check for RX errors
    if rx_enabled {
        errors = on_interrupt_handle_rx_errors(uart);
    }

    // Clear the interrupt status bits
    uart.irq_clr().write(|w| unsafe { w.bits(irq_end) });
    errors
}

/// Interrupt handler for UART A.
///
/// Should be called in the user interrupt handler to enable
/// asynchronous reception. This variant will overwrite old data in the ring buffer in case
/// the ring buffer is full.
pub fn on_interrupt_uart_a_overwriting<const N: usize>(
    prod: &mut heapless::spsc::Producer<u8, N>,
    shared_consumer: &Mutex<RefCell<Option<heapless::spsc::Consumer<'static, u8, N>>>>,
) -> Result<(), AsyncUartErrors> {
    on_interrupt_rx_async_heapless_queue_overwriting(
        unsafe { pac::Uarta::steal() },
        prod,
        shared_consumer,
    )
}

/// Interrupt handler for UART B.
///
/// Should be called in the user interrupt handler to enable
/// asynchronous reception. This variant will overwrite old data in the ring buffer in case
/// the ring buffer is full.
pub fn on_interrupt_uart_b_overwriting<const N: usize>(
    prod: &mut heapless::spsc::Producer<u8, N>,
    shared_consumer: &Mutex<RefCell<Option<heapless::spsc::Consumer<'static, u8, N>>>>,
) -> Result<(), AsyncUartErrors> {
    on_interrupt_rx_async_heapless_queue_overwriting(
        unsafe { pac::Uartb::steal() },
        prod,
        shared_consumer,
    )
}

pub fn on_interrupt_rx_async_heapless_queue_overwriting<Uart: Instance, const N: usize>(
    uart: Uart,
    prod: &mut heapless::spsc::Producer<u8, N>,
    shared_consumer: &Mutex<RefCell<Option<heapless::spsc::Consumer<'static, u8, N>>>>,
) -> Result<(), AsyncUartErrors> {
    let irq_end = uart.irq_end().read();
    let enb_status = uart.enable().read();
    let rx_enabled = enb_status.rxenable().bit_is_set();
    let mut read_some_data = false;
    let mut queue_overflow = false;

    // Half-Full interrupt. We have a guaranteed amount of data we can read.
    if irq_end.irq_rx().bit_is_set() {
        let available_bytes = uart.rxfifoirqtrg().read().bits() as usize;

        // If this interrupt bit is set, the trigger level is available at the very least.
        // Read everything as fast as possible
        for _ in 0..available_bytes {
            let byte = uart.data().read().bits();
            if !prod.ready() {
                queue_overflow = true;
                critical_section::with(|cs| {
                    let mut cons_ref = shared_consumer.borrow(cs).borrow_mut();
                    cons_ref.as_mut().unwrap().dequeue();
                });
            }
            prod.enqueue(byte as u8).ok();
        }
        read_some_data = true;
    }

    // Timeout, empty the FIFO completely.
    if irq_end.irq_rx_to().bit_is_set() {
        while uart.rxstatus().read().rdavl().bit_is_set() {
            // While there is data in the FIFO, write it into the reception buffer
            let byte = uart.data().read().bits();
            if !prod.ready() {
                queue_overflow = true;
                critical_section::with(|cs| {
                    let mut cons_ref = shared_consumer.borrow(cs).borrow_mut();
                    cons_ref.as_mut().unwrap().dequeue();
                });
            }
            prod.enqueue(byte as u8).ok();
        }
        read_some_data = true;
    }

    let uart_errors =
        on_interrupt_rx_common_post_processing(&uart, rx_enabled, read_some_data, irq_end.bits());
    if uart_errors.is_some() || queue_overflow {
        return Err(AsyncUartErrors {
            queue_overflow,
            uart_errors: uart_errors.unwrap_or_default(),
        });
    }
    Ok(())
}

/// Interrupt handler for UART A.
///
/// Should be called in the user interrupt handler to enable asynchronous reception.
pub fn on_interrupt_uart_a<const N: usize>(
    prod: &mut heapless::spsc::Producer<'_, u8, N>,
) -> Result<(), AsyncUartErrors> {
    on_interrupt_rx_async_heapless_queue(unsafe { pac::Uarta::steal() }, prod)
}

/// Interrupt handler for UART B.
///
/// Should be called in the user interrupt handler to enable asynchronous reception.
pub fn on_interrupt_uart_b<const N: usize>(
    prod: &mut heapless::spsc::Producer<'_, u8, N>,
) -> Result<(), AsyncUartErrors> {
    on_interrupt_rx_async_heapless_queue(unsafe { pac::Uartb::steal() }, prod)
}

pub fn on_interrupt_rx_async_heapless_queue<Uart: Instance, const N: usize>(
    uart: Uart,
    prod: &mut heapless::spsc::Producer<'_, u8, N>,
) -> Result<(), AsyncUartErrors> {
    //let uart = unsafe { Uart::steal() };
    let irq_end = uart.irq_end().read();
    let enb_status = uart.enable().read();
    let rx_enabled = enb_status.rxenable().bit_is_set();
    let mut read_some_data = false;
    let mut queue_overflow = false;

    // Half-Full interrupt. We have a guaranteed amount of data we can read.
    if irq_end.irq_rx().bit_is_set() {
        let available_bytes = uart.rxfifoirqtrg().read().bits() as usize;

        // If this interrupt bit is set, the trigger level is available at the very least.
        // Read everything as fast as possible
        for _ in 0..available_bytes {
            let byte = uart.data().read().bits();
            if !prod.ready() {
                queue_overflow = true;
            }
            prod.enqueue(byte as u8).ok();
        }
        read_some_data = true;
    }

    // Timeout, empty the FIFO completely.
    if irq_end.irq_rx_to().bit_is_set() {
        while uart.rxstatus().read().rdavl().bit_is_set() {
            // While there is data in the FIFO, write it into the reception buffer
            let byte = uart.data().read().bits();
            if !prod.ready() {
                queue_overflow = true;
            }
            prod.enqueue(byte as u8).ok();
        }
        read_some_data = true;
    }

    let uart_errors =
        on_interrupt_rx_common_post_processing(&uart, rx_enabled, read_some_data, irq_end.bits());
    if uart_errors.is_some() || queue_overflow {
        return Err(AsyncUartErrors {
            queue_overflow,
            uart_errors: uart_errors.unwrap_or_default(),
        });
    }
    Ok(())
}

struct ActiveReadGuard(usize);

impl Drop for ActiveReadGuard {
    fn drop(&mut self) {
        RX_READ_ACTIVE[self.0].store(false, Ordering::Relaxed);
    }
}

/// Core data structure to allow asynchrnous UART reception.
///
/// If the ring buffer becomes full, data will be lost.
pub struct RxAsync<Uart: Instance, const N: usize> {
    rx: Rx<Uart>,
    pub queue: heapless::spsc::Consumer<'static, u8, N>,
}

impl<Uart: Instance, const N: usize> ErrorType for RxAsync<Uart, N> {
    /// Error reporting is done using atomic booleans and the [get_and_clear_errors] function.
    type Error = Infallible;
}

impl<Uart: Instance, const N: usize> RxAsync<Uart, N> {
    /// Create a new asynchronous receiver.
    ///
    /// The passed [heapless::spsc::Consumer] will be used to asynchronously receive data which
    /// is filled by the interrupt handler.
    pub fn new(mut rx: Rx<Uart>, queue: heapless::spsc::Consumer<'static, u8, N>) -> Self {
        rx.disable_interrupts();
        rx.disable();
        rx.clear_fifo();
        // Enable those together.
        critical_section::with(|_| {
            rx.enable_interrupts();
            rx.enable();
        });
        Self { rx, queue }
    }
}

impl<Uart: Instance, const N: usize> embedded_io_async::Read for RxAsync<Uart, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Need to wait for the IRQ to read data and set this flag. If the queue is not
        // empty, we can read data immediately.
        if self.queue.len() == 0 {
            RX_HAS_DATA[Uart::IDX as usize].store(false, Ordering::Relaxed);
        }
        let _guard = ActiveReadGuard(Uart::IDX as usize);
        let mut handle_data_in_queue = |consumer: &mut heapless::spsc::Consumer<'static, u8, N>| {
            let data_to_read = consumer.len().min(buf.len());
            for byte in buf.iter_mut().take(data_to_read) {
                // We own the consumer and we checked that the amount of data is guaranteed to be available.
                *byte = unsafe { consumer.dequeue_unchecked() };
            }
            data_to_read
        };
        let fut = RxFuture::new(&mut self.rx);
        // Data is available, so read that data immediately.
        let read_data = handle_data_in_queue(&mut self.queue);
        if read_data > 0 {
            return Ok(read_data);
        }
        // Await data.
        let _ = fut.await;
        Ok(handle_data_in_queue(&mut self.queue))
    }
}

/// Core data structure to allow asynchrnous UART reception.
///
/// If the ring buffer becomes full, the oldest data will be overwritten when using the
/// [on_interrupt_uart_a_overwriting] and [on_interrupt_uart_b_overwriting] interrupt handlers.
pub struct RxAsyncSharedConsumer<Uart: Instance, const N: usize> {
    rx: Rx<Uart>,
    queue: &'static Mutex<RefCell<Option<Consumer<'static, u8, N>>>>,
}

impl<Uart: Instance, const N: usize> ErrorType for RxAsyncSharedConsumer<Uart, N> {
    /// Error reporting is done using atomic booleans and the [get_and_clear_errors] function.
    type Error = Infallible;
}

impl<Uart: Instance, const N: usize> RxAsyncSharedConsumer<Uart, N> {
    /// Create a new asynchronous receiver.
    ///
    /// The passed shared [heapless::spsc::Consumer] will be used to asynchronously receive data
    /// which is filled by the interrupt handler. The shared property allows using it in the
    /// interrupt handler to overwrite old data.
    pub fn new(
        mut rx: Rx<Uart>,
        queue: &'static Mutex<RefCell<Option<heapless::spsc::Consumer<'static, u8, N>>>>,
    ) -> Self {
        rx.disable_interrupts();
        rx.disable();
        rx.clear_fifo();
        // Enable those together.
        critical_section::with(|_| {
            rx.enable_interrupts();
            rx.enable();
        });
        Self { rx, queue }
    }
}

impl<Uart: Instance, const N: usize> embedded_io_async::Read for RxAsyncSharedConsumer<Uart, N> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Need to wait for the IRQ to read data and set this flag. If the queue is not
        // empty, we can read data immediately.

        critical_section::with(|cs| {
            let queue = self.queue.borrow(cs);
            if queue.borrow().as_ref().unwrap().len() == 0 {
                RX_HAS_DATA[Uart::IDX as usize].store(false, Ordering::Relaxed);
            }
        });
        let _guard = ActiveReadGuard(Uart::IDX as usize);
        let mut handle_data_in_queue = || {
            critical_section::with(|cs| {
                let mut consumer_ref = self.queue.borrow(cs).borrow_mut();
                let consumer = consumer_ref.as_mut().unwrap();
                let data_to_read = consumer.len().min(buf.len());
                for byte in buf.iter_mut().take(data_to_read) {
                    // We own the consumer and we checked that the amount of data is guaranteed to be available.
                    *byte = unsafe { consumer.dequeue_unchecked() };
                }
                data_to_read
            })
        };
        let fut = RxFuture::new(&mut self.rx);
        // Data is available, so read that data immediately.
        let read_data = handle_data_in_queue();
        if read_data > 0 {
            return Ok(read_data);
        }
        // Await data.
        let _ = fut.await;
        let read_data = handle_data_in_queue();
        Ok(read_data)
    }
}
