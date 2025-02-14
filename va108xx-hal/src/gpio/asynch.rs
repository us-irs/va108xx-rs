//! # Async GPIO functionality for the VA108xx family.
//!
//! This module provides the [InputPinAsync] and [InputDynPinAsync] which both implement
//! the [embedded_hal_async::digital::Wait] trait. These types allow for asynchronous waiting
//! on GPIO pins. Please note that this module does not specify/declare the interrupt handlers
//! which must be provided for async support to work. However, it provides one generic
//! [handler][on_interrupt_for_asynch_gpio] which should be called in ALL user interrupt handlers
//! which handle GPIO interrupts.
//!
//! # Example
//!
//! - [Async GPIO example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/embassy/src/bin/async-gpio.rs)
use core::future::Future;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::digital::InputPin;
use embedded_hal_async::digital::Wait;
use portable_atomic::AtomicBool;
use va108xx::{self as pac, Irqsel, Sysconfig};

use crate::InterruptConfig;

use super::{
    pin, DynGroup, DynPin, DynPinId, InputConfig, InterruptEdge, InvalidPinTypeError, Pin, PinId,
    NUM_GPIO_PINS, NUM_PINS_PORT_A,
};

static WAKERS: [AtomicWaker; NUM_GPIO_PINS] = [const { AtomicWaker::new() }; NUM_GPIO_PINS];
static EDGE_DETECTION: [AtomicBool; NUM_GPIO_PINS] =
    [const { AtomicBool::new(false) }; NUM_GPIO_PINS];

#[inline]
fn pin_id_to_offset(dyn_pin_id: DynPinId) -> usize {
    match dyn_pin_id.group {
        DynGroup::A => dyn_pin_id.num as usize,
        DynGroup::B => NUM_PINS_PORT_A + dyn_pin_id.num as usize,
    }
}

/// Generic interrupt handler for GPIO interrupts to support the async functionalities.
///
/// This handler will wake the correspoding wakers for the pins which triggered an interrupt
/// as well as updating the static edge detection structures. This allows the pin future to
/// complete async operations. The user should call this function in ALL interrupt handlers
/// which handle any GPIO interrupts.
#[inline]
pub fn on_interrupt_for_asynch_gpio() {
    let periphs = unsafe { pac::Peripherals::steal() };

    handle_interrupt_for_gpio_and_port(
        periphs.porta.irq_enb().read().bits(),
        periphs.porta.edge_status().read().bits(),
        0,
    );
    handle_interrupt_for_gpio_and_port(
        periphs.portb.irq_enb().read().bits(),
        periphs.portb.edge_status().read().bits(),
        NUM_PINS_PORT_A,
    );
}

// Uses the enabled interrupt register and the persistent edge status to capture all GPIO events.
#[inline]
fn handle_interrupt_for_gpio_and_port(mut irq_enb: u32, edge_status: u32, pin_base_offset: usize) {
    while irq_enb != 0 {
        let bit_pos = irq_enb.trailing_zeros() as usize;
        let bit_mask = 1 << bit_pos;

        WAKERS[pin_base_offset + bit_pos].wake();

        if edge_status & bit_mask != 0 {
            EDGE_DETECTION[pin_base_offset + bit_pos]
                .store(true, core::sync::atomic::Ordering::Relaxed);
        }

        // Clear the processed bit
        irq_enb &= !bit_mask;
    }
}

/// Input pin future which implements the [Future] trait.
///
/// Generally, you want to use the [InputPinAsync] or [InputDynPinAsync] types instead of this
/// which also implements the [embedded_hal_async::digital::Wait] trait. However, access to this
/// struture is granted  to allow writing custom async structures.
pub struct InputPinFuture {
    pin_id: DynPinId,
}

impl InputPinFuture {
    /// # Safety
    ///
    /// This calls [Self::new_with_dyn_pin] but uses [pac::Peripherals::steal] to get the system configuration
    /// and IRQ selection peripherals. Users must ensure that the registers and configuration
    /// related to this input pin are not being used elsewhere concurrently.
    pub unsafe fn new_unchecked_with_dyn_pin(
        pin: &mut DynPin,
        irq: pac::Interrupt,
        edge: InterruptEdge,
    ) -> Result<Self, InvalidPinTypeError> {
        let mut periphs = pac::Peripherals::steal();
        Self::new_with_dyn_pin(pin, irq, edge, &mut periphs.sysconfig, &mut periphs.irqsel)
    }

    pub fn new_with_dyn_pin(
        pin: &mut DynPin,
        irq: pac::Interrupt,
        edge: InterruptEdge,
        sys_cfg: &mut Sysconfig,
        irq_sel: &mut Irqsel,
    ) -> Result<Self, InvalidPinTypeError> {
        if !pin.is_input_pin() {
            return Err(InvalidPinTypeError(pin.mode()));
        }

        EDGE_DETECTION[pin_id_to_offset(pin.id())]
            .store(false, core::sync::atomic::Ordering::Relaxed);
        pin.configure_edge_interrupt(
            edge,
            InterruptConfig::new(irq, true, true),
            Some(sys_cfg),
            Some(irq_sel),
        )
        .unwrap();
        Ok(Self { pin_id: pin.id() })
    }

    /// # Safety
    ///
    /// This calls [Self::new_with_pin] but uses [pac::Peripherals::steal] to get the system configuration
    /// and IRQ selection peripherals. Users must ensure that the registers and configuration
    /// related to this input pin are not being used elsewhere concurrently.
    pub unsafe fn new_unchecked_with_pin<I: PinId, C: InputConfig>(
        pin: &mut Pin<I, pin::Input<C>>,
        irq: pac::Interrupt,
        edge: InterruptEdge,
    ) -> Self {
        let mut periphs = pac::Peripherals::steal();
        Self::new_with_pin(pin, irq, edge, &mut periphs.sysconfig, &mut periphs.irqsel)
    }

    pub fn new_with_pin<I: PinId, C: InputConfig>(
        pin: &mut Pin<I, pin::Input<C>>,
        irq: pac::Interrupt,
        edge: InterruptEdge,
        sys_cfg: &mut Sysconfig,
        irq_sel: &mut Irqsel,
    ) -> Self {
        EDGE_DETECTION[pin_id_to_offset(pin.id())]
            .store(false, core::sync::atomic::Ordering::Relaxed);
        pin.configure_edge_interrupt(
            edge,
            InterruptConfig::new(irq, true, true),
            Some(sys_cfg),
            Some(irq_sel),
        );
        Self { pin_id: pin.id() }
    }
}

impl Drop for InputPinFuture {
    fn drop(&mut self) {
        let periphs = unsafe { pac::Peripherals::steal() };
        if self.pin_id.group == DynGroup::A {
            periphs
                .porta
                .irq_enb()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id.num)) });
        } else {
            periphs
                .porta
                .irq_enb()
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id.num)) });
        }
    }
}

impl Future for InputPinFuture {
    type Output = ();
    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let idx = pin_id_to_offset(self.pin_id);
        WAKERS[idx].register(cx.waker());
        if EDGE_DETECTION[idx].swap(false, core::sync::atomic::Ordering::Relaxed) {
            return core::task::Poll::Ready(());
        }
        core::task::Poll::Pending
    }
}

pub struct InputDynPinAsync {
    pin: DynPin,
    irq: pac::Interrupt,
}

impl InputDynPinAsync {
    /// Create a new asynchronous input pin from a [DynPin]. The interrupt ID to be used must be
    /// passed as well and is used to route and enable the interrupt.
    ///
    /// Please note that the interrupt handler itself must be provided by the user and the
    /// generic [on_interrupt_for_asynch_gpio] function must be called inside that function for
    /// the asynchronous functionality to work.
    pub fn new(pin: DynPin, irq: pac::Interrupt) -> Result<Self, InvalidPinTypeError> {
        if !pin.is_input_pin() {
            return Err(InvalidPinTypeError(pin.mode()));
        }
        Ok(Self { pin, irq })
    }

    /// Asynchronously wait until the pin is high.
    ///
    /// This returns immediately if the pin is already high.
    pub async fn wait_for_high(&mut self) {
        let fut = unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_dyn_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::LowToHigh,
            )
            .unwrap()
        };
        if self.pin.is_high().unwrap() {
            return;
        }
        fut.await;
    }

    /// Asynchronously wait until the pin is low.
    ///
    /// This returns immediately if the pin is already high.
    pub async fn wait_for_low(&mut self) {
        let fut = unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_dyn_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::HighToLow,
            )
            .unwrap()
        };
        if self.pin.is_low().unwrap() {
            return;
        }
        fut.await;
    }

    /// Asynchronously wait until the pin sees a falling edge.
    pub async fn wait_for_falling_edge(&mut self) {
        unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_dyn_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::HighToLow,
            )
            .unwrap()
        }
        .await;
    }

    /// Asynchronously wait until the pin sees a rising edge.
    pub async fn wait_for_rising_edge(&mut self) {
        unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_dyn_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::LowToHigh,
            )
            .unwrap()
        }
        .await;
    }

    /// Asynchronously wait until the pin sees any edge (either rising or falling).
    pub async fn wait_for_any_edge(&mut self) {
        unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_dyn_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::BothEdges,
            )
            .unwrap()
        }
        .await;
    }

    pub fn release(self) -> DynPin {
        self.pin
    }
}

impl embedded_hal::digital::ErrorType for InputDynPinAsync {
    type Error = core::convert::Infallible;
}

impl Wait for InputDynPinAsync {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}

pub struct InputPinAsync<I: PinId, C: InputConfig> {
    pin: Pin<I, pin::Input<C>>,
    irq: pac::Interrupt,
}

impl<I: PinId, C: InputConfig> InputPinAsync<I, C> {
    /// Create a new asynchronous input pin from a typed [Pin]. The interrupt ID to be used must be
    /// passed as well and is used to route and enable the interrupt.
    ///
    /// Please note that the interrupt handler itself must be provided by the user and the
    /// generic [on_interrupt_for_asynch_gpio] function must be called inside that function for
    /// the asynchronous functionality to work.
    pub fn new(pin: Pin<I, pin::Input<C>>, irq: pac::Interrupt) -> Self {
        Self { pin, irq }
    }

    /// Asynchronously wait until the pin is high.
    ///
    /// This returns immediately if the pin is already high.
    pub async fn wait_for_high(&mut self) {
        let fut = unsafe {
            InputPinFuture::new_unchecked_with_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::LowToHigh,
            )
        };
        if self.pin.is_high().unwrap() {
            return;
        }
        fut.await;
    }

    /// Asynchronously wait until the pin is low.
    ///
    /// This returns immediately if the pin is already high.
    pub async fn wait_for_low(&mut self) {
        let fut = unsafe {
            InputPinFuture::new_unchecked_with_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::HighToLow,
            )
        };
        if self.pin.is_low().unwrap() {
            return;
        }
        fut.await;
    }

    /// Asynchronously wait until the pin sees falling edge.
    pub async fn wait_for_falling_edge(&mut self) {
        unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::HighToLow,
            )
        }
        .await;
    }

    /// Asynchronously wait until the pin sees rising edge.
    pub async fn wait_for_rising_edge(&mut self) {
        unsafe {
            // Unwrap okay, checked pin in constructor.
            InputPinFuture::new_unchecked_with_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::LowToHigh,
            )
        }
        .await;
    }

    /// Asynchronously wait until the pin sees any edge (either rising or falling).
    pub async fn wait_for_any_edge(&mut self) {
        unsafe {
            InputPinFuture::new_unchecked_with_pin(
                &mut self.pin,
                self.irq,
                InterruptEdge::BothEdges,
            )
        }
        .await;
    }

    pub fn release(self) -> Pin<I, pin::Input<C>> {
        self.pin
    }
}
impl<I: PinId, C: InputConfig> embedded_hal::digital::ErrorType for InputPinAsync<I, C> {
    type Error = core::convert::Infallible;
}

impl<I: PinId, C: InputConfig> Wait for InputPinAsync<I, C> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.wait_for_any_edge().await;
        Ok(())
    }
}
