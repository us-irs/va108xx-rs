//! # Async GPIO functionality for the VA108xx family.
//!
//! This module provides the [InputPinAsync] and [InputDynPinAsync] which both implement
//! the [embedded_hal_async::digital::Wait] trait. These types allow for asynchronous waiting
//! on GPIO pins. Please note that this module does not specify/declare the interrupt handlers
//! which must be provided for async support to work. However, it provides the
//! [on_interrupt_for_async_gpio_for_port] generic interrupt handler. This should be called in all
//! IRQ functions which handle any GPIO interrupts with the corresponding [Port] argument.
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
    pin, DynPin, DynPinId, InputConfig, InterruptEdge, InvalidPinTypeError, Pin, PinId, Port,
    NUM_PINS_PORT_A, NUM_PINS_PORT_B,
};

static WAKERS_FOR_PORT_A: [AtomicWaker; NUM_PINS_PORT_A] =
    [const { AtomicWaker::new() }; NUM_PINS_PORT_A];
static WAKERS_FOR_PORT_B: [AtomicWaker; NUM_PINS_PORT_B] =
    [const { AtomicWaker::new() }; NUM_PINS_PORT_B];
static EDGE_DETECTION_PORT_A: [AtomicBool; NUM_PINS_PORT_A] =
    [const { AtomicBool::new(false) }; NUM_PINS_PORT_A];
static EDGE_DETECTION_PORT_B: [AtomicBool; NUM_PINS_PORT_B] =
    [const { AtomicBool::new(false) }; NUM_PINS_PORT_B];

/// Generic interrupt handler for GPIO interrupts on a specific port to support async functionalities
///
/// This function should be called in all interrupt handlers which handle any GPIO interrupts
/// matching the [Port] argument.
/// The handler will wake the corresponding wakers for the pins that triggered an interrupts
/// as well as update the static edge detection structures. This allows the pin future tocomplete
/// complete async operations.
pub fn on_interrupt_for_async_gpio_for_port(port: Port) {
    let periphs = unsafe { pac::Peripherals::steal() };

    let (irq_enb, edge_status, wakers, edge_detection) = match port {
        Port::A => (
            periphs.porta.irq_enb().read().bits(),
            periphs.porta.edge_status().read().bits(),
            WAKERS_FOR_PORT_A.as_ref(),
            EDGE_DETECTION_PORT_A.as_ref(),
        ),
        Port::B => (
            periphs.portb.irq_enb().read().bits(),
            periphs.portb.edge_status().read().bits(),
            WAKERS_FOR_PORT_B.as_ref(),
            EDGE_DETECTION_PORT_B.as_ref(),
        ),
    };

    on_interrupt_for_port(irq_enb, edge_status, wakers, edge_detection);
}

#[inline]
fn on_interrupt_for_port(
    mut irq_enb: u32,
    edge_status: u32,
    wakers: &'static [AtomicWaker],
    edge_detection: &'static [AtomicBool],
) {
    while irq_enb != 0 {
        let bit_pos = irq_enb.trailing_zeros() as usize;
        let bit_mask = 1 << bit_pos;

        wakers[bit_pos].wake();

        if edge_status & bit_mask != 0 {
            edge_detection[bit_pos].store(true, core::sync::atomic::Ordering::Relaxed);

            // Clear the processed bit
            irq_enb &= !bit_mask;
        }
    }
}

/// Input pin future which implements the [Future] trait.
///
/// Generally, you want to use the [InputPinAsync] or [InputDynPinAsync] types instead of this
/// which also implements the [embedded_hal_async::digital::Wait] trait. However, access to this
/// struture is granted  to allow writing custom async structures.
pub struct InputPinFuture {
    pin_id: DynPinId,
    waker_group: &'static [AtomicWaker],
    edge_detection_group: &'static [AtomicBool],
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

    #[inline]
    pub fn pin_group_to_waker_and_edge_detection_group(
        group: Port,
    ) -> (&'static [AtomicWaker], &'static [AtomicBool]) {
        match group {
            Port::A => (WAKERS_FOR_PORT_A.as_ref(), EDGE_DETECTION_PORT_A.as_ref()),
            Port::B => (WAKERS_FOR_PORT_B.as_ref(), EDGE_DETECTION_PORT_B.as_ref()),
        }
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

        let (waker_group, edge_detection_group) =
            Self::pin_group_to_waker_and_edge_detection_group(pin.id().group);
        edge_detection_group[pin.id().num as usize]
            .store(false, core::sync::atomic::Ordering::Relaxed);
        pin.configure_edge_interrupt(
            edge,
            InterruptConfig::new(irq, true, true),
            Some(sys_cfg),
            Some(irq_sel),
        )
        .unwrap();
        Ok(Self {
            pin_id: pin.id(),
            waker_group,
            edge_detection_group,
        })
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
        let (waker_group, edge_detection_group) =
            Self::pin_group_to_waker_and_edge_detection_group(pin.id().group);
        edge_detection_group[pin.id().num as usize]
            .store(false, core::sync::atomic::Ordering::Relaxed);
        pin.configure_edge_interrupt(
            edge,
            InterruptConfig::new(irq, true, true),
            Some(sys_cfg),
            Some(irq_sel),
        );
        Self {
            pin_id: pin.id(),
            edge_detection_group,
            waker_group,
        }
    }
}

impl Drop for InputPinFuture {
    fn drop(&mut self) {
        let periphs = unsafe { pac::Peripherals::steal() };
        if self.pin_id.group == Port::A {
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
        let idx = self.pin_id.num as usize;
        self.waker_group[idx].register(cx.waker());
        if self.edge_detection_group[idx].swap(false, core::sync::atomic::Ordering::Relaxed) {
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
    /// generic [on_interrupt_for_async_gpio_for_port] function must be called inside that function
    /// for the asynchronous functionality to work.
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
    /// generic [on_interrupt_for_async_gpio_for_port] function must be called inside that function
    /// for the asynchronous functionality to work.
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
