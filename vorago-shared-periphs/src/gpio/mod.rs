use core::convert::Infallible;

pub use embedded_hal::digital::PinState;
pub use ll::PinId;
pub use ll::{Port, Pull};

use crate::ioconfig::regs::FunSel;

pub mod asynch;
pub mod ll;
pub mod regs;

pub trait PinMarker {
    const ID: ll::PinId;
}

pub struct Pin<I: PinMarker> {
    phantom: core::marker::PhantomData<I>,
}

impl<I: PinMarker> Pin<I> {
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            phantom: core::marker::PhantomData,
        }
    }
}

pub struct Output(ll::LowLevelGpio);

impl Output {
    pub fn new<I: PinMarker>(_pin: Pin<I>, init_level: PinState) -> Self {
        let mut ll = ll::LowLevelGpio::new(I::ID);
        ll.configure_as_output_push_pull(init_level);
        Output(ll)
    }

    #[inline]
    pub fn port(&self) -> Port {
        self.0.port()
    }

    #[inline]
    pub fn offset(&self) -> usize {
        self.0.offset()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.0.set_high();
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.0.set_low();
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        self.0.is_set_high()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self.0.is_set_low()
    }
}

impl embedded_hal::digital::ErrorType for Output {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for Output {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_high();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Output {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_high())
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_set_low())
    }

    /// Toggle pin output with dedicated HW feature.
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.0.toggle();
        Ok(())
    }
}

pub struct Input(ll::LowLevelGpio);

impl Input {
    pub fn new_floating<I: PinMarker>(_pin: Pin<I>) -> Self {
        let mut ll = ll::LowLevelGpio::new(I::ID);
        ll.configure_as_input_floating();
        Input(ll)
    }

    pub fn new_with_pull<I: PinMarker>(_pin: Pin<I>, pull: Pull) -> Self {
        let mut ll = ll::LowLevelGpio::new(I::ID);
        ll.configure_as_input_with_pull(pull);
        Input(ll)
    }

    #[inline]
    pub fn id(&self) -> PinId {
        self.0.id()
    }

    #[cfg(feature = "vor1x")]
    #[inline]
    pub fn enable_interrupt(&mut self, irq_cfg: crate::InterruptConfig) {
        self.0.enable_interrupt(irq_cfg);
    }

    #[inline]
    pub fn configure_edge_interrupt(&mut self, edge: ll::InterruptEdge) {
        self.0.configure_edge_interrupt(edge);
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self.0.is_low()
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.0.is_high()
    }
}

impl embedded_hal::digital::ErrorType for Input {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Input {
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_low())
    }

    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.is_high())
    }
}

pub enum PinMode {
    InputFloating,
    InputWithPull(Pull),
    OutputPushPull,
    OutputOpenDrain,
}

impl PinMode {
    pub fn is_input(&self) -> bool {
        matches!(self, PinMode::InputFloating | PinMode::InputWithPull(_))
    }

    pub fn is_output(&self) -> bool {
        !self.is_input()
    }
}

pub struct Flex {
    ll: ll::LowLevelGpio,
    mode: PinMode,
}

impl Flex {
    pub fn new<I: PinMarker>(_pin: Pin<I>) -> Self {
        let mut ll = ll::LowLevelGpio::new(I::ID);
        ll.configure_as_input_floating();
        Flex {
            ll,
            mode: PinMode::InputFloating,
        }
    }

    #[inline]
    pub fn port(&self) -> Port {
        self.ll.port()
    }

    #[inline]
    pub fn offset(&self) -> usize {
        self.ll.offset()
    }

    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    pub fn is_low(&self) -> bool {
        self.ll.is_low()
    }

    /// Reads the input state of the pin, regardless of configured mode.
    #[inline]
    pub fn is_high(&self) -> bool {
        self.ll.is_high()
    }

    /// If the pin is configured as an input pin, this function does nothing.
    #[inline]
    pub fn set_low(&mut self) {
        if !self.mode.is_input() {
            return;
        }
        self.ll.set_low();
    }

    /// If the pin is configured as an input pin, this function does nothing.
    #[inline]
    pub fn set_high(&mut self) {
        if !self.mode.is_input() {
            return;
        }
        self.ll.set_high();
    }
}

impl embedded_hal::digital::ErrorType for Flex {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Flex {
    /// Reads the input state of the pin, regardless of configured mode.
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_low())
    }

    /// Reads the input state of the pin, regardless of configured mode.
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_high())
    }
}

impl embedded_hal::digital::OutputPin for Flex {
    /// If the pin is configured as an input pin, this function does nothing.
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }

    /// If the pin is configured as an input pin, this function does nothing.
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }
}

impl embedded_hal::digital::StatefulOutputPin for Flex {
    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_high())
    }

    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.ll.is_set_low())
    }

    /// Toggle pin output.
    ///
    /// If the pin is not configured as a stateful output pin like Output Push-Pull, the result
    /// of this function is undefined.
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.ll.toggle();
        Ok(())
    }
}

pub struct IoPeriphPin {
    ll: ll::LowLevelGpio,
    fun_sel: FunSel,
}

impl IoPeriphPin {
    pub fn new<I: PinMarker>(_pin: Pin<I>, fun_sel: FunSel, pull: Option<Pull>) -> Self {
        let mut ll = ll::LowLevelGpio::new(I::ID);
        ll.configure_as_peripheral_pin(fun_sel, pull);
        IoPeriphPin { ll, fun_sel }
    }

    pub fn port(&self) -> Port {
        self.ll.port()
    }

    pub fn offset(&self) -> usize {
        self.ll.offset()
    }

    pub fn fun_sel(&self) -> FunSel {
        self.fun_sel
    }
}
