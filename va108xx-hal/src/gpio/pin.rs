//! # Type-level module for GPIO pins
//!
//! This documentation is strongly based on the
//! [atsamd documentation](https://docs.rs/atsamd-hal/latest/atsamd_hal/gpio/pin/index.html).
//!
//! This module provides a type-level API for GPIO pins. It uses the type system
//! to track the state of pins at compile-time. Representing GPIO pins in this
//! manner incurs no run-time overhead. Each [`Pin`] struct is zero-sized, so
//! there is no data to copy around. Instead, real code is generated as a side
//! effect of type transformations, and the resulting assembly is nearly
//! identical to the equivalent, hand-written C.
//!
//! To track the state of pins at compile-time, this module uses traits to
//! represent [type classes] and types as instances of those type classes. For
//! example, the trait [`InputConfig`] acts as a [type-level enum] of the
//! available input configurations, and the types [`Floating`], [`PullDown`] and
//! [`PullUp`] are its type-level variants.
//!
//! Type-level [`Pin`]s are parameterized by two type-level enums, [`PinId`] and
//! [`PinMode`].
//!
//! ```
//! pub struct Pin<I, M>
//! where
//!     I: PinId,
//!     M: PinMode,
//! {
//!     // ...
//! }
//! ```
//!
//! A [PinId] identifies a pin by it's group (A or B) and pin number. Each
//! [PinId] instance is named according to its datasheet identifier, e.g.
//! [PA2].
//!
//! A [PinMode] represents the various pin modes. The available [PinMode]
//! variants are [`Input`], [`Output`] and [`Alternate`], each with its own
//! corresponding configurations.
//!
//! It is not possible for users to create new instances of a [`Pin`]. Singleton
//! instances of each pin are made available to users through the PinsX
//! struct.
//!
//! Example for the pins of PORT A:
//!
//! To create the [PinsA] struct, users must supply the PAC
//! [Port](crate::pac::Porta) peripheral. The [PinsA] struct takes
//! ownership of the [Porta] and provides the corresponding pins. Each [`Pin`]
//! within the [PinsA] struct can be moved out and used individually.
//!
//!
//! ```
//! let mut peripherals = Peripherals::take().unwrap();
//! let pinsa = PinsA::new(peripherals.PORT);
//! ```
//!
//! Pins can be converted between modes using several different methods.
//!
//! ```no_run
//! // Use one of the literal function names
//! let pa0 = pinsa.pa0.into_floating_input();
//! // Use a generic method and one of the `PinMode` variant types
//! let pa0 = pinsa.pa0.into_mode::<FloatingInput>();
//! // Specify the target type and use `From`/`Into`
//! let pa0: Pin<PA0, FloatingInput> = pinsa.pa27.into();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for each [`Pin`]
//! in the corresponding [`PinMode`]s, namely: [embedded_hal::digital::InputPin],
//! [embedded_hal::digital::OutputPin] and [embedded_hal::digital::StatefulOutputPin].
use super::dynpin::{DynAlternate, DynInput, DynOutput, DynPinId, DynPinMode};
use super::{DynPin, InputPinAsync, InterruptEdge, InterruptLevel, PinState, Port};
use crate::{
    pac::{Porta, Portb},
    typelevel::Sealed,
};
use core::convert::Infallible;
use core::marker::PhantomData;
use core::mem::transmute;
use paste::paste;

//==================================================================================================
// Input configuration
//==================================================================================================

/// Type-level enum for input configurations
///
/// The valid options are [Floating], [PullDown] and [PullUp].
pub trait InputConfig: Sealed {
    /// Corresponding [DynInput]
    const DYN: DynInput;
}

#[derive(Debug)]
pub enum Floating {}
#[derive(Debug)]
pub enum PullDown {}
#[derive(Debug)]
pub enum PullUp {}

impl InputConfig for Floating {
    const DYN: DynInput = DynInput::Floating;
}
impl InputConfig for PullDown {
    const DYN: DynInput = DynInput::PullDown;
}
impl InputConfig for PullUp {
    const DYN: DynInput = DynInput::PullUp;
}

impl Sealed for Floating {}
impl Sealed for PullDown {}
impl Sealed for PullUp {}

/// Type-level variant of [`PinMode`] for floating input mode
pub type InputFloating = Input<Floating>;
/// Type-level variant of [`PinMode`] for pull-down input mode
pub type InputPullDown = Input<PullDown>;
/// Type-level variant of [`PinMode`] for pull-up input mode
pub type InputPullUp = Input<PullUp>;

/// Type-level variant of [`PinMode`] for input modes
///
/// Type `C` is one of three input configurations: [`Floating`], [`PullDown`] or
/// [`PullUp`]
#[derive(Debug)]
pub struct Input<C: InputConfig> {
    cfg: PhantomData<C>,
}

impl<C: InputConfig> Sealed for Input<C> {}

#[derive(Debug, PartialEq, Eq)]
pub enum FilterType {
    SystemClock = 0,
    DirectInputWithSynchronization = 1,
    FilterOneClockCycle = 2,
    FilterTwoClockCycles = 3,
    FilterThreeClockCycles = 4,
    FilterFourClockCycles = 5,
}

pub use crate::clock::FilterClkSel;

//==================================================================================================
// Output configuration
//==================================================================================================

pub trait OutputConfig: Sealed {
    const DYN: DynOutput;
}

pub trait ReadableOutput: Sealed {}

/// Type-level variant of [`OutputConfig`] for a push-pull configuration
#[derive(Debug)]
pub enum PushPull {}
/// Type-level variant of [`OutputConfig`] for an open drain configuration
#[derive(Debug)]
pub enum OpenDrain {}

/// Type-level variant of [`OutputConfig`] for a readable push-pull configuration
#[derive(Debug)]
pub enum ReadablePushPull {}
/// Type-level variant of [`OutputConfig`] for a readable open-drain configuration
#[derive(Debug)]
pub enum ReadableOpenDrain {}

impl Sealed for PushPull {}
impl Sealed for OpenDrain {}
impl Sealed for ReadableOpenDrain {}
impl Sealed for ReadablePushPull {}
impl ReadableOutput for ReadableOpenDrain {}
impl ReadableOutput for ReadablePushPull {}

impl OutputConfig for PushPull {
    const DYN: DynOutput = DynOutput::PushPull;
}
impl OutputConfig for OpenDrain {
    const DYN: DynOutput = DynOutput::OpenDrain;
}
impl OutputConfig for ReadablePushPull {
    const DYN: DynOutput = DynOutput::ReadablePushPull;
}
impl OutputConfig for ReadableOpenDrain {
    const DYN: DynOutput = DynOutput::ReadableOpenDrain;
}

/// Type-level variant of [`PinMode`] for output modes
///
/// Type `C` is one of four output configurations: [`PushPull`], [`OpenDrain`] or
/// their respective readable versions
#[derive(Debug)]
pub struct Output<C: OutputConfig> {
    cfg: PhantomData<C>,
}

impl<C: OutputConfig> Sealed for Output<C> {}

/// Type-level variant of [`PinMode`] for push-pull output mode
pub type PushPullOutput = Output<PushPull>;
/// Type-level variant of [`PinMode`] for open drain output mode
pub type OutputOpenDrain = Output<OpenDrain>;

pub type OutputReadablePushPull = Output<ReadablePushPull>;
pub type OutputReadableOpenDrain = Output<ReadableOpenDrain>;

//==================================================================================================
//  Alternate configurations
//==================================================================================================

/// Type-level enum for alternate peripheral function configurations
pub trait AlternateConfig: Sealed {
    const DYN: DynAlternate;
}

pub enum Funsel1 {}
pub enum Funsel2 {}
pub enum Funsel3 {}

impl AlternateConfig for Funsel1 {
    const DYN: DynAlternate = DynAlternate::Sel1;
}
impl AlternateConfig for Funsel2 {
    const DYN: DynAlternate = DynAlternate::Sel2;
}
impl AlternateConfig for Funsel3 {
    const DYN: DynAlternate = DynAlternate::Sel3;
}

impl Sealed for Funsel1 {}
impl Sealed for Funsel2 {}
impl Sealed for Funsel3 {}

/// Type-level variant of [`PinMode`] for alternate peripheral functions
///
/// Type `C` is an [`AlternateConfig`]
pub struct Alternate<C: AlternateConfig> {
    cfg: PhantomData<C>,
}

impl<C: AlternateConfig> Sealed for Alternate<C> {}

pub type AltFunc1 = Alternate<Funsel1>;
pub type AltFunc2 = Alternate<Funsel2>;
pub type AltFunc3 = Alternate<Funsel3>;

/// Type alias for the [`PinMode`] at reset
pub type Reset = InputFloating;

//==================================================================================================
//  Pin modes
//==================================================================================================

/// Type-level enum representing pin modes
///
/// The valid options are [Input], [Output] and [Alternate].
pub trait PinMode: Sealed {
    /// Corresponding [DynPinMode]
    const DYN: DynPinMode;
}

impl<C: InputConfig> PinMode for Input<C> {
    const DYN: DynPinMode = DynPinMode::Input(C::DYN);
}
impl<C: OutputConfig> PinMode for Output<C> {
    const DYN: DynPinMode = DynPinMode::Output(C::DYN);
}
impl<C: AlternateConfig> PinMode for Alternate<C> {
    const DYN: DynPinMode = DynPinMode::Alternate(C::DYN);
}

//==================================================================================================
//  Pin IDs
//==================================================================================================

/// Type-level enum for pin IDs
pub trait PinId: Sealed {
    /// Corresponding [DynPinId]
    const DYN: DynPinId;
}

macro_rules! pin_id {
    ($Group:ident, $Id:ident, $NUM:literal) => {
        // Need paste macro to use ident in doc attribute
        paste! {
            #[doc = "Pin ID representing pin " $Id]
            #[derive(Debug)]
            pub enum $Id {}
            impl Sealed for $Id {}
            impl PinId for $Id {
                const DYN: DynPinId = DynPinId::new(Port::$Group, $NUM);
            }
        }
    };
}

//==================================================================================================
//  Pin
//==================================================================================================

/// A type-level GPIO pin, parameterized by [PinId] and [PinMode] types
#[derive(Debug)]
pub struct Pin<I: PinId, M: PinMode> {
    inner: DynPin,
    phantom: PhantomData<(I, M)>,
}

impl<I: PinId, M: PinMode> Pin<I, M> {
    /// Create a new [Pin]
    ///
    /// # Safety
    ///
    /// Each [Pin] must be a singleton. For a given [PinId], there must be
    /// at most one corresponding [Pin] in existence at any given time.
    /// Violating this requirement is `unsafe`.
    #[inline]
    pub(crate) const unsafe fn new() -> Pin<I, M> {
        Pin {
            inner: DynPin::new(I::DYN, M::DYN),
            phantom: PhantomData,
        }
    }

    #[inline]
    pub const fn id(&self) -> DynPinId {
        self.inner.id()
    }

    /// Convert the pin to the requested [`PinMode`]
    #[inline]
    pub fn into_mode<N: PinMode>(mut self) -> Pin<I, N> {
        // Only modify registers if we are actually changing pin mode
        // This check should compile away
        if N::DYN != M::DYN {
            self.inner.change_mode(N::DYN);
        }
        // Safe because we drop the existing Pin
        unsafe { Pin::new() }
    }

    /// Configure the pin for function select 1. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_1(self) -> Pin<I, AltFunc1> {
        self.into_mode()
    }

    /// Configure the pin for function select 2. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_2(self) -> Pin<I, AltFunc2> {
        self.into_mode()
    }

    /// Configure the pin for function select 3. See Programmer Guide p.40 for the function table
    #[inline]
    pub fn into_funsel_3(self) -> Pin<I, AltFunc3> {
        self.into_mode()
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(self) -> Pin<I, InputFloating> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<I, InputPullDown> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<I, InputPullUp> {
        self.into_mode()
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<I, PushPullOutput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a readable push-pull output
    #[inline]
    pub fn into_readable_push_pull_output(self) -> Pin<I, OutputReadablePushPull> {
        self.into_mode()
    }

    /// Configure the pin to operate as a readable open-drain output
    #[inline]
    pub fn into_readable_open_drain_output(self) -> Pin<I, OutputReadableOpenDrain> {
        self.into_mode()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        !self.inner.read_pin()
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        self.inner.read_pin()
    }

    #[inline]
    pub fn datamask(&self) -> bool {
        self.inner.datamask()
    }

    #[inline]
    pub fn clear_datamask(&mut self) {
        self.inner.clear_datamask()
    }

    #[inline]
    pub fn set_datamask(&mut self) {
        self.inner.set_datamask()
    }

    #[inline]
    pub fn is_high_masked(&self) -> Result<bool, crate::gpio::IsMaskedError> {
        self.inner.is_high_masked()
    }

    #[inline]
    pub fn is_low_masked(&self) -> Result<bool, crate::gpio::IsMaskedError> {
        self.inner.is_low_masked()
    }

    #[inline]
    pub fn downgrade(self) -> DynPin {
        self.inner
    }

    // Those only serve for the embedded HAL implementations which have different mutability.

    #[inline]
    fn is_low_mut(&mut self) -> bool {
        self.is_low()
    }

    #[inline]
    fn is_high_mut(&mut self) -> bool {
        self.is_high()
    }

    #[inline]
    pub fn enable_interrupt(&mut self, irq_cfg: crate::InterruptConfig) {
        self.inner.enable_interrupt(irq_cfg);
    }

    #[inline]
    pub fn disable_interrupt(&mut self, reset_irqsel: bool) {
        self.inner.disable_interrupt(reset_irqsel);
    }

    /// Configure the pin for an edge interrupt but does not enable the interrupt.
    pub fn configure_edge_interrupt(&mut self, edge_type: InterruptEdge) {
        self.inner.configure_edge_interrupt(edge_type).unwrap();
    }

    /// Configure the pin for a level interrupt but does not enable the interrupt.
    pub fn configure_level_interrupt(&mut self, level_type: InterruptLevel) {
        self.inner.configure_level_interrupt(level_type).unwrap();
    }
}

//==============================================================================
//  AnyPin
//==============================================================================

/// Type class for [`Pin`] types
///
/// This trait uses the [`AnyKind`] trait pattern to create a [type class] for
/// [`Pin`] types. See the `AnyKind` documentation for more details on the
/// pattern.
///
/// ## `v1` Compatibility
///
/// Normally, this trait would use `Is<Type = SpecificPin<Self>>` as a super
/// trait. But doing so would restrict implementations to only the `v2` `Pin`
/// type in this module. To aid in backwards compatibility, we want to implement
/// `AnyPin` for the `v1` `Pin` type as well. This is possible for a few
/// reasons. First, both structs are zero-sized, so there is no meaningful
/// memory layout to begin with. And even if there were, the `v1` `Pin` type is
/// a newtype wrapper around a `v2` `Pin`, and single-field structs are
/// guaranteed to have the same layout as the field, even for `repr(Rust)`.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
/// [type class]: crate::typelevel#type-classes
pub trait AnyPin
where
    Self: Sealed,
    Self: From<SpecificPin<Self>>,
    Self: Into<SpecificPin<Self>>,
    Self: AsRef<SpecificPin<Self>>,
    Self: AsMut<SpecificPin<Self>>,
{
    /// [`PinId`] of the corresponding [`Pin`]
    type Id: PinId;
    /// [`PinMode`] of the corresponding [`Pin`]
    type Mode: PinMode;
}

impl<I, M> Sealed for Pin<I, M>
where
    I: PinId,
    M: PinMode,
{
}

impl<I, M> AnyPin for Pin<I, M>
where
    I: PinId,
    M: PinMode,
{
    type Id = I;
    type Mode = M;
}

/// Type alias to recover the specific [`Pin`] type from an implementation of
/// [`AnyPin`]
///
/// See the [`AnyKind`] documentation for more details on the pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
pub type SpecificPin<P> = Pin<<P as AnyPin>::Id, <P as AnyPin>::Mode>;

impl<P: AnyPin> AsRef<P> for SpecificPin<P> {
    #[inline]
    fn as_ref(&self) -> &P {
        // SAFETY: This is guaranteed to be safe, because P == SpecificPin<P>
        // Transmuting between `v1` and `v2` `Pin` types is also safe, because
        // both are zero-sized, and single-field, newtype structs are guaranteed
        // to have the same layout as the field anyway, even for repr(Rust).
        unsafe { transmute(self) }
    }
}

impl<P: AnyPin> AsMut<P> for SpecificPin<P> {
    #[inline]
    fn as_mut(&mut self) -> &mut P {
        // SAFETY: This is guaranteed to be safe, because P == SpecificPin<P>
        // Transmuting between `v1` and `v2` `Pin` types is also safe, because
        // both are zero-sized, and single-field, newtype structs are guaranteed
        // to have the same layout as the field anyway, even for repr(Rust).
        unsafe { transmute(self) }
    }
}

//==================================================================================================
//  Additional functionality
//==================================================================================================

impl<I: PinId, C: InputConfig> Pin<I, Input<C>> {
    /// Convert the pin into an async pin. The pin can be converted back by calling
    /// [InputPinAsync::release]
    pub fn into_async_input(self, irq: crate::pac::Interrupt) -> InputPinAsync<I, C> {
        InputPinAsync::new(self, irq)
    }
}

impl<I: PinId, C: OutputConfig> Pin<I, Output<C>> {
    #[inline]
    pub fn set_high(&mut self) {
        self.inner.write_pin(true)
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.inner.write_pin(false)
    }

    #[inline]
    pub fn toggle(&mut self) {
        self.inner.toggle().unwrap()
    }

    #[inline]
    pub fn set_high_masked(&mut self) -> Result<(), crate::gpio::IsMaskedError> {
        self.inner.set_high_masked()
    }

    #[inline]
    pub fn set_low_masked(&mut self) -> Result<(), crate::gpio::IsMaskedError> {
        self.inner.set_low_masked()
    }

    /// See p.53 of the programmers guide for more information.
    /// Possible delays in clock cycles:
    ///  - Delay 1: 1
    ///  - Delay 2: 2
    ///  - Delay 1 + Delay 2: 3
    #[inline]
    pub fn configure_delay(&mut self, delay_1: bool, delay_2: bool) {
        self.inner.configure_delay(delay_1, delay_2).unwrap();
    }

    /// See p.52 of the programmers guide for more information.
    ///
    /// When configured for pulse mode, a given pin will set the non-default state for exactly
    /// one clock cycle before returning to the configured default state
    pub fn configure_pulse_mode(&mut self, enable: bool, default_state: PinState) {
        self.inner
            .configure_pulse_mode(enable, default_state)
            .unwrap();
    }
}

impl<I: PinId, C: InputConfig> Pin<I, Input<C>> {
    /// See p.37 and p.38 of the programmers guide for more information.
    #[inline]
    pub fn configure_filter_type(&mut self, filter: FilterType, clksel: FilterClkSel) {
        self.inner.configure_filter_type(filter, clksel).unwrap();
    }
}

//==================================================================================================
//  Embedded HAL traits
//==================================================================================================

impl<I, M> embedded_hal::digital::ErrorType for Pin<I, M>
where
    I: PinId,
    M: PinMode,
{
    type Error = Infallible;
}

impl<I: PinId, C: OutputConfig> embedded_hal::digital::OutputPin for Pin<I, Output<C>> {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<I, C> embedded_hal::digital::InputPin for Pin<I, Input<C>>
where
    I: PinId,
    C: InputConfig,
{
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_high_mut())
    }
    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_low_mut())
    }
}

impl<I, C> embedded_hal::digital::StatefulOutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig + ReadableOutput,
{
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }
    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }

    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

//==================================================================================================
//  Pin definitions
//==================================================================================================

macro_rules! pins {
    (
        $Port:ident, $PinsName:ident, $($Id:ident,)+,
    ) => {
        paste!(
            /// Collection of all the individual [`Pin`]s for a given port (PORTA or PORTB)
            #[derive(Debug)]
            pub struct $PinsName {
                port: $Port,
                $(
                    #[doc = "Pin " $Id]
                    pub [<$Id:lower>]: Pin<$Id, Reset>,
                )+
            }

            impl $PinsName {
                /// Create a new struct containing all the Pins. Passing the IOCONFIG peripheral
                /// is optional because it might be required to create pin definitions for both
                /// ports.
                #[inline]
                pub fn new(
                    syscfg: &mut va108xx::Sysconfig,
                    port: $Port
                ) -> $PinsName {
                    syscfg.peripheral_clk_enable().modify(|_, w| {
                        w.[<$Port:lower>]().set_bit();
                        w.gpio().set_bit();
                        w.ioconfig().set_bit()
                    });
                    $PinsName {
                        //iocfg,
                        port,
                        // Safe because we only create one `Pin` per `PinId`
                        $(
                            [<$Id:lower>]: unsafe { Pin::new() },
                        )+
                    }
                }

                /// Get the peripheral ID
                /// Safety: Read-only register
                pub fn get_perid() -> u32 {
                    let port = unsafe { &(*$Port::ptr()) };
                    port.perid().read().bits()
                }

                /// Consumes the Pins struct and returns the port definitions
                pub fn release(self) -> $Port {
                    self.port
                }
            }
        );
    }
}

macro_rules! declare_pins {
    (
        $Group:ident, $PinsName:ident, $Port:ident, [$(($Id:ident, $NUM:literal),)+]
    ) => {
        pins!($Port, $PinsName, $($Id,)+,);
        $(
            pin_id!($Group, $Id, $NUM);
        )+
    }
}

declare_pins!(
    A,
    PinsA,
    Porta,
    [
        (PA0, 0),
        (PA1, 1),
        (PA2, 2),
        (PA3, 3),
        (PA4, 4),
        (PA5, 5),
        (PA6, 6),
        (PA7, 7),
        (PA8, 8),
        (PA9, 9),
        (PA10, 10),
        (PA11, 11),
        (PA12, 12),
        (PA13, 13),
        (PA14, 14),
        (PA15, 15),
        (PA16, 16),
        (PA17, 17),
        (PA18, 18),
        (PA19, 19),
        (PA20, 20),
        (PA21, 21),
        (PA22, 22),
        (PA23, 23),
        (PA24, 24),
        (PA25, 25),
        (PA26, 26),
        (PA27, 27),
        (PA28, 28),
        (PA29, 29),
        (PA30, 30),
        (PA31, 31),
    ]
);

declare_pins!(
    B,
    PinsB,
    Portb,
    [
        (PB0, 0),
        (PB1, 1),
        (PB2, 2),
        (PB3, 3),
        (PB4, 4),
        (PB5, 5),
        (PB6, 6),
        (PB7, 7),
        (PB8, 8),
        (PB9, 9),
        (PB10, 10),
        (PB11, 11),
        (PB12, 12),
        (PB13, 13),
        (PB14, 14),
        (PB15, 15),
        (PB16, 16),
        (PB17, 17),
        (PB18, 18),
        (PB19, 19),
        (PB20, 20),
        (PB21, 21),
        (PB22, 22),
        (PB23, 23),
    ]
);
