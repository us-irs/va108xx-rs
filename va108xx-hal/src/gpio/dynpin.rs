//! # Type-erased, value-level module for GPIO pins
//!
//! Although the type-level API is generally preferred, it is not suitable in
//! all cases. Because each pin is represented by a distinct type, it is not
//! possible to store multiple pins in a homogeneous data structure. The
//! value-level API solves this problem by erasing the type information and
//! tracking the pin at run-time.
//!
//! Value-level pins are represented by the [`DynPin`] type. [`DynPin`] has two
//! fields, `id` and `mode` with types [`DynPinId`] and [`DynPinMode`]
//! respectively. The implementation of these types closely mirrors the
//! type-level API.
//!
//! Instances of [`DynPin`] cannot be created directly. Rather, they must be
//! created from their type-level equivalents using [`From`]/[`Into`].
//!
//! ```
//! // Move a pin out of the Pins struct and convert to a DynPin
//! let pa0: DynPin = pins.pa0.into();
//! ```
//!
//! Conversions between pin modes use a value-level version of the type-level
//! API.
//!
//! ```
//! // Use one of the literal function names
//! pa0.into_floating_input();
//! // Use a method and a DynPinMode variant
//! pa0.into_mode(DYN_FLOATING_INPUT);
//! ```
//!
//! Because the pin state cannot be tracked at compile-time, many [`DynPin`]
//! operations become fallible. Run-time checks are inserted to ensure that
//! users don't try to, for example, set the output level of an input pin.
//!
//! Users may try to convert value-level pins back to their type-level
//! equivalents. However, this option is fallible, because the compiler cannot
//! guarantee the pin has the correct ID or is in the correct mode at
//! compile-time. Use [TryFrom]/[TryInto] for this conversion.
//!
//! ```
//! // Convert to a `DynPin`
//! let pa0: DynPin = pins.pa0.into();
//! // Change pin mode
//! pa0.into_floating_input();
//! // Convert back to a `Pin`
//! let pa0: Pin<PA0, FloatingInput> = pa0.try_into().unwrap();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for [`DynPin`].
//! However, whereas the type-level API uses
//! `Error = core::convert::Infallible`, the value-level API can return a real
//! error. If the [`DynPin`] is not in the correct [`DynPinMode`] for the
//! operation, the trait functions will return
//! [InvalidPinTypeError].

use super::{
    pin::{FilterType, Pin, PinId, PinMode},
    InputDynPinAsync, InterruptEdge, InterruptLevel, IsMaskedError, PinState, Port,
};
use crate::{clock::FilterClkSel, enable_nvic_interrupt, pac, FunSel};

//==================================================================================================
//  DynPinMode configurations
//==================================================================================================

/// Value-level `enum` for disabled configurations
#[derive(PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DynDisabled {
    Floating,
    PullDown,
    PullUp,
}

/// Value-level `enum` for input configurations
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DynInput {
    Floating,
    PullDown,
    PullUp,
}

/// Value-level `enum` for output configurations
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DynOutput {
    PushPull,
    OpenDrain,
    ReadablePushPull,
    ReadableOpenDrain,
}

pub type DynAlternate = FunSel;

//==============================================================================
//  Error
//==============================================================================

/// GPIO error type
///
/// [`DynPin`]s are not tracked and verified at compile-time, so run-time
/// operations are fallible. This `enum` represents the corresponding errors.
#[derive(Debug, PartialEq, Eq, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[error("Invalid pin type for operation: {0:?}")]
pub struct InvalidPinTypeError(pub DynPinMode);

impl embedded_hal::digital::Error for InvalidPinTypeError {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
        embedded_hal::digital::ErrorKind::Other
    }
}

//==================================================================================================
//  DynPinMode
//==================================================================================================

/// Value-level `enum` representing pin modes
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DynPinMode {
    Input(DynInput),
    Output(DynOutput),
    Alternate(DynAlternate),
}

/// Value-level variant of [`DynPinMode`] for floating input mode
pub const DYN_FLOATING_INPUT: DynPinMode = DynPinMode::Input(DynInput::Floating);
/// Value-level variant of [`DynPinMode`] for pull-down input mode
pub const DYN_PULL_DOWN_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullDown);
/// Value-level variant of [`DynPinMode`] for pull-up input mode
pub const DYN_PULL_UP_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullUp);

/// Value-level variant of [`DynPinMode`] for push-pull output mode
pub const DYN_PUSH_PULL_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::PushPull);
/// Value-level variant of [`DynPinMode`] for open-drain output mode
pub const DYN_OPEN_DRAIN_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::OpenDrain);
/// Value-level variant of [`DynPinMode`] for readable push-pull output mode
pub const DYN_RD_PUSH_PULL_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::ReadablePushPull);
/// Value-level variant of [`DynPinMode`] for readable opendrain output mode
pub const DYN_RD_OPEN_DRAIN_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::ReadableOpenDrain);

/// Value-level variant of [`DynPinMode`] for function select 1
pub const DYN_ALT_FUNC_1: DynPinMode = DynPinMode::Alternate(DynAlternate::Sel1);
/// Value-level variant of [`DynPinMode`] for function select 2
pub const DYN_ALT_FUNC_2: DynPinMode = DynPinMode::Alternate(DynAlternate::Sel2);
/// Value-level variant of [`DynPinMode`] for function select 3
pub const DYN_ALT_FUNC_3: DynPinMode = DynPinMode::Alternate(DynAlternate::Sel3);

//==================================================================================================
//  DynGroup & DynPinId
//==================================================================================================

pub type DynGroup = Port;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DynPinId {
    port: Port,
    num: u8,
}

impl DynPinId {
    pub const fn new(port: Port, num: u8) -> Self {
        DynPinId { port, num }
    }

    pub const fn port(&self) -> Port {
        self.port
    }
    pub const fn num(&self) -> u8 {
        self.num
    }
}

//==================================================================================================
//  ModeFields
//==================================================================================================

/// Collect all fields needed to set the [`PinMode`](super::PinMode)
#[derive(Default)]
struct ModeFields {
    dir: bool,
    opendrn: bool,
    pull_en: bool,
    /// true for pullup, false for pulldown
    pull_dir: bool,
    funsel: u8,
    enb_input: bool,
}

impl From<DynPinMode> for ModeFields {
    #[inline]
    fn from(mode: DynPinMode) -> Self {
        let mut fields = Self::default();
        match mode {
            DynPinMode::Input(config) => {
                fields.dir = false;
                fields.funsel = FunSel::Sel0 as u8;
                match config {
                    DynInput::Floating => (),
                    DynInput::PullUp => {
                        fields.pull_en = true;
                        fields.pull_dir = true;
                    }
                    DynInput::PullDown => {
                        fields.pull_en = true;
                    }
                }
            }
            DynPinMode::Output(config) => {
                fields.dir = true;
                fields.funsel = FunSel::Sel0 as u8;
                match config {
                    DynOutput::PushPull => (),
                    DynOutput::OpenDrain => {
                        fields.opendrn = true;
                    }
                    DynOutput::ReadableOpenDrain => {
                        fields.enb_input = true;
                        fields.opendrn = true;
                    }
                    DynOutput::ReadablePushPull => {
                        fields.enb_input = true;
                    }
                }
            }
            DynPinMode::Alternate(config) => {
                fields.funsel = config as u8;
            }
        }
        fields
    }
}

/// Type definition to avoid confusion: These register blocks are identical
type PortRegisterBlock = pac::porta::RegisterBlock;
pub type PortReg = pac::ioconfig::Porta;

//==================================================================================================
//  DynPin
//==================================================================================================

/// A value-level pin, parameterized by [`DynPinId`] and [`DynPinMode`]
///
/// This type acts as a type-erased version of [`Pin`]. Every pin is represented
/// by the same type, and pins are tracked and distinguished at run-time.
#[derive(Debug)]
pub struct DynPin {
    id: DynPinId,
    mode: DynPinMode,
}

impl DynPin {
    /// Create a new [DynPin]
    ///
    /// # Safety
    ///
    /// Each [DynPin] must be a singleton. For a given [DynPinId], there
    /// must be at most one corresponding [`DynPin`] in existence at any given
    /// time.  Violating this requirement is `unsafe`.
    #[inline]
    pub(crate) const unsafe fn new(id: DynPinId, mode: DynPinMode) -> Self {
        DynPin { id, mode }
    }

    /// Steals a new [DynPin].
    ///
    /// This function will simply set the internal mode to [DYN_FLOATING_INPUT] pin without
    /// modifying any registers related to the behaviour of the pin. The user should call
    /// [Self::into_mode] to ensure the correct mode of the pin.
    ///
    /// # Safety
    ///
    /// Circumvents the HAL's safety guarantees. The caller must ensure that the pin is not
    /// used cocurrently somewhere else. The caller might also want to call [Self::into_mode]
    /// to ensure the correct desired state of the pin. It is recommended to create the pin using
    /// [Pin::downgrade] instead.
    pub const unsafe fn steal(id: DynPinId) -> Self {
        DynPin {
            id,
            mode: DYN_FLOATING_INPUT,
        }
    }

    /// Return a copy of the pin ID
    #[inline]
    pub const fn id(&self) -> DynPinId {
        self.id
    }

    /// Return a copy of the pin mode
    #[inline]
    pub const fn mode(&self) -> DynPinMode {
        self.mode
    }

    /// Convert the pin to the requested [`DynPinMode`]
    #[inline]
    pub fn into_mode(&mut self, mode: DynPinMode) {
        self.change_mode(mode);
        self.mode = mode;
    }

    #[inline]
    pub fn is_input_pin(&self) -> bool {
        matches!(self.mode, DynPinMode::Input(_))
    }

    #[inline]
    pub fn is_output_pin(&self) -> bool {
        matches!(self.mode, DynPinMode::Output(_))
    }

    #[inline]
    pub fn into_funsel_1(&mut self) {
        self.into_mode(DYN_ALT_FUNC_1);
    }

    #[inline]
    pub fn into_funsel_2(&mut self) {
        self.into_mode(DYN_ALT_FUNC_2);
    }

    #[inline]
    pub fn into_funsel_3(&mut self) {
        self.into_mode(DYN_ALT_FUNC_3);
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(&mut self) {
        self.into_mode(DYN_FLOATING_INPUT);
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(&mut self) {
        self.into_mode(DYN_PULL_DOWN_INPUT);
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(&mut self) {
        self.into_mode(DYN_PULL_UP_INPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_push_pull_output(&mut self) {
        self.into_mode(DYN_PUSH_PULL_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_open_drain_output(&mut self) {
        self.into_mode(DYN_OPEN_DRAIN_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_readable_push_pull_output(&mut self) {
        self.into_mode(DYN_RD_PUSH_PULL_OUTPUT);
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_readable_open_drain_output(&mut self) {
        self.into_mode(DYN_RD_OPEN_DRAIN_OUTPUT);
    }

    #[inline(always)]
    pub fn is_low(&self) -> Result<bool, InvalidPinTypeError> {
        self.read_internal().map(|v| !v)
    }

    #[inline(always)]
    pub fn is_high(&self) -> Result<bool, InvalidPinTypeError> {
        self.read_internal()
    }

    #[inline(always)]
    pub fn set_low(&mut self) -> Result<(), InvalidPinTypeError> {
        self.write_internal(false)
    }

    #[inline(always)]
    pub fn set_high(&mut self) -> Result<(), InvalidPinTypeError> {
        self.write_internal(true)
    }

    /// Toggle the logic level of an output pin
    #[inline(always)]
    pub fn toggle(&mut self) -> Result<(), InvalidPinTypeError> {
        if !self.is_output_pin() {
            return Err(InvalidPinTypeError(self.mode));
        }
        // Safety: TOGOUT is a "mask" register, and we only write the bit for
        // this pin ID
        unsafe { self.port_reg().togout().write(|w| w.bits(self.mask_32())) };
        Ok(())
    }

    pub fn enable_interrupt(&mut self, irq_cfg: crate::InterruptConfig) {
        if irq_cfg.route {
            self.configure_irqsel(irq_cfg.id);
        }
        if irq_cfg.enable_in_nvic {
            unsafe { enable_nvic_interrupt(irq_cfg.id) };
        }

        // We only manipulate our own bit.
        self.port_reg()
            .irq_enb()
            .modify(|r, w| unsafe { w.bits(r.bits() | self.mask_32()) });
    }

    pub fn disable_interrupt(&mut self, reset_irqsel: bool) {
        if reset_irqsel {
            self.reset_irqsel();
        }
        // We only manipulate our own bit.
        self.port_reg()
            .irq_enb()
            .modify(|r, w| unsafe { w.bits(r.bits() & !self.mask_32()) });
    }

    /// Try to recreate a type-level [`Pin`] from a value-level [`DynPin`]
    ///
    /// There is no way for the compiler to know if the conversion will be
    /// successful at compile-time. We must verify the conversion at run-time
    /// or refuse to perform it.
    #[inline]
    pub fn upgrade<I: PinId, M: PinMode>(self) -> Result<Pin<I, M>, InvalidPinTypeError> {
        if self.id == I::DYN && self.mode == M::DYN {
            // The `DynPin` is consumed, so it is safe to replace it with the
            // corresponding `Pin`
            return Ok(unsafe { Pin::new() });
        }
        Err(InvalidPinTypeError(self.mode))
    }

    /// Convert the pin into an async pin. The pin can be converted back by calling
    /// [InputDynPinAsync::release]
    pub fn into_async_input(
        self,
        irq: crate::pac::Interrupt,
    ) -> Result<InputDynPinAsync, InvalidPinTypeError> {
        InputDynPinAsync::new(self, irq)
    }

    /// Configure the IRQSEL peripheral for this particular pin with the given interrupt ID.
    pub fn configure_irqsel(&mut self, id: pac::Interrupt) {
        let mut syscfg = unsafe { pac::Sysconfig::steal() };
        let irqsel = unsafe { pac::Irqsel::steal() };
        crate::clock::enable_peripheral_clock(&mut syscfg, crate::clock::PeripheralClocks::Irqsel);
        match self.id().port() {
            // Set the correct interrupt number in the IRQSEL register
            super::Port::A => {
                irqsel
                    .porta0(self.id().num() as usize)
                    .write(|w| unsafe { w.bits(id as u32) });
            }
            super::Port::B => {
                irqsel
                    .portb0(self.id().num as usize)
                    .write(|w| unsafe { w.bits(id as u32) });
            }
        }
    }

    /// Reset the IRQSEL peripheral value for this particular pin.
    pub fn reset_irqsel(&mut self) {
        let mut syscfg = unsafe { pac::Sysconfig::steal() };
        let irqsel = unsafe { pac::Irqsel::steal() };
        crate::clock::enable_peripheral_clock(&mut syscfg, crate::clock::PeripheralClocks::Irqsel);
        match self.id().port() {
            // Set the correct interrupt number in the IRQSEL register
            super::Port::A => {
                irqsel
                    .porta0(self.id().num() as usize)
                    .write(|w| unsafe { w.bits(u32::MAX) });
            }
            super::Port::B => {
                irqsel
                    .portb0(self.id().num as usize)
                    .write(|w| unsafe { w.bits(u32::MAX) });
            }
        }
    }

    // Get DATAMASK bit for this particular pin
    #[inline(always)]
    pub fn datamask(&self) -> bool {
        (self.port_reg().datamask().read().bits() >> self.id().num) == 1
    }

    /// Clear DATAMASK bit for this particular pin. This prevents access
    /// of the corresponding bit for output and input operations
    #[inline(always)]
    pub fn clear_datamask(&self) {
        self.port_reg()
            .datamask()
            .modify(|r, w| unsafe { w.bits(r.bits() & !self.mask_32()) });
    }

    /// Set DATAMASK bit for this particular pin. 1 is the default
    /// state of the bit and allows access of the corresponding bit
    #[inline(always)]
    pub fn set_datamask(&self) {
        self.port_reg()
            .datamask()
            .modify(|r, w| unsafe { w.bits(r.bits() | self.mask_32()) });
    }

    #[inline]
    pub fn is_high_masked(&self) -> Result<bool, crate::gpio::IsMaskedError> {
        self.read_pin_masked()
    }

    #[inline]
    pub fn is_low_masked(&self) -> Result<bool, crate::gpio::IsMaskedError> {
        self.read_pin_masked().map(|v| !v)
    }

    #[inline]
    pub fn set_high_masked(&mut self) -> Result<(), crate::gpio::IsMaskedError> {
        self.write_pin_masked(true)
    }

    #[inline]
    pub fn set_low_masked(&mut self) -> Result<(), crate::gpio::IsMaskedError> {
        self.write_pin_masked(false)
    }

    /// Possible delays in clock cycles:
    ///  - Delay 1: 1
    ///  - Delay 2: 2
    ///  - Delay 1 + Delay 2: 3
    #[inline]
    pub fn configure_delay(
        &mut self,
        delay_1: bool,
        delay_2: bool,
    ) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.configure_delay_internal(delay_1, delay_2);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    /// When configured for pulse mode, a given pin will set the non-default state for exactly
    /// one clock cycle before returning to the configured default state
    #[inline]
    pub fn configure_pulse_mode(
        &mut self,
        enable: bool,
        default_state: PinState,
    ) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.configure_pulse_mode_internal(enable, default_state);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    /// See p.37 and p.38 of the programmers guide for more information.
    #[inline]
    pub fn configure_filter_type(
        &mut self,
        filter: FilterType,
        clksel: FilterClkSel,
    ) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Input(_) => {
                self.configure_filter_type_internal(filter, clksel);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    pub fn configure_edge_interrupt(
        &mut self,
        edge_type: InterruptEdge,
    ) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Input(_) | DynPinMode::Output(_) => {
                self.configure_edge_interrupt_internal(edge_type);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    pub fn configure_level_interrupt(
        &mut self,
        level_type: InterruptLevel,
    ) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Input(_) | DynPinMode::Output(_) => {
                self.configure_level_interrupt_internal(level_type);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    /// Change the pin mode
    #[inline]
    pub(crate) fn change_mode(&mut self, mode: DynPinMode) {
        let ModeFields {
            dir,
            funsel,
            opendrn,
            pull_dir,
            pull_en,
            enb_input,
        } = mode.into();
        let (portreg, iocfg) = (self.port_reg(), self.iocfg_port());
        iocfg.write(|w| {
            w.opendrn().bit(opendrn);
            w.pen().bit(pull_en);
            w.plevel().bit(pull_dir);
            w.iewo().bit(enb_input);
            unsafe { w.funsel().bits(funsel) }
        });
        let mask = self.mask_32();
        unsafe {
            if dir {
                portreg.dir().modify(|r, w| w.bits(r.bits() | mask));
                // Clear output
                portreg.clrout().write(|w| w.bits(mask));
            } else {
                portreg.dir().modify(|r, w| w.bits(r.bits() & !mask));
            }
        }
    }

    #[inline]
    const fn port_reg(&self) -> &PortRegisterBlock {
        match self.id().port() {
            Port::A => unsafe { &(*pac::Porta::ptr()) },
            Port::B => unsafe { &(*pac::Portb::ptr()) },
        }
    }

    #[inline]
    const fn iocfg_port(&self) -> &PortReg {
        let ioconfig = unsafe { va108xx::Ioconfig::ptr().as_ref().unwrap() };
        match self.id().port() {
            Port::A => ioconfig.porta(self.id().num() as usize),
            Port::B => ioconfig.portb0(self.id().num() as usize),
        }
    }

    #[inline(always)]
    fn read_internal(&self) -> Result<bool, InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Input(_) | DYN_RD_OPEN_DRAIN_OUTPUT | DYN_RD_PUSH_PULL_OUTPUT => {
                Ok(self.read_pin())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    #[inline(always)]
    fn write_internal(&mut self, bit: bool) -> Result<(), InvalidPinTypeError> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.write_pin(bit);
                Ok(())
            }
            _ => Err(InvalidPinTypeError(self.mode)),
        }
    }

    #[inline]
    /// Read the logic level of an output pin
    pub(crate) fn read_pin(&self) -> bool {
        let portreg = self.port_reg();
        ((portreg.datainraw().read().bits() >> self.id().num) & 0x01) == 1
    }

    /// Read a pin but use the masked version but check whether the datamask for the pin is
    /// cleared as well
    #[inline(always)]
    fn read_pin_masked(&self) -> Result<bool, IsMaskedError> {
        if !self.datamask() {
            Err(IsMaskedError)
        } else {
            Ok(((self.port_reg().datain().read().bits() >> self.id().num) & 0x01) == 1)
        }
    }

    /// Write the logic level of an output pin
    #[inline(always)]
    pub(crate) fn write_pin(&mut self, bit: bool) {
        // Safety: SETOUT is a "mask" register, and we only write the bit for
        // this pin ID
        unsafe {
            if bit {
                self.port_reg().setout().write(|w| w.bits(self.mask_32()));
            } else {
                self.port_reg().clrout().write(|w| w.bits(self.mask_32()));
            }
        }
    }

    /// Write the logic level of an output pin but check whether the datamask for the pin is
    /// cleared as well
    #[inline]
    fn write_pin_masked(&mut self, bit: bool) -> Result<(), IsMaskedError> {
        if !self.datamask() {
            Err(IsMaskedError)
        } else {
            // Safety: SETOUT is a "mask" register, and we only write the bit for
            // this pin ID
            unsafe {
                if bit {
                    self.port_reg().setout().write(|w| w.bits(self.mask_32()));
                } else {
                    self.port_reg().clrout().write(|w| w.bits(self.mask_32()));
                }
                Ok(())
            }
        }
    }

    /// Toggle the logic level of an output pin
    #[inline(always)]
    pub fn toggle_with_togout_reg(&mut self) {
        // Safety: TOGOUT is a "mask" register, and we only write the bit for
        // this pin ID
        unsafe { self.port_reg().togout().write(|w| w.bits(self.mask_32())) };
    }

    /// Only useful for interrupt pins. Configure whether to use edges or level as interrupt soure
    /// When using edge mode, it is possible to generate interrupts on both edges as well
    #[inline]
    fn configure_edge_interrupt_internal(&mut self, edge_type: InterruptEdge) {
        unsafe {
            self.port_reg()
                .irq_sen()
                .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            match edge_type {
                InterruptEdge::HighToLow => {
                    self.port_reg()
                        .irq_evt()
                        .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
                }
                InterruptEdge::LowToHigh => {
                    self.port_reg()
                        .irq_evt()
                        .modify(|r, w| w.bits(r.bits() | self.mask_32()));
                }
                InterruptEdge::BothEdges => {
                    self.port_reg()
                        .irq_edge()
                        .modify(|r, w| w.bits(r.bits() | self.mask_32()));
                }
            }
        }
    }

    /// Configure which edge or level type triggers an interrupt
    #[inline]
    fn configure_level_interrupt_internal(&mut self, level: InterruptLevel) {
        unsafe {
            self.port_reg()
                .irq_sen()
                .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            if level == InterruptLevel::Low {
                self.port_reg()
                    .irq_evt()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            } else {
                self.port_reg()
                    .irq_evt()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            }
        }
    }

    /// Only useful for input pins
    #[inline]
    fn configure_filter_type_internal(&mut self, filter: FilterType, clksel: FilterClkSel) {
        self.iocfg_port().modify(|_, w| {
            // Safety: Only write to register for this Pin ID
            unsafe {
                w.flttype().bits(filter as u8);
                w.fltclk().bits(clksel as u8)
            }
        });
    }

    #[inline]
    fn configure_pulse_mode_internal(&mut self, enable: bool, default_state: PinState) {
        let portreg = self.port_reg();
        unsafe {
            if enable {
                portreg
                    .pulse()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .pulse()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
            if default_state == PinState::Low {
                portreg
                    .pulsebase()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            } else {
                portreg
                    .pulsebase()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            }
        }
    }

    /// Only useful for output pins
    #[inline]
    fn configure_delay_internal(&mut self, delay_1: bool, delay_2: bool) {
        let portreg = self.port_reg();
        unsafe {
            if delay_1 {
                portreg
                    .delay1()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .delay1()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
            if delay_2 {
                portreg
                    .delay2()
                    .modify(|r, w| w.bits(r.bits() | self.mask_32()));
            } else {
                portreg
                    .delay2()
                    .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
            }
        }
    }

    // Only serves disambiguation purposes for the Embedded HAL impl
    #[inline(always)]
    fn is_low_mut(&mut self) -> Result<bool, InvalidPinTypeError> {
        self.is_low()
    }

    // Only serves disambiguation purposes for the Embedded HAL impl
    #[inline(always)]
    fn is_high_mut(&mut self) -> Result<bool, InvalidPinTypeError> {
        self.is_high()
    }

    #[inline(always)]
    const fn mask_32(&self) -> u32 {
        1 << self.id().num()
    }
}

//==================================================================================================
//  Convert between Pin and DynPin
//==================================================================================================

impl<I: PinId, M: PinMode> From<Pin<I, M>> for DynPin {
    /// Erase the type-level information in a [`Pin`] and return a value-level
    /// [`DynPin`]
    #[inline]
    fn from(pin: Pin<I, M>) -> Self {
        pin.downgrade()
    }
}

impl<I: PinId, M: PinMode> TryFrom<DynPin> for Pin<I, M> {
    type Error = InvalidPinTypeError;

    /// Try to recreate a type-level [`Pin`] from a value-level [`DynPin`]
    ///
    /// There is no way for the compiler to know if the conversion will be
    /// successful at compile-time. We must verify the conversion at run-time
    /// or refuse to perform it.
    #[inline]
    fn try_from(pin: DynPin) -> Result<Self, Self::Error> {
        pin.upgrade()
    }
}

//==================================================================================================
// Embedded HAL traits
//==================================================================================================

impl embedded_hal::digital::ErrorType for DynPin {
    type Error = InvalidPinTypeError;
}

impl embedded_hal::digital::OutputPin for DynPin {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high()
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low()
    }
}

impl embedded_hal::digital::InputPin for DynPin {
    #[inline]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_high_mut()
    }
    #[inline]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_low_mut()
    }
}

impl embedded_hal::digital::StatefulOutputPin for DynPin {
    #[inline]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.is_high_mut()
    }

    #[inline]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.is_low_mut()
    }

    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle()
    }
}
