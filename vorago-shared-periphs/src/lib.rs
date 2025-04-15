#![no_std]
pub mod gpio;
pub mod ioconfig;
pub mod sysconfig;

#[cfg(not(feature = "_family-selected"))]
compile_error!("no Vorago CPU family was select. Choices: vor1x or vor4x");

pub use ioconfig::regs::FunSel;

#[cfg(feature = "vor1x")]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PeripheralSelect {
    PortA = 0,
    PortB = 1,
    Spi0 = 4,
    Spi1 = 5,
    Spi2 = 6,
    Uart0 = 8,
    Uart1 = 9,
    I2c0 = 16,
    I2c1 = 17,
    Irqsel = 21,
    Ioconfig = 22,
    Utility = 23,
    Gpio = 24,
}

cfg_if::cfg_if! {
    if #[cfg(feature = "vor1x")] {
        /// Number of GPIO ports and IOCONFIG registers for PORT A
        pub const NUM_PORT_A: usize = 32;
        /// Number of GPIO ports and IOCONFIG registers for PORT B
        pub const NUM_PORT_B: usize = 24;
    } else if #[cfg(feature = "vor4x")] {
        /// Number of GPIO ports and IOCONFIG registers for PORT C to Port F
        pub const NUM_PORT_DEFAULT: usize = 16;
        /// Number of GPIO ports and IOCONFIG registers for PORT A
        pub const NUM_PORT_A: usize = NUM_PORT_DEFAULT;
        /// Number of GPIO ports and IOCONFIG registers for PORT B
        pub const NUM_PORT_B: usize = NUM_PORT_DEFAULT;
        /// Number of GPIO ports and IOCONFIG registers for PORT G
        pub const NUM_PORT_G: usize = 8;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Port {
    A = 0,
    B = 1,
    #[cfg(feature = "vor4x")]
    C = 2,
    #[cfg(feature = "vor4x")]
    D = 3,
    #[cfg(feature = "vor4x")]
    E = 4,
    #[cfg(feature = "vor4x")]
    F = 5,
    #[cfg(feature = "vor4x")]
    G = 6,
}

impl Port {
    pub const fn max_offset(&self) -> usize {
        match self {
            Port::A => NUM_PORT_A,
            Port::B => NUM_PORT_B,
            #[cfg(feature = "vor4x")]
            Port::C | Port::D | Port::E | Port::F => NUM_PORT_DEFAULT,
            #[cfg(feature = "vor4x")]
            Port::G => NUM_PORT_G,
        }
    }

    /// Unsafely steal the GPIO peripheral block for the given port.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees by the HAL.
    pub unsafe fn steal_gpio(&self) -> gpio::regs::MmioGpio<'static> {
        gpio::regs::Gpio::new_mmio(*self)
    }
}

#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[error("invalid GPIO offset {offset} for port {port:?}")]
pub struct InvalidOffsetError {
    offset: usize,
    port: Port,
}

/// Generic interrupt config which can be used to specify whether the HAL driver will
/// use the IRQSEL register to route an interrupt, and whether the IRQ will be unmasked in the
/// Cortex-M0 NVIC. Both are generally necessary for IRQs to work, but the user might want to
/// perform those steps themselves.
#[cfg(feature = "vor1x")]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Interrupt target vector. Should always be set, might be required for disabling IRQs
    pub id: va108xx::Interrupt,
    /// Specfiy whether IRQ should be routed to an IRQ vector using the IRQSEL peripheral.
    pub route: bool,
    /// Specify whether the IRQ is unmasked in the Cortex-M NVIC. If an interrupt is used for
    /// multiple purposes, the user can enable the interrupts themselves.
    pub enable_in_nvic: bool,
}

#[cfg(feature = "vor1x")]
impl InterruptConfig {
    pub fn new(id: va108xx::Interrupt, route: bool, enable_in_nvic: bool) -> Self {
        InterruptConfig {
            id,
            route,
            enable_in_nvic,
        }
    }
}

/// Enable a specific interrupt using the NVIC peripheral.
///
/// # Safety
///
/// This function is `unsafe` because it can break mask-based critical sections.
#[cfg(feature = "vor1x")]
#[inline]
pub unsafe fn enable_nvic_interrupt(irq: va108xx::Interrupt) {
    unsafe {
        cortex_m::peripheral::NVIC::unmask(irq);
    }
}

/// Disable a specific interrupt using the NVIC peripheral.
#[cfg(feature = "vor1x")]
#[inline]
pub fn disable_nvic_interrupt(irq: va108xx::Interrupt) {
    cortex_m::peripheral::NVIC::mask(irq);
}

#[allow(dead_code)]
pub(crate) mod sealed {
    pub trait Sealed {}
}
