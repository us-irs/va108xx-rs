#![no_std]
#![cfg_attr(docsrs, feature(doc_auto_cfg))]

pub use va108xx;
pub use va108xx as pac;

pub mod clock;
pub mod gpio;
pub mod i2c;
pub mod prelude;
pub mod pwm;
pub mod spi;
pub mod sysconfig;
pub mod time;
pub mod timer;
pub mod typelevel;
pub mod uart;

#[derive(Debug, Eq, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FunSel {
    Sel1 = 0b01,
    Sel2 = 0b10,
    Sel3 = 0b11,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortSel {
    PortA,
    PortB,
}

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

/// Generic interrupt config which can be used to specify whether the HAL driver will
/// use the IRQSEL register to route an interrupt, and whether the IRQ will be unmasked in the
/// Cortex-M0 NVIC. Both are generally necessary for IRQs to work, but the user might want to
/// perform those steps themselves.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// Interrupt target vector. Should always be set, might be required for disabling IRQs
    pub id: pac::Interrupt,
    /// Specfiy whether IRQ should be routed to an IRQ vector using the IRQSEL peripheral.
    pub route: bool,
    /// Specify whether the IRQ is unmasked in the Cortex-M NVIC. If an interrupt is used for
    /// multiple purposes, the user can enable the interrupts themselves.
    pub enable_in_nvic: bool,
}

impl InterruptConfig {
    pub fn new(id: pac::Interrupt, route: bool, enable_in_nvic: bool) -> Self {
        InterruptConfig {
            id,
            route,
            enable_in_nvic,
        }
    }
}

pub type IrqCfg = InterruptConfig;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidPin(pub(crate) ());

/// Can be used to manually manipulate the function select of port pins
pub fn port_function_select(
    ioconfig: &mut pac::Ioconfig,
    port: PortSel,
    pin: u8,
    funsel: FunSel,
) -> Result<(), InvalidPin> {
    match port {
        PortSel::PortA => {
            if pin > 31 {
                return Err(InvalidPin(()));
            }
            ioconfig
                .porta(pin as usize)
                .modify(|_, w| unsafe { w.funsel().bits(funsel as u8) });
            Ok(())
        }
        PortSel::PortB => {
            if pin > 23 {
                return Err(InvalidPin(()));
            }
            ioconfig
                .portb0(pin as usize)
                .modify(|_, w| unsafe { w.funsel().bits(funsel as u8) });
            Ok(())
        }
    }
}

/// Enable a specific interrupt using the NVIC peripheral.
///
/// # Safety
///
/// This function is `unsafe` because it can break mask-based critical sections.
#[inline]
pub unsafe fn enable_nvic_interrupt(irq: pac::Interrupt) {
    unsafe {
        cortex_m::peripheral::NVIC::unmask(irq);
    }
}

/// Disable a specific interrupt using the NVIC peripheral.
#[inline]
pub fn disable_nvic_interrupt(irq: pac::Interrupt) {
    cortex_m::peripheral::NVIC::mask(irq);
}
