#![no_std]
#![cfg_attr(docs_rs, feature(doc_auto_cfg))]

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
pub mod utility;

#[derive(Debug, Eq, Copy, Clone, PartialEq)]
pub enum FunSel {
    Sel1 = 0b01,
    Sel2 = 0b10,
    Sel3 = 0b11,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PortSel {
    PortA,
    PortB,
}

#[derive(Copy, Clone, PartialEq, Eq)]
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

/// Generic IRQ config which can be used to specify whether the HAL driver will
/// use the IRQSEL register to route an interrupt, and whether the IRQ will be unmasked in the
/// Cortex-M0 NVIC. Both are generally necessary for IRQs to work, but the user might perform
/// this steps themselves
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct IrqCfg {
    /// Interrupt target vector. Should always be set, might be required for disabling IRQs
    pub irq: pac::Interrupt,
    /// Specfiy whether IRQ should be routed to an IRQ vector using the IRQSEL peripheral
    pub route: bool,
    /// Specify whether the IRQ is unmasked in the Cortex-M NVIC
    pub enable: bool,
}

impl IrqCfg {
    pub fn new(irq: pac::Interrupt, route: bool, enable: bool) -> Self {
        IrqCfg { irq, route, enable }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct InvalidPin(pub(crate) ());

/// Can be used to manually manipulate the function select of port pins
pub fn port_mux(
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
