pub use embedded_hal::digital::PinState;

use crate::ioconfig::FilterClkSel;
use crate::ioconfig::FilterType;
#[cfg(feature = "vor1x")]
use crate::{PeripheralSelect, sysconfig::enable_peripheral_clock};

pub use crate::InvalidOffsetError;
pub use crate::Port;
pub use crate::ioconfig::regs::Pull;
use crate::ioconfig::regs::{FunSel, IoConfig, MmioIoConfig};

use super::Pin;
use super::PinIdProvider;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptEdge {
    HighToLow,
    LowToHigh,
    BothEdges,
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptLevel {
    Low = 0,
    High = 1,
}

/// Pin identifier for all physical pins exposed by Vorago MCUs.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PinId {
    port: Port,
    /// Offset within the port.
    offset: u8,
}

impl PinId {
    /// Unchecked constructor which panics on invalid offsets.
    pub const fn new_unchecked(port: Port, offset: usize) -> Self {
        if offset >= port.max_offset() {
            panic!("Pin ID construction: offset is out of range");
        }
        PinId {
            port,
            offset: offset as u8,
        }
    }

    pub const fn new(port: Port, offset: usize) -> Result<Self, InvalidOffsetError> {
        if offset >= port.max_offset() {
            return Err(InvalidOffsetError { offset, port });
        }
        Ok(PinId {
            port,
            offset: offset as u8,
        })
    }

    pub const fn port(&self) -> Port {
        self.port
    }

    pub const fn offset(&self) -> usize {
        self.offset as usize
    }
}

/// Low-level driver structure for GPIO pins.
pub struct LowLevelGpio {
    gpio: super::regs::MmioGpio<'static>,
    ioconfig: MmioIoConfig<'static>,
    id: PinId,
}

impl core::fmt::Debug for LowLevelGpio {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LowLevelGpio")
            .field("gpio", &self.gpio.port())
            .field("id", &self.id)
            .finish()
    }
}

impl LowLevelGpio {
    /// Create a new low-level GPIO pin instance from a given [Pin].
    ///
    /// Can be used for performing resource management of the [Pin]s.
    pub fn new_with_pin<I: PinIdProvider>(_pin: Pin<I>) -> Self {
        Self::new(I::ID)
    }

    /// Create a new low-level GPIO pin instance using only the [PinId].
    pub fn new(id: PinId) -> Self {
        LowLevelGpio {
            gpio: super::regs::Gpio::new_mmio(id.port),
            ioconfig: IoConfig::new_mmio(),
            id,
        }
    }

    #[inline]
    pub fn id(&self) -> PinId {
        self.id
    }

    #[inline]
    pub fn port(&self) -> Port {
        self.id.port()
    }

    #[inline]
    pub fn offset(&self) -> usize {
        self.id.offset()
    }

    pub fn configure_as_input_floating(&mut self) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_funsel(FunSel::Sel0);
            config.set_io_disable(false);
            config.set_invert_input(false);
            config.set_open_drain(false);
            config.set_pull_enable(false);
            config.set_pull_when_output_active(false);
            config.set_invert_output(false);
            config.set_input_enable_when_output(false);
            config
        });
        self.gpio.modify_dir(|mut dir| {
            dir &= !(1 << self.id.offset());
            dir
        });
    }

    pub fn configure_as_input_with_pull(&mut self, pull: Pull) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_funsel(FunSel::Sel0);
            config.set_io_disable(false);
            config.set_invert_input(false);
            config.set_open_drain(false);
            config.set_pull_enable(true);
            config.set_pull_dir(pull);
            config.set_pull_when_output_active(false);
            config.set_invert_output(false);
            config.set_input_enable_when_output(false);
            config
        });
        self.gpio.modify_dir(|mut dir| {
            dir &= !(1 << self.id.offset());
            dir
        });
    }

    pub fn configure_as_output_push_pull(&mut self, init_level: PinState) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_funsel(FunSel::Sel0);
            config.set_io_disable(false);
            config.set_invert_input(false);
            config.set_open_drain(false);
            config.set_pull_enable(false);
            config.set_pull_when_output_active(false);
            config.set_invert_output(false);
            config.set_input_enable_when_output(true);
            config
        });
        match init_level {
            PinState::Low => self.gpio.write_clr_out(self.mask_32()),
            PinState::High => self.gpio.write_set_out(self.mask_32()),
        }
        self.gpio.modify_dir(|mut dir| {
            dir |= 1 << self.id.offset();
            dir
        });
    }

    pub fn configure_as_output_open_drain(&mut self, init_level: PinState) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_funsel(FunSel::Sel0);
            config.set_io_disable(false);
            config.set_invert_input(false);
            config.set_open_drain(true);
            config.set_pull_enable(true);
            config.set_pull_dir(Pull::Up);
            config.set_pull_when_output_active(false);
            config.set_invert_output(false);
            config.set_input_enable_when_output(true);
            config
        });
        let mask32 = self.mask_32();
        match init_level {
            PinState::Low => self.gpio.write_clr_out(mask32),
            PinState::High => self.gpio.write_set_out(mask32),
        }
        self.gpio.modify_dir(|mut dir| {
            dir |= mask32;
            dir
        });
    }

    pub fn configure_as_peripheral_pin(&mut self, fun_sel: FunSel, pull: Option<Pull>) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_funsel(fun_sel);
            config.set_io_disable(false);
            config.set_invert_input(false);
            config.set_open_drain(false);
            config.set_pull_enable(pull.is_some());
            config.set_pull_dir(pull.unwrap_or(Pull::Up));
            config.set_invert_output(false);
            config
        });
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        (self.gpio.read_data_in() >> self.offset()) & 1 == 1
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.gpio.write_set_out(self.mask_32());
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.gpio.write_clr_out(self.mask_32());
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        (self.gpio.read_data_out() >> self.offset()) & 1 == 1
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    #[inline]
    pub fn toggle(&mut self) {
        self.gpio.write_tog_out(self.mask_32());
    }

    #[cfg(feature = "vor1x")]
    pub fn enable_interrupt(&mut self, irq_cfg: crate::InterruptConfig) {
        if irq_cfg.route {
            self.configure_irqsel(irq_cfg.id);
        }
        if irq_cfg.enable_in_nvic {
            unsafe { crate::enable_nvic_interrupt(irq_cfg.id) };
        }
        self.gpio.modify_irq_enb(|mut value| {
            value |= 1 << self.id.offset;
            value
        });
    }

    #[cfg(feature = "vor1x")]
    pub fn disable_interrupt(&mut self, reset_irqsel: bool) {
        if reset_irqsel {
            self.reset_irqsel();
        }
        // We only manipulate our own bit.
        self.gpio.modify_irq_enb(|mut value| {
            value &= !(1 << self.id.offset);
            value
        });
    }

    /// Only useful for interrupt pins. Configure whether to use edges or level as interrupt soure
    /// When using edge mode, it is possible to generate interrupts on both edges as well
    #[inline]
    pub fn configure_edge_interrupt(&mut self, edge_type: InterruptEdge) {
        let mask32 = self.mask_32();
        self.gpio.modify_irq_sen(|mut value| {
            value &= !mask32;
            value
        });
        match edge_type {
            InterruptEdge::HighToLow => {
                self.gpio.modify_irq_evt(|mut value| {
                    value &= !mask32;
                    value
                });
            }
            InterruptEdge::LowToHigh => {
                self.gpio.modify_irq_evt(|mut value| {
                    value |= mask32;
                    value
                });
            }
            InterruptEdge::BothEdges => {
                self.gpio.modify_irq_edge(|mut value| {
                    value |= mask32;
                    value
                });
            }
        }
    }

    /// Configure which edge or level type triggers an interrupt
    #[inline]
    pub fn configure_level_interrupt(&mut self, level: InterruptLevel) {
        let mask32 = self.mask_32();
        self.gpio.modify_irq_sen(|mut value| {
            value |= mask32;
            value
        });
        if level == InterruptLevel::Low {
            self.gpio.modify_irq_evt(|mut value| {
                value &= !mask32;
                value
            });
        } else {
            self.gpio.modify_irq_evt(|mut value| {
                value |= mask32;
                value
            });
        }
    }

    /// Only useful for input pins
    #[inline]
    pub fn configure_filter_type(&mut self, filter: FilterType, clksel: FilterClkSel) {
        self.ioconfig.modify_pin_config(self.id, |mut config| {
            config.set_filter_type(filter);
            config.set_filter_clk_sel(clksel);
            config
        });
    }

    /// Only useful for output pins.
    #[inline]
    pub fn configure_pulse_mode(&mut self, enable: bool, default_state: PinState) {
        self.gpio.modify_pulse(|mut value| {
            if enable {
                value |= 1 << self.id.offset;
            } else {
                value &= !(1 << self.id.offset);
            }
            value
        });
        self.gpio.modify_pulsebase(|mut value| {
            if default_state == PinState::High {
                value |= 1 << self.id.offset;
            } else {
                value &= !(1 << self.id.offset);
            }
            value
        });
    }

    /// Only useful for output pins
    #[inline]
    pub fn configure_delay(&mut self, delay_1: bool, delay_2: bool) {
        self.gpio.modify_delay1(|mut value| {
            if delay_1 {
                value |= 1 << self.id.offset;
            } else {
                value &= !(1 << self.id.offset);
            }
            value
        });
        self.gpio.modify_delay2(|mut value| {
            if delay_2 {
                value |= 1 << self.id.offset;
            } else {
                value &= !(1 << self.id.offset);
            }
            value
        });
    }

    #[cfg(feature = "vor1x")]
    /// Configure the IRQSEL peripheral for this particular pin with the given interrupt ID.
    pub fn configure_irqsel(&mut self, id: va108xx::Interrupt) {
        let irqsel = unsafe { va108xx::Irqsel::steal() };
        enable_peripheral_clock(PeripheralSelect::Irqsel);
        match self.id().port() {
            // Set the correct interrupt number in the IRQSEL register
            super::Port::A => {
                irqsel
                    .porta0(self.id().offset())
                    .write(|w| unsafe { w.bits(id as u32) });
            }
            super::Port::B => {
                irqsel
                    .portb0(self.id().offset())
                    .write(|w| unsafe { w.bits(id as u32) });
            }
        }
    }

    #[cfg(feature = "vor1x")]
    /// Reset the IRQSEL peripheral value for this particular pin.
    pub fn reset_irqsel(&mut self) {
        let irqsel = unsafe { va108xx::Irqsel::steal() };
        enable_peripheral_clock(PeripheralSelect::Irqsel);
        match self.id().port() {
            // Set the correct interrupt number in the IRQSEL register
            super::Port::A => {
                irqsel
                    .porta0(self.id().offset())
                    .write(|w| unsafe { w.bits(u32::MAX) });
            }
            super::Port::B => {
                irqsel
                    .portb0(self.id().offset())
                    .write(|w| unsafe { w.bits(u32::MAX) });
            }
        }
    }

    #[inline(always)]
    pub const fn mask_32(&self) -> u32 {
        1 << self.id.offset()
    }
}
