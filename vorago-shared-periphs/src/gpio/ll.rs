pub use embedded_hal::digital::PinState;

pub use crate::InvalidOffsetError;
pub use crate::Port;
pub use crate::ioconfig::regs::Pull;
use crate::ioconfig::regs::{FunSel, IoConfig, MmioIoConfig};

pub struct LowLevelGpio {
    gpio: super::regs::MmioGpio<'static>,
    ioconfig: MmioIoConfig<'static>,
    port: Port,
    offset: usize,
}

impl LowLevelGpio {
    pub fn new(port: Port, offset: usize) -> Result<Self, InvalidOffsetError> {
        if offset >= port.max_offset() {
            return Err(InvalidOffsetError {
                offset,
                port: Port::A,
            });
        }
        Ok(LowLevelGpio {
            gpio: super::regs::Gpio::new_mmio(port),
            ioconfig: IoConfig::new_mmio(),
            port,
            offset,
        })
    }

    #[inline]
    pub fn port(&self) -> Port {
        self.port
    }

    #[inline]
    pub fn offset(&self) -> usize {
        self.offset
    }

    pub fn configure_as_input_floating(&mut self) {
        unsafe {
            self.ioconfig
                .modify_pin_config_unchecked(self.port, self.offset, |mut config| {
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
        }
        self.gpio.modify_dir(|mut dir| {
            dir &= !(1 << self.offset);
            dir
        });
    }

    pub fn configure_as_input_with_pull(&mut self, pull: Pull) {
        unsafe {
            self.ioconfig
                .modify_pin_config_unchecked(self.port, self.offset, |mut config| {
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
        }
        self.gpio.modify_dir(|mut dir| {
            dir &= !(1 << self.offset);
            dir
        });
    }

    pub fn configure_as_output_push_pull(&mut self, init_level: PinState) {
        unsafe {
            self.ioconfig
                .modify_pin_config_unchecked(self.port, self.offset, |mut config| {
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
        }
        match init_level {
            PinState::Low => self.gpio.write_clr_out(1 << self.offset),
            PinState::High => self.gpio.write_set_out(1 << self.offset),
        }
        self.gpio.modify_dir(|mut dir| {
            dir |= 1 << self.offset;
            dir
        });
    }

    pub fn configure_as_output_open_drain(&mut self, init_level: PinState) {
        unsafe {
            self.ioconfig
                .modify_pin_config_unchecked(self.port, self.offset, |mut config| {
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
        }
        match init_level {
            PinState::Low => self.gpio.write_clr_out(1 << self.offset),
            PinState::High => self.gpio.write_set_out(1 << self.offset),
        }
        self.gpio.modify_dir(|mut dir| {
            dir |= 1 << self.offset;
            dir
        });
    }

    pub fn configure_as_peripheral_pin(&mut self, fun_sel: FunSel, pull: Option<Pull>) {
        unsafe {
            self.ioconfig
                .modify_pin_config_unchecked(self.port, self.offset, |mut config| {
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
    }

    #[inline]
    pub fn is_high(&self) -> bool {
        (self.gpio.read_data_in() >> self.offset) & 1 == 1
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    #[inline]
    pub fn set_high(&mut self) {
        self.gpio.write_set_out(1 << self.offset);
    }

    #[inline]
    pub fn set_low(&mut self) {
        self.gpio.write_clr_out(1 << self.offset);
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        (self.gpio.read_data_out() >> self.offset) & 1 == 1
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        !self.is_set_high()
    }

    #[inline]
    pub fn toggle(&mut self) {
        self.gpio.write_tog_out(1 << self.offset);
    }
}
