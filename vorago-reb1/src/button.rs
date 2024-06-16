//! # API for the REB1 button
//!
//! ## Examples
//!
//! - [Button Blinky with IRQs](https://egit.irs.uni-stuttgart.de/rust/vorago-reb1/src/branch/main/examples/blinky-button-irq.rs)
//! - [Button Blinky with IRQs and RTIC](https://egit.irs.uni-stuttgart.de/rust/vorago-reb1/src/branch/main/examples/blinky-button-rtic.rs)
use embedded_hal::digital::InputPin;
use va108xx_hal::{
    gpio::{FilterClkSel, FilterType, InputFloating, InterruptEdge, InterruptLevel, Pin, PA11},
    pac, IrqCfg,
};

pub struct Button {
    button: Pin<PA11, InputFloating>,
}

impl Button {
    pub fn new(pin: Pin<PA11, InputFloating>) -> Button {
        Button { button: pin }
    }

    #[inline]
    pub fn pressed(&mut self) -> bool {
        self.button.is_low().ok().unwrap()
    }

    #[inline]
    pub fn released(&mut self) -> bool {
        self.button.is_high().ok().unwrap()
    }

    /// Configures an IRQ on edge.
    pub fn edge_irq(
        mut self,
        edge_type: InterruptEdge,
        irq_cfg: IrqCfg,
        syscfg: Option<&mut pac::Sysconfig>,
        irqsel: Option<&mut pac::Irqsel>,
    ) -> Self {
        self.button = self
            .button
            .interrupt_edge(edge_type, irq_cfg, syscfg, irqsel);
        self
    }

    /// Configures an IRQ on level.
    pub fn level_irq(
        mut self,
        level: InterruptLevel,
        irq_cfg: IrqCfg,
        syscfg: Option<&mut pac::Sysconfig>,
        irqsel: Option<&mut pac::Irqsel>,
    ) -> Self {
        self.button = self.button.interrupt_level(level, irq_cfg, syscfg, irqsel);
        self
    }

    /// Configures a filter on the button. This can be useful for debouncing the switch.
    ///
    /// Please note that you still have to set a clock divisor yourself using the
    /// [`va108xx_hal::clock::set_clk_div_register`] function in order for this to work.
    pub fn filter_type(mut self, filter: FilterType, clksel: FilterClkSel) -> Self {
        self.button = self.button.filter_type(filter, clksel);
        self
    }
}
