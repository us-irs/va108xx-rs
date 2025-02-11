//! # API for the REB1 button
//!
//! ## Examples
//!
//! - [Button Blinky with IRQs](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-irq.rs)
//! - [Button Blinky with IRQs and RTIC](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-rtic.rs)
use embedded_hal::digital::InputPin;
use va108xx_hal::{
    gpio::{FilterClkSel, FilterType, InputFloating, InterruptEdge, InterruptLevel, Pin, PA11},
    pac, InterruptConfig,
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
    pub fn configure_edge_interrupt(
        &mut self,
        edge_type: InterruptEdge,
        irq_cfg: InterruptConfig,
        syscfg: Option<&mut pac::Sysconfig>,
        irqsel: Option<&mut pac::Irqsel>,
    ) {
        self.button
            .configure_edge_interrupt(edge_type, irq_cfg, syscfg, irqsel);
    }

    /// Configures an IRQ on level.
    pub fn configure_level_interrupt(
        &mut self,
        level: InterruptLevel,
        irq_cfg: InterruptConfig,
        syscfg: Option<&mut pac::Sysconfig>,
        irqsel: Option<&mut pac::Irqsel>,
    ) {
        self.button
            .configure_level_interrupt(level, irq_cfg, syscfg, irqsel);
    }

    /// Configures a filter on the button. This can be useful for debouncing the switch.
    ///
    /// Please note that you still have to set a clock divisor yourself using the
    /// [`va108xx_hal::clock::set_clk_div_register`] function in order for this to work.
    pub fn configure_filter_type(&mut self, filter: FilterType, clksel: FilterClkSel) {
        self.button.configure_filter_type(filter, clksel);
    }
}
