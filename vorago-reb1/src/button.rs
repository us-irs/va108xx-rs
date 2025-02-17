//! # API for the REB1 button
//!
//! ## Examples
//!
//! - [Button Blinky with IRQs](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-irq.rs)
//! - [Button Blinky with IRQs and RTIC](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-rtic.rs)
use va108xx_hal::{
    gpio::{FilterClkSel, FilterType, InputFloating, InterruptEdge, InterruptLevel, Pin, PA11},
    InterruptConfig,
};

#[derive(Debug)]
pub struct Button(pub Pin<PA11, InputFloating>);

impl Button {
    pub fn new(pin: Pin<PA11, InputFloating>) -> Button {
        Button(pin)
    }

    #[inline]
    pub fn pressed(&mut self) -> bool {
        self.0.is_low()
    }

    #[inline]
    pub fn released(&mut self) -> bool {
        self.0.is_high()
    }

    /// Configures an IRQ on edge.
    pub fn configure_and_enable_edge_interrupt(
        &mut self,
        edge_type: InterruptEdge,
        irq_cfg: InterruptConfig,
    ) {
        self.0.configure_edge_interrupt(edge_type);
        self.0.enable_interrupt(irq_cfg);
    }

    /// Configures an IRQ on level.
    pub fn configure_and_enable_level_interrupt(
        &mut self,
        level: InterruptLevel,
        irq_cfg: InterruptConfig,
    ) {
        self.0.configure_level_interrupt(level);
        self.0.enable_interrupt(irq_cfg);
    }

    /// Configures a filter on the button. This can be useful for debouncing the switch.
    ///
    /// Please note that you still have to set a clock divisor yourself using the
    /// [`va108xx_hal::clock::set_clk_div_register`] function in order for this to work.
    pub fn configure_filter_type(&mut self, filter: FilterType, clksel: FilterClkSel) {
        self.0.configure_filter_type(filter, clksel);
    }
}
