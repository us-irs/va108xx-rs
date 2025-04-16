//! # API for clock related functionality
//!
//! This also includes functionality to enable the peripheral clocks
use crate::time::Hertz;
use cortex_m::interrupt::{self, Mutex};
use once_cell::unsync::OnceCell;

pub use vorago_shared_periphs::gpio::FilterClkSel;
pub use vorago_shared_periphs::sysconfig::{disable_peripheral_clock, enable_peripheral_clock};

static SYS_CLOCK: Mutex<OnceCell<Hertz>> = Mutex::new(OnceCell::new());

/// The Vorago in powered by an external clock which might have different frequencies.
/// The clock can be set here so it can be used by other software components as well.
/// The clock can be set exactly once
pub fn set_sys_clock(freq: impl Into<Hertz>) {
    interrupt::free(|cs| {
        SYS_CLOCK.borrow(cs).set(freq.into()).ok();
    })
}

/// Returns the configured system clock
pub fn get_sys_clock() -> Option<Hertz> {
    interrupt::free(|cs| SYS_CLOCK.borrow(cs).get().copied())
}

pub fn set_clk_div_register(syscfg: &mut va108xx::Sysconfig, clk_sel: FilterClkSel, div: u32) {
    match clk_sel {
        FilterClkSel::SysClk => (),
        FilterClkSel::Clk1 => {
            syscfg.ioconfig_clkdiv1().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk2 => {
            syscfg.ioconfig_clkdiv2().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk3 => {
            syscfg.ioconfig_clkdiv3().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk4 => {
            syscfg.ioconfig_clkdiv4().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk5 => {
            syscfg.ioconfig_clkdiv5().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk6 => {
            syscfg.ioconfig_clkdiv6().write(|w| unsafe { w.bits(div) });
        }
        FilterClkSel::Clk7 => {
            syscfg.ioconfig_clkdiv7().write(|w| unsafe { w.bits(div) });
        }
    }
}
