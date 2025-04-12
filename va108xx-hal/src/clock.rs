//! # API for clock related functionality
//!
//! This also includes functionality to enable the peripheral clocks
pub use vorago_shared_periphs::gpio::FilterClkSel;
pub use vorago_shared_periphs::sysconfig::{disable_peripheral_clock, enable_peripheral_clock};

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
