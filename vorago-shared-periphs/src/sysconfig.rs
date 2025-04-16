#[cfg(feature = "vor1x")]
#[inline]
pub fn enable_peripheral_clock(clock: crate::PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << clock as u8)) });
}

#[cfg(feature = "vor1x")]
#[inline]
pub fn disable_peripheral_clock(clock: crate::PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << clock as u8)) });
}

#[cfg(feature = "vor1x")]
#[inline]
pub fn assert_peripheral_reset(periph_sel: crate::PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << periph_sel as u8)) });
}

#[cfg(feature = "vor1x")]
#[inline]
pub fn deassert_peripheral_reset(periph_sel: crate::PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << periph_sel as u8)) });
}

#[cfg(feature = "vor1x")]
#[inline]
pub fn reset_peripheral_for_cycles(periph_sel: crate::PeripheralSelect, cycles: u32) {
    assert_peripheral_reset(periph_sel);
    cortex_m::asm::delay(cycles);
    deassert_peripheral_reset(periph_sel);
}
