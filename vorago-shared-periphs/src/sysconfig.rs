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
