use crate::PeripheralSelect;

#[derive(PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidCounterResetVal(pub(crate) ());

/// Enable scrubbing for the ROM
///
/// Returns [InvalidCounterResetVal] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_rom_scrubbing(scrub_rate: u32) -> Result<(), InvalidCounterResetVal> {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(InvalidCounterResetVal(()));
    }
    syscfg.rom_scrub().write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_rom_scrubbing() {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg.rom_scrub().write(|w| unsafe { w.bits(0) });
}

/// Enable scrubbing for the RAM
///
/// Returns [InvalidCounterResetVal] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_ram_scrubbing(scrub_rate: u32) -> Result<(), InvalidCounterResetVal> {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(InvalidCounterResetVal(()));
    }
    syscfg.ram_scrub().write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_ram_scrubbing() {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg.ram_scrub().write(|w| unsafe { w.bits(0) });
}

#[inline]
pub fn assert_peripheral_reset(periph_sel: PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << periph_sel as u8)) });
}

#[inline]
pub fn deassert_peripheral_reset(periph_sel: PeripheralSelect) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << periph_sel as u8)) });
}

pub fn reset_peripheral_for_cycles(periph_sel: PeripheralSelect, cycles: u32) {
    assert_peripheral_reset(periph_sel);
    cortex_m::asm::delay(cycles);
    deassert_peripheral_reset(periph_sel);
}
