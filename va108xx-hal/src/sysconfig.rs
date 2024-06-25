use crate::{pac, PeripheralSelect};

#[derive(PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidCounterResetVal(pub(crate) ());

/// Enable scrubbing for the ROM
///
/// Returns [InvalidCounterResetVal] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_rom_scrubbing(
    syscfg: &mut pac::Sysconfig,
    scrub_rate: u32,
) -> Result<(), InvalidCounterResetVal> {
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(InvalidCounterResetVal(()));
    }
    syscfg.rom_scrub().write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_rom_scrubbing(syscfg: &mut pac::Sysconfig) {
    syscfg.rom_scrub().write(|w| unsafe { w.bits(0) })
}

/// Enable scrubbing for the RAM
///
/// Returns [InvalidCounterResetVal] if the scrub rate is 0
/// (equivalent to disabling) or larger than 24 bits
pub fn enable_ram_scrubbing(
    syscfg: &mut pac::Sysconfig,
    scrub_rate: u32,
) -> Result<(), InvalidCounterResetVal> {
    if scrub_rate == 0 || scrub_rate > u32::pow(2, 24) {
        return Err(InvalidCounterResetVal(()));
    }
    syscfg.ram_scrub().write(|w| unsafe { w.bits(scrub_rate) });
    Ok(())
}

pub fn disable_ram_scrubbing(syscfg: &mut pac::Sysconfig) {
    syscfg.ram_scrub().write(|w| unsafe { w.bits(0) })
}

/// Clear the reset bit. This register is active low, so doing this will hold the peripheral
/// in a reset state
pub fn clear_reset_bit(syscfg: &mut pac::Sysconfig, periph_sel: PeripheralSelect) {
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << periph_sel as u8)) });
}

pub fn set_reset_bit(syscfg: &mut pac::Sysconfig, periph_sel: PeripheralSelect) {
    syscfg
        .peripheral_reset()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << periph_sel as u8)) });
}
