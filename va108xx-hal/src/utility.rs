//! # API for utility functions like the Error Detection and Correction (EDAC) block
//!
//! Some more information about the recommended scrub rates can be found on the
//! [Vorago White Paper website](https://www.voragotech.com/resources) in the
//! application note AN1212
use crate::pac;

/// Unmask and enable an IRQ with the given interrupt number
///
/// ## Safety
///
/// The unmask function can break mask-based critical sections
#[inline]
pub(crate) fn unmask_irq(irq: pac::Interrupt) {
    unsafe { cortex_m::peripheral::NVIC::unmask(irq) };
}
