//! Dummy app which does not do anything.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use va108xx_hal as _;
use panic_rtt_target as _;

#[entry]
fn main() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
