//! Code to test RTT logger functionality
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal as _;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108XX RTT example --");
    let mut counter = 0;
    loop {
        rprintln!("{}: Hello, world!", counter);
        counter += 1;
        cortex_m::asm::delay(25_000_000);
    }
}
