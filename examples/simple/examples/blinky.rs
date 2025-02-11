//! Simple blinky example
//!
//! Additional note on LEDs when using the REB1 development board:
//! Be not afraid: Pulling the GPIOs low makes the LEDs blink. See REB1
//! schematic for more details.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::{
    delay::DelayNs,
    digital::{OutputPin, StatefulOutputPin},
};
use panic_halt as _;
use va108xx_hal::{
    gpio::PinsA,
    pac::{self, interrupt},
    prelude::*,
    timer::DelayMs,
    timer::{default_ms_irq_handler, set_up_ms_tick, CountdownTimer},
    InterruptConfig,
};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut delay_ms = DelayMs::new(set_up_ms_tick(
        InterruptConfig::new(interrupt::OC0, true, true),
        &mut dp.sysconfig,
        Some(&mut dp.irqsel),
        50.MHz(),
        dp.tim0,
    ))
    .unwrap();
    let mut delay_tim1 = CountdownTimer::new(&mut dp.sysconfig, 50.MHz(), dp.tim1);
    let porta = PinsA::new(&mut dp.sysconfig, dp.porta);
    let mut led1 = porta.pa10.into_readable_push_pull_output();
    let mut led2 = porta.pa7.into_readable_push_pull_output();
    let mut led3 = porta.pa6.into_readable_push_pull_output();
    for _ in 0..10 {
        led1.set_low().ok();
        led2.set_low().ok();
        led3.set_low().ok();
        delay_ms.delay_ms(200);
        led1.set_high().ok();
        led2.set_high().ok();
        led3.set_high().ok();
        delay_tim1.delay_ms(200);
    }
    loop {
        led1.toggle().ok();
        delay_ms.delay_ms(200);
        led2.toggle().ok();
        delay_tim1.delay_ms(200);
        led3.toggle().ok();
        delay_ms.delay_ms(200);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC0() {
    default_ms_irq_handler()
}
