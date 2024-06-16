//! Blinky examples using the PAC directly, the HAL, or the BSP
//!
//! Additional note on LEDs:
//! Be not afraid: Pulling the GPIOs low makes the LEDs blink. See REB1
//! schematic for more details.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_halt as _;
use va108xx_hal::{gpio::pins::PinsA, pac, prelude::*, timer::set_up_ms_delay_provider};
use vorago_reb1::leds::Leds;

// REB LED pin definitions. All on port A
const LED_D2: u32 = 1 << 10;
const LED_D3: u32 = 1 << 7;
const LED_D4: u32 = 1 << 6;

#[allow(dead_code)]
enum LibType {
    Pac,
    Hal,
    Bsp,
}

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();

    let lib_type = LibType::Bsp;

    match lib_type {
        LibType::Pac => {
            // Enable all peripheral clocks
            dp.sysconfig
                .peripheral_clk_enable()
                .modify(|_, w| unsafe { w.bits(0xffffffff) });
            dp.porta
                .dir()
                .modify(|_, w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
            dp.porta
                .datamask()
                .modify(|_, w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
            for _ in 0..10 {
                dp.porta
                    .clrout()
                    .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
                cortex_m::asm::delay(5_000_000);
                dp.porta
                    .setout()
                    .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
                cortex_m::asm::delay(5_000_000);
            }
            loop {
                dp.porta
                    .togout()
                    .write(|w| unsafe { w.bits(LED_D2 | LED_D3 | LED_D4) });
                cortex_m::asm::delay(25_000_000);
            }
        }
        LibType::Hal => {
            let pins = PinsA::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.porta);
            let mut led1 = pins.pa10.into_readable_push_pull_output();
            let mut led2 = pins.pa7.into_readable_push_pull_output();
            let mut led3 = pins.pa6.into_readable_push_pull_output();
            let mut delay = set_up_ms_delay_provider(&mut dp.sysconfig, 50.MHz(), dp.tim0);
            for _ in 0..10 {
                led1.set_low().ok();
                led2.set_low().ok();
                led3.set_low().ok();
                delay.delay_ms(200);
                led1.set_high().ok();
                led2.set_high().ok();
                led3.set_high().ok();
                delay.delay_ms(200);
            }
            loop {
                led1.toggle().ok();
                delay.delay_ms(200);
                led2.toggle().ok();
                delay.delay_ms(200);
                // Alternatively use deidscted register.
                led3.toggle_with_toggle_reg();
                delay.delay_ms(200);
            }
        }
        LibType::Bsp => {
            let pinsa = PinsA::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.porta);
            let mut leds = Leds::new(
                pinsa.pa10.into_push_pull_output(),
                pinsa.pa7.into_push_pull_output(),
                pinsa.pa6.into_push_pull_output(),
            );
            let mut delay = set_up_ms_delay_provider(&mut dp.sysconfig, 50.MHz(), dp.tim0);
            loop {
                for _ in 0..10 {
                    // Blink all LEDs quickly
                    for led in leds.iter_mut() {
                        led.toggle();
                    }
                    delay.delay_ms(500);
                }
                // Now use a wave pattern
                loop {
                    for led in leds.iter_mut() {
                        led.toggle();
                        delay.delay_ms(200);
                    }
                }
            }
        }
    }
}
