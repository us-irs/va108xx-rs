//! Test image
//!
//! It would be nice to use a test framework like defmt-test, but I have issues
//! with probe run and it would be better to make the RTT work first
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::{PinState, PinsA, PinsB},
    pac::{self, interrupt},
    prelude::*,
    time::Hertz,
    timer::{default_ms_irq_handler, set_up_ms_tick, CountdownTimer, InterruptConfig},
};

#[allow(dead_code)]
#[derive(Debug)]
enum TestCase {
    // Tie PORTA[0] to PORTA[1] for these tests!
    TestBasic,
    TestPullup,
    TestPulldown,
    TestMask,
    // Tie PORTB[22] to PORTB[23] for this test
    PortB,
    Perid,
    // Tie PA0 to an oscilloscope and configure pulse detection
    Pulse,
    // Tie PA0, PA1 and PA3 to an oscilloscope
    DelayGpio,
    DelayMs,
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx Test Application --");
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    let pinsa = PinsA::new(&mut dp.sysconfig, dp.porta);
    let pinsb = PinsB::new(&mut dp.sysconfig, dp.portb);
    let mut led1 = pinsa.pa10.into_readable_push_pull_output();
    let test_case = TestCase::DelayMs;

    match test_case {
        TestCase::TestBasic
        | TestCase::TestPulldown
        | TestCase::TestPullup
        | TestCase::TestMask => {
            rprintln!(
                "Test case {:?}. Make sure to tie PORTA[0] to PORTA[1]",
                test_case
            );
        }
        _ => {
            rprintln!("Test case {:?}", test_case);
        }
    }
    match test_case {
        TestCase::TestBasic => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let mut out = pinsa.pa0.into_readable_push_pull_output();
            let input = pinsa.pa1.into_floating_input();
            out.set_high();
            assert!(input.is_high());
            out.set_low();
            assert!(input.is_low());
        }
        TestCase::TestPullup => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = pinsa.pa1.into_pull_up_input();
            assert!(input.is_high());
            let mut out = pinsa.pa0.into_readable_push_pull_output();
            out.set_low();
            assert!(input.is_low());
            out.set_high();
            assert!(input.is_high());
            out.into_floating_input();
            assert!(input.is_high());
        }
        TestCase::TestPulldown => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let input = pinsa.pa1.into_pull_down_input();
            assert!(input.is_low());
            let mut out = pinsa.pa0.into_push_pull_output();
            out.set_low();
            assert!(input.is_low());
            out.set_high();
            assert!(input.is_high());
            out.into_floating_input();
            assert!(input.is_low());
        }
        TestCase::TestMask => {
            // Tie PORTA[0] to PORTA[1] for these tests!
            let mut input = pinsa.pa1.into_pull_down_input();
            input.clear_datamask();
            assert!(!input.datamask());
            let mut out = pinsa.pa0.into_push_pull_output();
            out.clear_datamask();
            assert!(input.is_low_masked().is_err());
            assert!(out.set_high_masked().is_err());
        }
        TestCase::PortB => {
            // Tie PORTB[22] to PORTB[23] for these tests!
            let mut out = pinsb.pb22.into_readable_push_pull_output();
            let input = pinsb.pb23.into_floating_input();
            out.set_high();
            assert!(input.is_high());
            out.set_low();
            assert!(input.is_low());
        }
        TestCase::Perid => {
            assert_eq!(PinsA::get_perid(), 0x004007e1);
            assert_eq!(PinsB::get_perid(), 0x004007e1);
        }
        TestCase::Pulse => {
            let mut output_pulsed = pinsa.pa0.into_push_pull_output();
            output_pulsed.configure_pulse_mode(true, PinState::Low);
            rprintln!("Pulsing high 10 times..");
            output_pulsed.set_low();
            for _ in 0..10 {
                output_pulsed.set_high();
                cortex_m::asm::delay(25_000_000);
            }
            output_pulsed.configure_pulse_mode(true, PinState::High);
            rprintln!("Pulsing low 10 times..");
            for _ in 0..10 {
                output_pulsed.set_low();
                cortex_m::asm::delay(25_000_000);
            }
        }
        TestCase::DelayGpio => {
            let mut out_0 = pinsa.pa0.into_readable_push_pull_output();
            out_0.configure_delay(true, false);
            let mut out_1 = pinsa.pa1.into_readable_push_pull_output();
            out_1.configure_delay(false, true);
            let mut out_2 = pinsa.pa3.into_readable_push_pull_output();
            out_2.configure_delay(true, true);
            for _ in 0..20 {
                out_0.toggle();
                out_1.toggle();
                out_2.toggle();
                cortex_m::asm::delay(25_000_000);
            }
        }
        TestCase::DelayMs => {
            let mut ms_timer = set_up_ms_tick(
                InterruptConfig::new(pac::Interrupt::OC0, true, true),
                &mut dp.sysconfig,
                Some(&mut dp.irqsel),
                50.MHz(),
                dp.tim0,
            );
            for _ in 0..5 {
                led1.toggle();
                ms_timer.delay_ms(500);
                led1.toggle();
                ms_timer.delay_ms(500);
            }

            let mut delay_timer = CountdownTimer::new(&mut dp.sysconfig, 50.MHz(), dp.tim1);
            let mut pa0 = pinsa.pa0.into_readable_push_pull_output();
            for _ in 0..5 {
                led1.toggle();
                delay_timer.delay_ms(500);
                led1.toggle();
                delay_timer.delay_ms(500);
            }
            let ahb_freq: Hertz = 50.MHz();
            let mut syst_delay = cortex_m::delay::Delay::new(cp.SYST, ahb_freq.raw());
            // Test usecond delay using both TIM peripheral and SYST. Use the release image if you
            // want to verify the timings!
            loop {
                pa0.toggle();
                delay_timer.delay_us(50);
                pa0.toggle();
                delay_timer.delay_us(50);
                pa0.toggle();
                syst_delay.delay_us(50);
                pa0.toggle();
                syst_delay.delay_us(50);
            }
        }
    }

    rprintln!("Test success");
    loop {
        led1.toggle();
        cortex_m::asm::delay(25_000_000);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC0() {
    default_ms_irq_handler()
}
