//! Simple PWM example
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::{delay::DelayNs, pwm::SetDutyCycle};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    pac,
    pins::PinsA,
    prelude::*,
    pwm::{self, get_duty_from_percent, PwmA, PwmB, PwmPin},
    timer::CountdownTimer,
};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx PWM example application--");
    let dp = pac::Peripherals::take().unwrap();
    let pinsa = PinsA::new(dp.porta);
    let mut pwm = pwm::PwmPin::new(50.MHz(), (pinsa.pa3, dp.tim3), 10.Hz()).unwrap();
    let mut delay = CountdownTimer::new(50.MHz(), dp.tim0);
    let mut current_duty_cycle = 0.0;
    pwm.set_duty_cycle(get_duty_from_percent(current_duty_cycle))
        .unwrap();
    pwm.enable();

    // Delete type information, increased code readibility for the rest of the code
    loop {
        let mut counter = 0;
        // Increase duty cycle continuously
        while current_duty_cycle < 1.0 {
            delay.delay_ms(400);
            current_duty_cycle += 0.02;
            counter += 1;
            if counter % 10 == 0 {
                rprintln!("current duty cycle: {}", current_duty_cycle);
            }

            pwm.set_duty_cycle(get_duty_from_percent(current_duty_cycle))
                .unwrap();
        }

        // Switch to PWMB and decrease the window with a high signal from 100 % to 0 %
        // continously
        current_duty_cycle = 0.0;
        let mut upper_limit = 1.0;
        let mut lower_limit = 0.0;
        let mut pwmb: PwmPin<PwmB> = PwmPin::from(pwm);
        pwmb.set_pwmb_lower_limit(get_duty_from_percent(lower_limit));
        pwmb.set_pwmb_upper_limit(get_duty_from_percent(upper_limit));
        while lower_limit < 0.5 {
            delay.delay_ms(400);
            lower_limit += 0.01;
            upper_limit -= 0.01;
            pwmb.set_pwmb_lower_limit(get_duty_from_percent(lower_limit));
            pwmb.set_pwmb_upper_limit(get_duty_from_percent(upper_limit));
            rprintln!("Lower limit: {}", pwmb.pwmb_lower_limit());
            rprintln!("Upper limit: {}", pwmb.pwmb_upper_limit());
        }
        pwm = PwmPin::<PwmA>::from(pwmb);
    }
}
