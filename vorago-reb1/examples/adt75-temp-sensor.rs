#![no_main]
#![no_std]
use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{pac, prelude::*, timer::set_up_ms_delay_provider};
use vorago_reb1::temp_sensor::Adt75TempSensor;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- Vorago Temperature Sensor and I2C Example --");
    let mut dp = pac::Peripherals::take().unwrap();
    let mut delay = set_up_ms_delay_provider(&mut dp.sysconfig, 50.MHz(), dp.tim0);
    let mut temp_sensor = Adt75TempSensor::new(dp.i2ca, 50.MHz(), Some(&mut dp.sysconfig))
        .expect("Creating temperature sensor struct failed");
    loop {
        let temp = temp_sensor
            .read_temperature()
            .expect("Failed reading temperature");
        rprintln!("Temperature in Celcius: {}", temp);
        delay.delay_ms(500);
    }
}
