#![no_std]
#![no_main]
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use embedded_hal::digital::StatefulOutputPin;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{gpio::PinsA, pac, prelude::*};

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

// main is itself an async function.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    rtt_init_print!();
    rprintln!("-- VA108xx Embassy Demo --");

    let mut dp = pac::Peripherals::take().unwrap();

    // Safety: Only called once here.
    unsafe {
        embassy_example::init(
            &mut dp.sysconfig,
            &dp.irqsel,
            SYSCLK_FREQ,
            dp.tim23,
            dp.tim22,
        )
    };

    let porta = PinsA::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.porta);
    let mut led0 = porta.pa10.into_readable_push_pull_output();
    let mut led1 = porta.pa7.into_readable_push_pull_output();
    let mut led2 = porta.pa6.into_readable_push_pull_output();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        rprintln!("Current time: {}", Instant::now().as_secs());
        led0.toggle().ok();
        led1.toggle().ok();
        led2.toggle().ok();
    }
}
