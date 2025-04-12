#![no_std]
#![no_main]
use embassy_example as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};

cfg_if::cfg_if! {
    if #[cfg(feature = "custom-irqs")] {
        use va108xx_embassy::embassy_time_driver_irqs;
        use va108xx_hal::pac::interrupt;
        embassy_time_driver_irqs!(timekeeper_irq = OC23, alarm_irq = OC24);
    }
}

use va108xx_hal::{gpio::PinsA, pac, prelude::*};

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

// main is itself an async function.
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    defmt::println!("-- VA108xx Embassy Demo --");

    let mut dp = pac::Peripherals::take().unwrap();

    // Safety: Only called once here.
    cfg_if::cfg_if! {
        if #[cfg(not(feature = "custom-irqs"))] {
            va108xx_embassy::init(
                &mut dp.sysconfig,
                &dp.irqsel,
                SYSCLK_FREQ,
                dp.tim23,
                dp.tim22,
            );
        } else {
            va108xx_embassy::init_with_custom_irqs(
                &mut dp.sysconfig,
                &dp.irqsel,
                SYSCLK_FREQ,
                dp.tim23,
                dp.tim22,
                pac::Interrupt::OC23,
                pac::Interrupt::OC24,
            );
        }
    }

    let porta = PinsA::new(&mut dp.sysconfig, dp.porta);
    let mut led0 = porta.pa10.into_readable_push_pull_output();
    let mut led1 = porta.pa7.into_readable_push_pull_output();
    let mut led2 = porta.pa6.into_readable_push_pull_output();
    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        ticker.next().await;
        defmt::info!("Current time: {}", Instant::now().as_secs());
        led0.toggle();
        led1.toggle();
        led2.toggle();
    }
}
