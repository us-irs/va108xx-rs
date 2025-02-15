//! RTIC minimal blinky
#![no_main]
#![no_std]

#[rtic::app(device = pac, dispatchers = [OC31, OC30, OC29])]
mod app {
    use cortex_m::asm;
    use panic_rtt_target as _;
    use rtic_example::SYSCLK_FREQ;
    use rtic_monotonics::systick::prelude::*;
    use rtic_monotonics::Monotonic;
    use rtt_target::{rprintln, rtt_init_print};
    use va108xx_hal::{
        gpio::{OutputReadablePushPull, Pin, PinsA, PA10, PA6, PA7},
        pac,
    };

    #[local]
    struct Local {
        led0: Pin<PA10, OutputReadablePushPull>,
        led1: Pin<PA7, OutputReadablePushPull>,
        led2: Pin<PA6, OutputReadablePushPull>,
    }

    #[shared]
    struct Shared {}

    rtic_monotonics::systick_monotonic!(Mono, 1_000);

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("-- Vorago VA108xx RTIC template --");

        Mono::start(cx.core.SYST, SYSCLK_FREQ.raw());

        let porta = PinsA::new(&mut cx.device.sysconfig, cx.device.porta);
        let led0 = porta.pa10.into_readable_push_pull_output();
        let led1 = porta.pa7.into_readable_push_pull_output();
        let led2 = porta.pa6.into_readable_push_pull_output();
        blinky::spawn().ok();
        (Shared {}, Local { led0, led1, led2 })
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    #[task(
        priority = 3,
        local=[led0, led1, led2],
    )]
    async fn blinky(cx: blinky::Context) {
        loop {
            rprintln!("toggling LEDs");
            cx.local.led0.toggle();
            cx.local.led1.toggle();
            cx.local.led2.toggle();
            Mono::delay(1000.millis()).await;
        }
    }
}
