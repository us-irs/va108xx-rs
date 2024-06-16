//! Empty RTIC project template
#![no_main]
#![no_std]

use defmt_testapp as _;

#[rtic::app(device = pac)]
mod app {
    use va108xx_hal::pac;

    #[local]
    struct Local {}

    #[shared]
    struct Shared {}

    #[init]
    fn init(_ctx: init::Context) -> (Shared, Local) {
        defmt::println!("-- Vorago RTIC template --");
        (Shared {}, Local {})
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        #[allow(clippy::empty_loop)]
        loop {}
    }
}
