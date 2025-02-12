//! Blinky button application for the REB1 board using RTIC
#![no_main]
#![no_std]

#[rtic::app(device = pac)]
mod app {
    use panic_rtt_target as _;
    use rtic_example::SYSCLK_FREQ;
    use rtt_target::{rprintln, rtt_init_default, set_print_channel};
    use va108xx_hal::{
        clock::{set_clk_div_register, FilterClkSel},
        gpio::{FilterType, InterruptEdge, PinsA},
        pac,
        prelude::*,
        timer::{default_ms_irq_handler, set_up_ms_tick, InterruptConfig},
    };
    use vorago_reb1::button::Button;
    use vorago_reb1::leds::Leds;

    rtic_monotonics::systick_monotonic!(Mono, 1_000);

    #[derive(Debug, PartialEq)]
    pub enum PressMode {
        Toggle,
        Keep,
    }

    #[derive(Debug, PartialEq)]
    pub enum CfgMode {
        Prompt,
        Fixed,
    }

    const CFG_MODE: CfgMode = CfgMode::Fixed;
    // You can change the press mode here
    const DEFAULT_MODE: PressMode = PressMode::Toggle;

    #[local]
    struct Local {
        leds: Leds,
        button: Button,
        mode: PressMode,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init_default!();
        set_print_channel(channels.up.0);
        rprintln!("-- Vorago Button IRQ Example --");
        Mono::start(cx.core.SYST, SYSCLK_FREQ.raw());

        let mode = match CFG_MODE {
            // Ask mode from user via RTT
            CfgMode::Prompt => prompt_mode(channels.down.0),
            // Use mode hardcoded in `DEFAULT_MODE`
            CfgMode::Fixed => DEFAULT_MODE,
        };
        rprintln!("Using {:?} mode", mode);

        let mut dp = cx.device;
        let pinsa = PinsA::new(&mut dp.sysconfig, dp.porta);
        let edge_irq = match mode {
            PressMode::Toggle => InterruptEdge::HighToLow,
            PressMode::Keep => InterruptEdge::BothEdges,
        };

        // Configure an edge interrupt on the button and route it to interrupt vector 15
        let mut button = Button::new(pinsa.pa11.into_floating_input());
        button.configure_edge_interrupt(
            edge_irq,
            InterruptConfig::new(pac::interrupt::OC15, true, true),
            Some(&mut dp.sysconfig),
            Some(&mut dp.irqsel),
        );

        if mode == PressMode::Toggle {
            // This filter debounces the switch for edge based interrupts
            button.configure_filter_type(FilterType::FilterFourClockCycles, FilterClkSel::Clk1);
            set_clk_div_register(&mut dp.sysconfig, FilterClkSel::Clk1, 50_000);
        }
        let mut leds = Leds::new(
            pinsa.pa10.into_push_pull_output(),
            pinsa.pa7.into_push_pull_output(),
            pinsa.pa6.into_push_pull_output(),
        );
        for led in leds.iter_mut() {
            led.off();
        }
        set_up_ms_tick(
            InterruptConfig::new(pac::Interrupt::OC0, true, true),
            &mut dp.sysconfig,
            Some(&mut dp.irqsel),
            50.MHz(),
            dp.tim0,
        );
        (Shared {}, Local { leds, button, mode })
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = OC15, local=[button, leds, mode])]
    fn button_task(cx: button_task::Context) {
        let leds = cx.local.leds;
        let button = cx.local.button;
        let mode = cx.local.mode;
        if *mode == PressMode::Toggle {
            leds[0].toggle();
        } else if button.released() {
            leds[0].off();
        } else {
            leds[0].on();
        }
    }

    #[task(binds = OC0)]
    fn ms_tick(_cx: ms_tick::Context) {
        default_ms_irq_handler();
    }

    fn prompt_mode(mut down_channel: rtt_target::DownChannel) -> PressMode {
        rprintln!("Using prompt mode");
        rprintln!("Please enter the mode [0: Toggle, 1: Keep]");
        let mut read_buf: [u8; 16] = [0; 16];
        let mut read;
        loop {
            read = down_channel.read(&mut read_buf);
            for &byte in &read_buf[..read] {
                match byte as char {
                    '0' => return PressMode::Toggle,
                    '1' => return PressMode::Keep,
                    _ => continue, // Ignore other characters
                }
            }
        }
    }
}
