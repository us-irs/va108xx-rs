//! Blinky button application for the REB1 board
#![no_main]
#![no_std]

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    clock::{set_clk_div_register, FilterClkSel},
    gpio::{FilterType, InterruptEdge, PinsA},
    pac::{self, interrupt},
    prelude::*,
    timer::{default_ms_irq_handler, set_up_ms_tick, InterruptConfig},
};
use vorago_reb1::button::Button;
use vorago_reb1::leds::Leds;

static LEDS: Mutex<RefCell<Option<Leds>>> = Mutex::new(RefCell::new(None));
static BUTTON: Mutex<RefCell<Option<Button>>> = Mutex::new(RefCell::new(None));

#[derive(Debug, PartialEq)]
pub enum PressMode {
    Toggle,
    Keep,
}

// You can change the press mode here
const PRESS_MODE: PressMode = PressMode::Keep;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- Vorago Button IRQ Example --");
    let mut dp = pac::Peripherals::take().unwrap();
    let pinsa = PinsA::new(&mut dp.sysconfig, dp.porta);
    let edge_irq = match PRESS_MODE {
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

    if PRESS_MODE == PressMode::Toggle {
        // This filter debounces the switch for edge based interrupts
        button.configure_filter_type(FilterType::FilterFourClockCycles, FilterClkSel::Clk1);
        set_clk_div_register(&mut dp.sysconfig, FilterClkSel::Clk1, 50_000);
    }

    set_up_ms_tick(
        InterruptConfig::new(pac::Interrupt::OC0, true, true),
        &mut dp.sysconfig,
        Some(&mut dp.irqsel),
        50.MHz(),
        dp.tim0,
    );
    let mut leds = Leds::new(
        pinsa.pa10.into_push_pull_output(),
        pinsa.pa7.into_push_pull_output(),
        pinsa.pa6.into_push_pull_output(),
    );
    for led in leds.iter_mut() {
        led.off();
    }
    // Make both button and LEDs accessible from the IRQ handler as well
    cortex_m::interrupt::free(|cs| {
        LEDS.borrow(cs).replace(Some(leds));
        BUTTON.borrow(cs).replace(Some(button));
    });
    loop {
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn OC0() {
    default_ms_irq_handler();
}

#[interrupt]
fn OC15() {
    cortex_m::interrupt::free(|cs| {
        if PRESS_MODE == PressMode::Toggle {
            if let Some(ref mut leds) = LEDS.borrow(cs).borrow_mut().as_deref_mut() {
                leds[0].toggle();
            }
        } else if let (Some(ref mut leds), Some(ref mut button)) = (
            LEDS.borrow(cs).borrow_mut().as_deref_mut(),
            BUTTON.borrow(cs).borrow_mut().as_mut(),
        ) {
            if button.released() {
                leds[0].off();
            } else {
                leds[0].on();
            }
        }
    });
}
