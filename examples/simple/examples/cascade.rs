//! Simple Cascade example
//!
//! A timer will be periodically started which starts another timer via the cascade feature.
//! This timer will then start another timer with the cascade feature as well.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    pac::{self, interrupt},
    prelude::*,
    timer::{
        default_ms_irq_handler, set_up_ms_delay_provider, CascadeCtrl, CascadeSource,
        CountdownTimer, Event, InterruptConfig,
    },
};

static CSD_TGT_1: Mutex<RefCell<Option<CountdownTimer<pac::Tim4>>>> =
    Mutex::new(RefCell::new(None));
static CSD_TGT_2: Mutex<RefCell<Option<CountdownTimer<pac::Tim5>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx Cascade example application--");

    let mut dp = pac::Peripherals::take().unwrap();
    let mut delay = set_up_ms_delay_provider(&mut dp.sysconfig, 50.MHz(), dp.tim0);

    // Will be started periodically to trigger a cascade
    let mut cascade_triggerer =
        CountdownTimer::new(&mut dp.sysconfig, 50.MHz(), dp.tim3).auto_disable(true);
    cascade_triggerer.listen(
        Event::TimeOut,
        InterruptConfig::new(pac::Interrupt::OC1, true, false),
        Some(&mut dp.irqsel),
        Some(&mut dp.sysconfig),
    );

    // First target for cascade
    let mut cascade_target_1 =
        CountdownTimer::new(&mut dp.sysconfig, 50.MHz(), dp.tim4).auto_deactivate(true);
    cascade_target_1
        .cascade_0_source(CascadeSource::Tim(3))
        .expect("Configuring cascade source for TIM4 failed");
    let mut csd_cfg = CascadeCtrl {
        enb_start_src_csd0: true,
        ..Default::default()
    };
    // Use trigger mode here
    csd_cfg.trg_csd0 = true;
    cascade_target_1.cascade_control(csd_cfg);
    // Normally it should already be sufficient to activate IRQ in the CTRL
    // register but a full interrupt is use here to display print output when
    // the timer expires
    cascade_target_1.listen(
        Event::TimeOut,
        InterruptConfig::new(pac::Interrupt::OC2, true, false),
        Some(&mut dp.irqsel),
        Some(&mut dp.sysconfig),
    );
    // The counter will only activate when the cascade signal is coming in so
    // it is okay to call start here to set the reset value
    cascade_target_1.start(1.Hz());

    // Activated by first cascade target
    let mut cascade_target_2 =
        CountdownTimer::new(&mut dp.sysconfig, 50.MHz(), dp.tim5).auto_deactivate(true);
    // Set TIM4 as cascade source
    cascade_target_2
        .cascade_1_source(CascadeSource::Tim(4))
        .expect("Configuring cascade source for TIM5 failed");

    csd_cfg = CascadeCtrl::default();
    csd_cfg.enb_start_src_csd1 = true;
    // Use trigger mode here
    csd_cfg.trg_csd1 = true;
    cascade_target_2.cascade_control(csd_cfg);
    // Normally it should already be sufficient to activate IRQ in the CTRL
    // register but a full interrupt is use here to display print output when
    // the timer expires
    cascade_target_2.listen(
        Event::TimeOut,
        InterruptConfig::new(pac::Interrupt::OC3, true, false),
        Some(&mut dp.irqsel),
        Some(&mut dp.sysconfig),
    );
    // The counter will only activate when the cascade signal is coming in so
    // it is okay to call start here to set the reset value
    cascade_target_2.start(1.Hz());

    // Unpend all IRQs
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC1);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC2);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC3);
    }
    // Make both cascade targets accessible from the IRQ handler with the Mutex dance
    cortex_m::interrupt::free(|cs| {
        CSD_TGT_1.borrow(cs).replace(Some(cascade_target_1));
        CSD_TGT_2.borrow(cs).replace(Some(cascade_target_2));
    });
    loop {
        rprintln!("-- Triggering cascade in 0.5 seconds --");
        cascade_triggerer.start(2.Hz());
        delay.delay_ms(5000);
    }
}

#[interrupt]
fn OC0() {
    default_ms_irq_handler()
}

#[interrupt]
fn OC1() {
    static mut IDX: u32 = 0;
    rprintln!("{}: Cascade triggered timed out", &IDX);
    *IDX += 1;
}

#[interrupt]
fn OC2() {
    static mut IDX: u32 = 0;
    rprintln!("{}: First cascade target timed out", &IDX);
    *IDX += 1;
}

#[interrupt]
fn OC3() {
    static mut IDX: u32 = 0;
    rprintln!("{}: Second cascade target timed out", &IDX);
    *IDX += 1;
}
