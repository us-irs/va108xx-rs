//! MS and Second counter implemented using the TIM0 and TIM1 peripheral
#![no_main]
#![no_std]

use core::cell::Cell;
use cortex_m_rt::entry;
use critical_section::Mutex;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    clock::{get_sys_clock, set_sys_clock},
    pac::{self, interrupt},
    prelude::*,
    time::Hertz,
    timer::{default_ms_irq_handler, set_up_ms_tick, CountDownTimer, Event, IrqCfg, MS_COUNTER},
};

#[allow(dead_code)]
enum LibType {
    Pac,
    Hal,
}

static SEC_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut dp = pac::Peripherals::take().unwrap();
    let mut last_ms = 0;
    rprintln!("-- Vorago system ticks using timers --");
    set_sys_clock(50.MHz());
    let lib_type = LibType::Hal;
    match lib_type {
        LibType::Pac => {
            unsafe {
                dp.sysconfig
                    .peripheral_clk_enable()
                    .modify(|_, w| w.irqsel().set_bit());
                dp.sysconfig
                    .tim_clk_enable()
                    .modify(|r, w| w.bits(r.bits() | (1 << 0) | (1 << 1)));
                dp.irqsel.tim0(0).write(|w| w.bits(0x00));
                dp.irqsel.tim0(1).write(|w| w.bits(0x01));
            }

            let sys_clk: Hertz = 50.MHz();
            let cnt_ms = sys_clk.raw() / 1000 - 1;
            let cnt_sec = sys_clk.raw() - 1;
            unsafe {
                dp.tim0.cnt_value().write(|w| w.bits(cnt_ms));
                dp.tim0.rst_value().write(|w| w.bits(cnt_ms));
                dp.tim0.ctrl().write(|w| {
                    w.enable().set_bit();
                    w.irq_enb().set_bit()
                });
                dp.tim1.cnt_value().write(|w| w.bits(cnt_sec));
                dp.tim1.rst_value().write(|w| w.bits(cnt_sec));
                dp.tim1.ctrl().write(|w| {
                    w.enable().set_bit();
                    w.irq_enb().set_bit()
                });
                unmask_irqs();
            }
        }
        LibType::Hal => {
            set_up_ms_tick(
                IrqCfg::new(interrupt::OC0, true, true),
                &mut dp.sysconfig,
                Some(&mut dp.irqsel),
                50.MHz(),
                dp.tim0,
            );
            let mut second_timer =
                CountDownTimer::new(&mut dp.sysconfig, get_sys_clock().unwrap(), dp.tim1);
            second_timer.listen(
                Event::TimeOut,
                IrqCfg::new(interrupt::OC1, true, true),
                Some(&mut dp.irqsel),
                Some(&mut dp.sysconfig),
            );
            second_timer.start(1.Hz());
        }
    }
    loop {
        let current_ms = critical_section::with(|cs| MS_COUNTER.borrow(cs).get());
        if current_ms - last_ms >= 1000 {
            // To prevent drift.
            last_ms += 1000;
            rprintln!("MS counter: {}", current_ms);
            let second = critical_section::with(|cs| SEC_COUNTER.borrow(cs).get());
            rprintln!("Second counter: {}", second);
        }
        cortex_m::asm::delay(10000);
    }
}

fn unmask_irqs() {
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC0);
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OC1);
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC0() {
    default_ms_irq_handler()
}

#[interrupt]
#[allow(non_snake_case)]
fn OC1() {
    critical_section::with(|cs| {
        let mut sec = SEC_COUNTER.borrow(cs).get();
        sec += 1;
        SEC_COUNTER.borrow(cs).set(sec);
    });
}
