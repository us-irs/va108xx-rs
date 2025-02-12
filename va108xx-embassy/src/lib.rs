//! # Embassy-rs support for the Vorago VA108xx MCU family
//!
//! This repository contains the [embassy-rs](https://github.com/embassy-rs/embassy) support for the
//! VA108xx family. Currently, it contains the time driver to allow using embassy-rs. It uses the TIM
//! peripherals provided by the VA108xx family for this purpose.
//!
//! ## Usage
//!
//! This library only exposes the [embassy::init] method which sets up the time driver. This
//! function must be called once at the start of the application.
//!
//! This implementation requires two TIM peripherals provided by the VA108xx device.
//! The user can freely specify the two used TIM peripheral by passing the concrete TIM instances
//! into the [embassy::init_with_custom_irqs] and [embassy::init] method.
//!
//! The application also requires two interrupt handlers to handle the timekeeper and alarm
//! interrupts. By default, this library will define the interrupt handler inside the library
//! itself by using the `irq-oc30-oc31` feature flag. This library exposes three combinations:
//!
//! - `irq-oc30-oc31`: Uses [pac::Interrupt::OC30] and [pac::Interrupt::OC31]
//! - `irq-oc29-oc30`: Uses [pac::Interrupt::OC29] and [pac::Interrupt::OC30]
//! - `irq-oc28-oc29`: Uses [pac::Interrupt::OC28] and [pac::Interrupt::OC20]
//!
//! You can disable the default features and then specify one of the features above to use the
//! documented combination of IRQs. It is also possible to specify custom IRQs by importing and
//! using the [embassy::embassy_time_driver_irqs] macro to declare the IRQ handlers in the
//! application code. If this is done, [embassy::init_with_custom_irqs] must be used
//! method to pass the IRQ numbers to the library.
//!
//! ## Examples
//!
//! [embassy example project](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/embassy)
#![no_std]
use core::cell::{Cell, RefCell};
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::CriticalSectionMutex as Mutex;
use portable_atomic::{AtomicU32, Ordering};

use embassy_time_driver::{time_driver_impl, Driver, TICK_HZ};
use embassy_time_queue_utils::Queue;
use once_cell::sync::OnceCell;
#[cfg(feature = "irqs-in-lib")]
use va108xx_hal::pac::interrupt;
use va108xx_hal::{
    clock::enable_peripheral_clock,
    enable_nvic_interrupt, pac,
    prelude::*,
    timer::{enable_tim_clk, get_tim_raw, TimRegInterface},
    PeripheralSelect,
};

time_driver_impl!(
    static TIME_DRIVER: TimerDriver = TimerDriver {
        periods: AtomicU32::new(0),
        alarms: Mutex::new(AlarmState::new()),
        queue: Mutex::new(RefCell::new(Queue::new())),
});

/// Macro to define the IRQ handlers for the time driver.
///
/// By default, the code generated by this macro will be defined inside the library depending on
/// the feature flags specified. However, the macro is exported to allow users to specify the
/// interrupt handlers themselves.
///
/// Please note that you have to explicitely import the [va108xx_hal::pac::interrupt]
/// macro in the application code in case this macro is used there.
#[macro_export]
macro_rules! embassy_time_driver_irqs {
    (
        timekeeper_irq = $timekeeper_irq:ident,
        alarm_irq = $alarm_irq:ident
    ) => {
        const TIMEKEEPER_IRQ: pac::Interrupt = pac::Interrupt::$timekeeper_irq;

        #[interrupt]
        #[allow(non_snake_case)]
        fn $timekeeper_irq() {
            // Safety: We call it once here.
            unsafe { $crate::embassy::time_driver().on_interrupt_timekeeping() }
        }

        const ALARM_IRQ: pac::Interrupt = pac::Interrupt::$alarm_irq;

        #[interrupt]
        #[allow(non_snake_case)]
        fn $alarm_irq() {
            // Safety: We call it once here.
            unsafe { $crate::embassy::time_driver().on_interrupt_alarm() }
        }
    };
}

// Provide three combinations of IRQs for the time driver by default.

#[cfg(feature = "irq-oc30-oc31")]
embassy_time_driver_irqs!(timekeeper_irq = OC31, alarm_irq = OC30);
#[cfg(feature = "irq-oc29-oc30")]
embassy_time_driver_irqs!(timekeeper_irq = OC30, alarm_irq = OC29);
#[cfg(feature = "irq-oc28-oc29")]
embassy_time_driver_irqs!(timekeeper_irq = OC29, alarm_irq = OC28);

pub mod embassy {
    use super::*;
    use va108xx_hal::{pac, timer::TimRegInterface};

    /// Expose the time driver so the user can specify the IRQ handlers themselves.
    pub fn time_driver() -> &'static TimerDriver {
        &TIME_DRIVER
    }

    /// Initialization method for embassy
    ///
    /// # Safety
    ///
    /// This has to be called once at initialization time to initiate the time driver for
    /// embassy.
    #[cfg(feature = "irqs-in-lib")]
    pub unsafe fn init(
        syscfg: &mut pac::Sysconfig,
        irqsel: &pac::Irqsel,
        sysclk: impl Into<Hertz>,
        timekeeper_tim: impl TimRegInterface,
        alarm_tim: impl TimRegInterface,
    ) {
        TIME_DRIVER.init(
            syscfg,
            irqsel,
            sysclk,
            timekeeper_tim,
            alarm_tim,
            TIMEKEEPER_IRQ,
            ALARM_IRQ,
        )
    }

    /// Initialization method for embassy
    ///
    /// # Safety
    ///
    /// This has to be called once at initialization time to initiate the time driver for
    /// embassy.
    pub unsafe fn init_with_custom_irqs(
        syscfg: &mut pac::Sysconfig,
        irqsel: &pac::Irqsel,
        sysclk: impl Into<Hertz>,
        timekeeper_tim: impl TimRegInterface,
        alarm_tim: impl TimRegInterface,
        timekeeper_irq: pac::Interrupt,
        alarm_irq: pac::Interrupt,
    ) {
        TIME_DRIVER.init(
            syscfg,
            irqsel,
            sysclk,
            timekeeper_tim,
            alarm_tim,
            timekeeper_irq,
            alarm_irq,
        )
    }
}

struct AlarmState {
    timestamp: Cell<u64>,
}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

unsafe impl Send for AlarmState {}

static SCALE: OnceCell<u64> = OnceCell::new();
static TIMEKEEPER_TIM: OnceCell<u8> = OnceCell::new();
static ALARM_TIM: OnceCell<u8> = OnceCell::new();

pub struct TimerDriver {
    periods: AtomicU32,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarms: Mutex<AlarmState>,
    queue: Mutex<RefCell<Queue>>,
}

impl TimerDriver {
    #[allow(clippy::too_many_arguments)]
    fn init(
        &self,
        syscfg: &mut pac::Sysconfig,
        irqsel: &pac::Irqsel,
        sysclk: impl Into<Hertz>,
        timekeeper_tim: impl TimRegInterface,
        alarm_tim: impl TimRegInterface,
        timekeeper_irq: pac::Interrupt,
        alarm_irq: pac::Interrupt,
    ) {
        if ALARM_TIM.get().is_some() {
            return;
        }
        ALARM_TIM.set(alarm_tim.tim_id()).ok();
        TIMEKEEPER_TIM.set(timekeeper_tim.tim_id()).ok();
        enable_peripheral_clock(syscfg, PeripheralSelect::Irqsel);
        enable_tim_clk(syscfg, timekeeper_tim.tim_id());
        let timekeeper_reg_block = timekeeper_tim.reg_block();
        let alarm_tim_reg_block = alarm_tim.reg_block();
        let sysclk = sysclk.into();
        // Initiate scale value here. This is required to convert timer ticks back to a timestamp.
        SCALE.set((sysclk.raw() / TICK_HZ as u32) as u64).unwrap();
        timekeeper_reg_block
            .rst_value()
            .write(|w| unsafe { w.bits(u32::MAX) });
        // Decrementing counter.
        timekeeper_reg_block
            .cnt_value()
            .write(|w| unsafe { w.bits(u32::MAX) });
        // Switch on. Timekeeping should always be done.
        irqsel
            .tim0(timekeeper_tim.tim_id() as usize)
            .write(|w| unsafe { w.bits(timekeeper_irq as u32) });
        unsafe {
            enable_nvic_interrupt(timekeeper_irq);
        }
        timekeeper_reg_block
            .ctrl()
            .modify(|_, w| w.irq_enb().set_bit());
        timekeeper_reg_block
            .enable()
            .write(|w| unsafe { w.bits(1) });

        enable_tim_clk(syscfg, alarm_tim.tim_id());

        // Explicitely disable alarm timer until needed.
        alarm_tim_reg_block.ctrl().modify(|_, w| {
            w.irq_enb().clear_bit();
            w.enable().clear_bit()
        });
        // Enable general interrupts. The IRQ enable of the peripheral remains cleared.
        unsafe {
            enable_nvic_interrupt(alarm_irq);
        }
        irqsel
            .tim0(alarm_tim.tim_id() as usize)
            .write(|w| unsafe { w.bits(alarm_irq as u32) });
    }

    /// Should be called inside the IRQ of the timekeeper timer.
    ///
    /// # Safety
    ///
    /// This function has to be called once by the TIM IRQ used for the timekeeping.
    pub unsafe fn on_interrupt_timekeeping(&self) {
        self.next_period();
    }

    /// Should be called inside the IRQ of the alarm timer.
    ///
    /// # Safety
    ///
    ///This function has to be called once by the TIM IRQ used for the timekeeping.
    pub unsafe fn on_interrupt_alarm(&self) {
        critical_section::with(|cs| {
            if self.alarms.borrow(cs).timestamp.get() <= self.now() {
                self.trigger_alarm(cs)
            }
        })
    }

    fn timekeeper_tim() -> &'static pac::tim0::RegisterBlock {
        TIMEKEEPER_TIM
            .get()
            .map(|idx| unsafe { get_tim_raw(*idx as usize) })
            .unwrap()
    }
    fn alarm_tim() -> &'static pac::tim0::RegisterBlock {
        ALARM_TIM
            .get()
            .map(|idx| unsafe { get_tim_raw(*idx as usize) })
            .unwrap()
    }

    fn next_period(&self) {
        let period = self.periods.fetch_add(1, Ordering::AcqRel) + 1;
        let t = (period as u64) << 32;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs);
            let at = alarm.timestamp.get();
            if at < t {
                self.trigger_alarm(cs);
            } else {
                let alarm_tim = Self::alarm_tim();

                let remaining_ticks = (at - t).checked_mul(*SCALE.get().unwrap());
                if remaining_ticks.is_some_and(|v| v <= u32::MAX as u64) {
                    alarm_tim.enable().write(|w| unsafe { w.bits(0) });
                    alarm_tim
                        .cnt_value()
                        .write(|w| unsafe { w.bits(remaining_ticks.unwrap() as u32) });
                    alarm_tim.ctrl().modify(|_, w| w.irq_enb().set_bit());
                    alarm_tim.enable().write(|w| unsafe { w.bits(1) });
                }
            }
        })
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        Self::alarm_tim().ctrl().modify(|_, w| {
            w.irq_enb().clear_bit();
            w.enable().clear_bit()
        });

        let alarm = &self.alarms.borrow(cs);
        // Setting the maximum value disables the alarm.
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.
        let mut next = self
            .queue
            .borrow(cs)
            .borrow_mut()
            .next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self
                .queue
                .borrow(cs)
                .borrow_mut()
                .next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        if SCALE.get().is_none() {
            return false;
        }
        let alarm_tim = Self::alarm_tim();
        alarm_tim.ctrl().modify(|_, w| {
            w.irq_enb().clear_bit();
            w.enable().clear_bit()
        });

        let alarm = self.alarms.borrow(cs);
        alarm.timestamp.set(timestamp);

        let t = self.now();
        if timestamp <= t {
            alarm.timestamp.set(u64::MAX);
            return false;
        }

        // If it hasn't triggered yet, setup the relevant reset value, regardless of whether
        // the interrupts are enabled or not. When they are enabled at a later point, the
        // right value is already set.

        // If the timestamp is in the next few ticks, add a bit of buffer to be sure the alarm
        // is not missed.
        //
        // This means that an alarm can be delayed for up to 2 ticks (from t+1 to t+3), but this is allowed
        // by the Alarm trait contract. What's not allowed is triggering alarms *before* their scheduled time,
        // and we don't do that here.
        let safe_timestamp = timestamp.max(t + 3);
        let timer_ticks = (safe_timestamp - t).checked_mul(*SCALE.get().unwrap());
        alarm_tim.rst_value().write(|w| unsafe { w.bits(u32::MAX) });
        if timer_ticks.is_some_and(|v| v <= u32::MAX as u64) {
            alarm_tim
                .cnt_value()
                .write(|w| unsafe { w.bits(timer_ticks.unwrap() as u32) });
            alarm_tim.ctrl().modify(|_, w| w.irq_enb().set_bit());
            alarm_tim.enable().write(|w| unsafe { w.bits(1) });
        }
        // If it's too far in the future, don't enable timer yet.
        // It will be enabled later by `next_period`.

        true
    }
}

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        if SCALE.get().is_none() {
            return 0;
        }
        let mut period1: u32;
        let mut period2: u32;
        let mut counter_val: u32;

        loop {
            // Acquire ensures that we get the latest value of `periods` and
            // no instructions can be reordered before the load.
            period1 = self.periods.load(Ordering::Acquire);

            counter_val = u32::MAX - Self::timekeeper_tim().cnt_value().read().bits();

            // Double read to protect against race conditions when the counter is overflowing.
            period2 = self.periods.load(Ordering::Relaxed);
            if period1 == period2 {
                let now = (((period1 as u64) << 32) | counter_val as u64) / *SCALE.get().unwrap();
                return now;
            }
        }
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}
