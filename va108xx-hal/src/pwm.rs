//! API for Pulse-Width Modulation (PWM)
//!
//! The Vorago VA108xx devices use the TIM peripherals to perform PWM related tasks
//!
//! ## Examples
//!
//! - [PWM example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/pwm.rs)
use core::convert::Infallible;
use core::marker::PhantomData;

use crate::clock::enable_peripheral_clock;
use crate::pac;
use crate::time::Hertz;
use crate::timer::{TimId, TimPeripheralMarker, TimPin, TimRegInterface};

const DUTY_MAX: u16 = u16::MAX;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum StatusSelPwm {
    PwmA = 3,
    PwmB = 4,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmA {}
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmB {}

#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[error("pin tim ID {pin_tim:?} and timer tim id {tim_id:?} do not match")]
pub struct TimMissmatchError {
    pin_tim: TimId,
    tim_id: TimId,
}

//==================================================================================================
// PWM pin
//==================================================================================================

/// Reduced version where type information is deleted
pub struct PwmPin<Mode = PwmA> {
    tim_id: TimId,
    sys_clk: Hertz,
    /// For PWMB, this is the upper limit
    current_duty: u16,
    /// For PWMA, this value will not be used
    current_lower_limit: u16,
    current_period: Hertz,
    current_rst_val: u32,
    mode: PhantomData<Mode>,
}

impl<Mode> PwmPin<Mode> {
    /// Create a new strongly typed PWM pin
    pub fn new<Pin: TimPin, Tim: TimPeripheralMarker + TimRegInterface>(
        sys_cfg: &mut pac::Sysconfig,
        sys_clk: impl Into<Hertz> + Copy,
        pin_and_tim: (Pin, Tim),
        initial_period: impl Into<Hertz> + Copy,
    ) -> Result<Self, TimMissmatchError> {
        if Pin::TIM_ID != Tim::ID {
            return Err(TimMissmatchError {
                pin_tim: Pin::TIM_ID,
                tim_id: Tim::ID,
            });
        }
        let mut pin = PwmPin {
            tim_id: Tim::ID,
            current_duty: 0,
            current_lower_limit: 0,
            current_period: initial_period.into(),
            current_rst_val: 0,
            sys_clk: sys_clk.into(),
            mode: PhantomData,
        };
        enable_peripheral_clock(crate::clock::PeripheralClocks::Gpio);
        enable_peripheral_clock(crate::clock::PeripheralClocks::Ioconfig);
        sys_cfg
            .tim_clk_enable()
            .modify(|r, w| unsafe { w.bits(r.bits() | pin_and_tim.1.mask_32()) });
        pin.enable_pwm_a();
        pin.set_period(initial_period);
        Ok(pin)
    }

    #[inline]
    fn enable_pwm_a(&mut self) {
        self.tim_id
            .reg_block()
            .ctrl()
            .modify(|_, w| unsafe { w.status_sel().bits(StatusSelPwm::PwmA as u8) });
    }

    #[inline]
    fn enable_pwm_b(&mut self) {
        self.tim_id
            .reg_block()
            .ctrl()
            .modify(|_, w| unsafe { w.status_sel().bits(StatusSelPwm::PwmB as u8) });
    }

    #[inline]
    pub fn get_period(&self) -> Hertz {
        self.current_period
    }

    #[inline]
    pub fn set_period(&mut self, period: impl Into<Hertz>) {
        self.current_period = period.into();
        // Avoid division by 0
        if self.current_period.raw() == 0 {
            return;
        }
        self.current_rst_val = self.sys_clk.raw() / self.current_period.raw();
        self.tim_id
            .reg_block()
            .rst_value()
            .write(|w| unsafe { w.bits(self.current_rst_val) });
    }

    #[inline]
    pub fn disable(&mut self) {
        self.tim_id
            .reg_block()
            .ctrl()
            .modify(|_, w| w.enable().clear_bit());
    }

    #[inline]
    pub fn enable(&mut self) {
        self.tim_id
            .reg_block()
            .ctrl()
            .modify(|_, w| w.enable().set_bit());
    }

    #[inline]
    pub fn period(&self) -> Hertz {
        self.current_period
    }

    #[inline(always)]
    pub fn duty(&self) -> u16 {
        self.current_duty
    }
}

impl From<PwmPin<PwmA>> for PwmPin<PwmB> {
    fn from(other: PwmPin<PwmA>) -> Self {
        let mut pwmb = Self {
            mode: PhantomData,
            tim_id: other.tim_id,
            sys_clk: other.sys_clk,
            current_duty: other.current_duty,
            current_lower_limit: other.current_lower_limit,
            current_period: other.current_period,
            current_rst_val: other.current_rst_val,
        };
        pwmb.enable_pwm_b();
        pwmb
    }
}

impl From<PwmPin<PwmB>> for PwmPin<PwmA> {
    fn from(other: PwmPin<PwmB>) -> Self {
        let mut pwmb = Self {
            mode: PhantomData,
            tim_id: other.tim_id,
            sys_clk: other.sys_clk,
            current_duty: other.current_duty,
            current_lower_limit: other.current_lower_limit,
            current_period: other.current_period,
            current_rst_val: other.current_rst_val,
        };
        pwmb.enable_pwm_a();
        pwmb
    }
}

//==================================================================================================
// PWMB implementations
//==================================================================================================

impl PwmPin<PwmB> {
    #[inline(always)]
    pub fn pwmb_lower_limit(&self) -> u16 {
        self.current_lower_limit
    }

    #[inline(always)]
    pub fn pwmb_upper_limit(&self) -> u16 {
        self.current_duty
    }

    /// Set the lower limit for PWMB
    ///
    /// The PWM signal will be 1 as long as the current RST counter is larger than
    /// the lower limit. For example, with a lower limit of 0.5 and and an upper limit
    /// of 0.7, Only a fixed period between 0.5 * period and 0.7 * period will be in a high
    /// state
    #[inline(always)]
    pub fn set_pwmb_lower_limit(&mut self, duty: u16) {
        self.current_lower_limit = duty;
        let pwmb_val: u64 =
            (self.current_rst_val as u64 * self.current_lower_limit as u64) / DUTY_MAX as u64;
        self.tim_id
            .reg_block()
            .pwmb_value()
            .write(|w| unsafe { w.bits(pwmb_val as u32) });
    }

    /// Set the higher limit for PWMB
    ///
    /// The PWM signal will be 1 as long as the current RST counter is smaller than
    /// the higher limit. For example, with a lower limit of 0.5 and and an upper limit
    /// of 0.7, Only a fixed period between 0.5 * period and 0.7 * period will be in a high
    /// state
    pub fn set_pwmb_upper_limit(&mut self, duty: u16) {
        self.current_duty = duty;
        let pwma_val: u64 =
            (self.current_rst_val as u64 * self.current_duty as u64) / DUTY_MAX as u64;
        self.tim_id
            .reg_block()
            .pwma_value()
            .write(|w| unsafe { w.bits(pwma_val as u32) });
    }
}

//==================================================================================================
// Embedded HAL implementation: PWMA only
//==================================================================================================

impl embedded_hal::pwm::ErrorType for PwmPin {
    type Error = Infallible;
}

impl embedded_hal::pwm::SetDutyCycle for PwmPin {
    #[inline]
    fn max_duty_cycle(&self) -> u16 {
        DUTY_MAX
    }

    #[inline]
    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.current_duty = duty;
        let pwma_val: u64 = (self.current_rst_val as u64
            * (DUTY_MAX as u64 - self.current_duty as u64))
            / DUTY_MAX as u64;
        self.tim_id
            .reg_block()
            .pwma_value()
            .write(|w| unsafe { w.bits(pwma_val as u32) });
        Ok(())
    }
}

/// Get the corresponding u16 duty cycle from a percent value ranging between 0.0 and 1.0.
///
/// Please note that this might load a lot of floating point code because this processor does not
/// have a FPU
pub fn get_duty_from_percent(percent: f32) -> u16 {
    if percent > 1.0 {
        DUTY_MAX
    } else if percent <= 0.0 {
        0
    } else {
        (percent * DUTY_MAX as f32) as u16
    }
}
