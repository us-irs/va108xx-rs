//! API for the TIM peripherals
//!
//! ## Examples
//!
//! - [MS and second tick implementation](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/timer-ticks.rs)
//! - [Cascade feature example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/cascade.rs)
pub use crate::InterruptConfig;
use crate::{
    enable_nvic_interrupt,
    pac::{self, tim0},
    pins::{
        Pa0, Pa1, Pa10, Pa11, Pa12, Pa13, Pa14, Pa15, Pa2, Pa24, Pa25, Pa26, Pa27, Pa28, Pa29, Pa3,
        Pa30, Pa31, Pa4, Pa5, Pa6, Pa7, Pa8, Pa9, Pb0, Pb1, Pb10, Pb11, Pb12, Pb13, Pb14, Pb15,
        Pb16, Pb17, Pb18, Pb19, Pb2, Pb20, Pb21, Pb22, Pb23, Pb3, Pb4, Pb5, Pb6,
    },
    sealed::Sealed,
    time::Hertz,
};
use fugit::RateExtU32;
use vorago_shared_periphs::{
    gpio::{Pin, PinId, PinIdProvider},
    ioconfig::regs::FunSel,
    sysconfig::enable_peripheral_clock,
    PeripheralSelect,
};

/// Get the peripheral block of a TIM peripheral given the index.
///
/// This function panics if the given index is greater than 23.
///
/// # Safety
///
/// This returns a direct handle to the peripheral block, which allows to circumvent ownership
/// rules for the peripheral block. You have to ensure that the retrieved peripheral block is not
/// used by any other software component.
#[inline(always)]
pub const unsafe fn get_tim_raw(tim_idx: usize) -> &'static pac::tim0::RegisterBlock {
    match tim_idx {
        0 => unsafe { &*pac::Tim0::ptr() },
        1 => unsafe { &*pac::Tim1::ptr() },
        2 => unsafe { &*pac::Tim2::ptr() },
        3 => unsafe { &*pac::Tim3::ptr() },
        4 => unsafe { &*pac::Tim4::ptr() },
        5 => unsafe { &*pac::Tim5::ptr() },
        6 => unsafe { &*pac::Tim6::ptr() },
        7 => unsafe { &*pac::Tim7::ptr() },
        8 => unsafe { &*pac::Tim8::ptr() },
        9 => unsafe { &*pac::Tim9::ptr() },
        10 => unsafe { &*pac::Tim10::ptr() },
        11 => unsafe { &*pac::Tim11::ptr() },
        12 => unsafe { &*pac::Tim12::ptr() },
        13 => unsafe { &*pac::Tim13::ptr() },
        14 => unsafe { &*pac::Tim14::ptr() },
        15 => unsafe { &*pac::Tim15::ptr() },
        16 => unsafe { &*pac::Tim16::ptr() },
        17 => unsafe { &*pac::Tim17::ptr() },
        18 => unsafe { &*pac::Tim18::ptr() },
        19 => unsafe { &*pac::Tim19::ptr() },
        20 => unsafe { &*pac::Tim20::ptr() },
        21 => unsafe { &*pac::Tim21::ptr() },
        22 => unsafe { &*pac::Tim22::ptr() },
        23 => unsafe { &*pac::Tim23::ptr() },
        _ => {
            panic!("invalid alarm timer index")
        }
    }
}

//==================================================================================================
// Defintions
//==================================================================================================

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

#[derive(Default, Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CascadeCtrl {
    /// Enable Cascade 0 signal active as a requirement for counting
    pub enb_start_src_csd0: bool,
    /// Invert Cascade 0, making it active low
    pub inv_csd0: bool,
    /// Enable Cascade 1 signal active as a requirement for counting
    pub enb_start_src_csd1: bool,
    /// Invert Cascade 1, making it active low
    pub inv_csd1: bool,
    /// Specify required operation if both Cascade 0 and Cascade 1 are active.
    /// 0 is a logical AND of both cascade signals, 1 is a logical OR
    pub dual_csd_op: bool,
    /// Enable trigger mode for Cascade 0. In trigger mode, couting will start with the selected
    /// cascade signal active, but once the counter is active, cascade control will be ignored
    pub trg_csd0: bool,
    /// Trigger mode, identical to [`trg_csd0`](CascadeCtrl) but for Cascade 1
    pub trg_csd1: bool,
    /// Enable Cascade 2 signal active as a requirement to stop counting. This mode is similar
    /// to the REQ_STOP control bit, but signalled by a Cascade source
    pub enb_stop_src_csd2: bool,
    /// Invert Cascade 2, making it active low
    pub inv_csd2: bool,
    /// The counter is automatically disabled if the corresponding Cascade 2 level-sensitive input
    /// souce is active when the count reaches 0. If the counter is not 0, the cascade control is
    /// ignored
    pub trg_csd2: bool,
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CascadeSel {
    Csd0 = 0,
    Csd1 = 1,
    Csd2 = 2,
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidCascadeSourceId;

/// The numbers are the base numbers for bundles like PORTA, PORTB or TIM
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum CascadeSource {
    PortA(u8),
    PortB(u8),
    Tim(u8),
    RamSbe = 96,
    RamMbe = 97,
    RomSbe = 98,
    RomMbe = 99,
    Txev = 100,
    ClockDivider(u8),
}

impl CascadeSource {
    fn id(&self) -> Result<u8, InvalidCascadeSourceId> {
        let port_check = |base: u8, id: u8, len: u8| {
            if id > len - 1 {
                return Err(InvalidCascadeSourceId);
            }
            Ok(base + id)
        };
        match self {
            CascadeSource::PortA(id) => port_check(0, *id, 32),
            CascadeSource::PortB(id) => port_check(32, *id, 32),
            CascadeSource::Tim(id) => port_check(64, *id, 24),
            CascadeSource::RamSbe => Ok(96),
            CascadeSource::RamMbe => Ok(97),
            CascadeSource::RomSbe => Ok(98),
            CascadeSource::RomMbe => Ok(99),
            CascadeSource::Txev => Ok(100),
            CascadeSource::ClockDivider(id) => port_check(120, *id, 8),
        }
    }
}

//==================================================================================================
// Valid TIM and PIN combinations
//==================================================================================================

pub trait TimPin: Sealed {
    const PIN_ID: PinId;
    const FUN_SEL: FunSel;
    const TIM_ID: TimId;
}

pub trait TimMarker: Sealed {
    // TIM ID ranging from 0 to 23 for 24 TIM peripherals
    const ID: TimId;
}

macro_rules! tim_marker {
    ($TIMX:path, $ID:expr) => {
        impl TimMarker for $TIMX {
            const ID: TimId = TimId($ID);
        }

        unsafe impl TimRegInterface for $TIMX {
            fn raw_id(&self) -> u8 {
                Self::ID.0
            }
        }

        impl Sealed for $TIMX {}
    };
}

tim_marker!(pac::Tim0, 0);
tim_marker!(pac::Tim1, 1);
tim_marker!(pac::Tim2, 2);
tim_marker!(pac::Tim3, 3);
tim_marker!(pac::Tim4, 4);
tim_marker!(pac::Tim5, 5);
tim_marker!(pac::Tim6, 6);
tim_marker!(pac::Tim7, 7);
tim_marker!(pac::Tim8, 8);
tim_marker!(pac::Tim9, 9);
tim_marker!(pac::Tim10, 10);
tim_marker!(pac::Tim11, 11);
tim_marker!(pac::Tim12, 12);
tim_marker!(pac::Tim13, 13);
tim_marker!(pac::Tim14, 14);
tim_marker!(pac::Tim15, 15);
tim_marker!(pac::Tim16, 16);
tim_marker!(pac::Tim17, 17);
tim_marker!(pac::Tim18, 18);
tim_marker!(pac::Tim19, 19);
tim_marker!(pac::Tim20, 20);
tim_marker!(pac::Tim21, 21);
tim_marker!(pac::Tim22, 22);
tim_marker!(pac::Tim23, 23);

pub trait ValidTimAndPin<Pin: TimPin, Tim: TimMarker>: Sealed {}

macro_rules! pin_and_tim {
    ($Px:ident, $FunSel:path, $ID:expr) => {
        impl TimPin for Pin<$Px>
        where
            $Px: PinIdProvider,
        {
            const PIN_ID: PinId = $Px::ID;
            const FUN_SEL: FunSel = $FunSel;
            const TIM_ID: TimId = TimId($ID);
        }
    };
}

pin_and_tim!(Pa0, FunSel::Sel1, 0);
pin_and_tim!(Pa1, FunSel::Sel1, 1);
pin_and_tim!(Pa2, FunSel::Sel1, 2);
pin_and_tim!(Pa3, FunSel::Sel1, 3);
pin_and_tim!(Pa4, FunSel::Sel1, 4);
pin_and_tim!(Pa5, FunSel::Sel1, 5);
pin_and_tim!(Pa6, FunSel::Sel1, 6);
pin_and_tim!(Pa7, FunSel::Sel1, 7);
pin_and_tim!(Pa8, FunSel::Sel1, 8);
pin_and_tim!(Pa9, FunSel::Sel1, 9);
pin_and_tim!(Pa10, FunSel::Sel1, 10);
pin_and_tim!(Pa11, FunSel::Sel1, 11);
pin_and_tim!(Pa12, FunSel::Sel1, 12);
pin_and_tim!(Pa13, FunSel::Sel1, 13);
pin_and_tim!(Pa14, FunSel::Sel1, 14);
pin_and_tim!(Pa15, FunSel::Sel1, 15);

pin_and_tim!(Pa24, FunSel::Sel2, 16);
pin_and_tim!(Pa25, FunSel::Sel2, 17);
pin_and_tim!(Pa26, FunSel::Sel2, 18);
pin_and_tim!(Pa27, FunSel::Sel2, 19);
pin_and_tim!(Pa28, FunSel::Sel2, 20);
pin_and_tim!(Pa29, FunSel::Sel2, 21);
pin_and_tim!(Pa30, FunSel::Sel2, 22);
pin_and_tim!(Pa31, FunSel::Sel2, 23);

pin_and_tim!(Pb0, FunSel::Sel3, 0);
pin_and_tim!(Pb1, FunSel::Sel3, 1);
pin_and_tim!(Pb2, FunSel::Sel3, 2);
pin_and_tim!(Pb3, FunSel::Sel3, 3);
pin_and_tim!(Pb4, FunSel::Sel3, 4);
pin_and_tim!(Pb5, FunSel::Sel3, 5);
pin_and_tim!(Pb6, FunSel::Sel3, 6);

pin_and_tim!(Pb10, FunSel::Sel3, 10);
pin_and_tim!(Pb11, FunSel::Sel3, 11);
pin_and_tim!(Pb12, FunSel::Sel3, 12);
pin_and_tim!(Pb13, FunSel::Sel3, 13);
pin_and_tim!(Pb14, FunSel::Sel3, 14);
pin_and_tim!(Pb15, FunSel::Sel3, 15);
pin_and_tim!(Pb16, FunSel::Sel3, 16);
pin_and_tim!(Pb17, FunSel::Sel3, 17);
pin_and_tim!(Pb18, FunSel::Sel3, 18);
pin_and_tim!(Pb19, FunSel::Sel3, 19);
pin_and_tim!(Pb20, FunSel::Sel3, 20);
pin_and_tim!(Pb21, FunSel::Sel3, 21);
pin_and_tim!(Pb22, FunSel::Sel3, 22);
pin_and_tim!(Pb23, FunSel::Sel3, 23);

//==================================================================================================
// Register Interface for TIM registers and TIM pins
//==================================================================================================

pub type TimRegBlock = tim0::RegisterBlock;

/// Register interface.
///
/// This interface provides valid TIM pins a way to access their corresponding TIM
/// registers
///
/// # Safety
///
/// Users should only implement the [Self::raw_id] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that a each
/// pin ID is a singleton.
pub unsafe trait TimRegInterface {
    fn raw_id(&self) -> u8;

    const PORT_BASE: *const tim0::RegisterBlock = pac::Tim0::ptr() as *const _;

    /// All 24 TIM blocks are identical. This helper functions returns the correct
    /// memory mapped peripheral depending on the TIM ID.
    #[inline(always)]
    fn reg_block(&self) -> &TimRegBlock {
        unsafe { &*Self::PORT_BASE.offset(self.raw_id() as isize) }
    }

    #[inline(always)]
    fn mask_32(&self) -> u32 {
        1 << self.raw_id()
    }

    /// Clear the reset bit of the TIM, holding it in reset
    ///
    /// # Safety
    ///
    /// Only the bit related to the corresponding TIM peripheral is modified
    #[inline]
    fn assert_tim_reset(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .sysconfig
                .tim_reset()
                .modify(|r, w| w.bits(r.bits() & !self.mask_32()));
        }
    }

    #[inline]
    fn deassert_tim_reset(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .sysconfig
                .tim_reset()
                .modify(|r, w| w.bits(r.bits() | self.mask_32()));
        }
    }

    fn assert_tim_reset_for_cycles(&self, cycles: u32) {
        self.assert_tim_reset();
        cortex_m::asm::delay(cycles);
        self.deassert_tim_reset();
    }
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimId(u8);

unsafe impl TimRegInterface for TimId {
    fn raw_id(&self) -> u8 {
        self.0
    }
}

//==================================================================================================
// Timers
//==================================================================================================

/// Hardware timers
pub struct CountdownTimer {
    tim: TimId,
    curr_freq: Hertz,
    sys_clk: Hertz,
    rst_val: u32,
    last_cnt: u32,
}

unsafe impl TimRegInterface for CountdownTimer {
    fn raw_id(&self) -> u8 {
        self.tim.0
    }
}

impl CountdownTimer {
    /// Create a countdown timer structure for a given TIM peripheral.
    ///
    /// This does not enable the timer. You can use the [Self::load], [Self::start],
    /// [Self::enable_interrupt] and [Self::enable] API to set up and configure the countdown
    /// timer.
    pub fn new<Tim: TimMarker + TimRegInterface>(sys_clk: Hertz, tim: Tim) -> Self {
        enable_tim_clk(Tim::ID.raw_id());
        tim.assert_tim_reset_for_cycles(2);
        CountdownTimer {
            tim: Tim::ID,
            sys_clk,
            rst_val: 0,
            curr_freq: 0.Hz(),
            last_cnt: 0,
        }
    }

    #[inline(always)]
    pub fn enable(&mut self) {
        self.tim
            .reg_block()
            .enable()
            .write(|w| unsafe { w.bits(1) });
    }

    pub fn enable_interrupt(&mut self, irq_cfg: InterruptConfig) {
        if irq_cfg.route {
            let irqsel = unsafe { pac::Irqsel::steal() };
            enable_peripheral_clock(PeripheralSelect::Irqsel);
            irqsel
                .tim0(self.raw_id() as usize)
                .write(|w| unsafe { w.bits(irq_cfg.id as u32) });
        }
        if irq_cfg.enable_in_nvic {
            unsafe { enable_nvic_interrupt(irq_cfg.id) };
        }
        self.tim
            .reg_block()
            .ctrl()
            .modify(|_, w| w.irq_enb().set_bit());
    }

    /// Calls [Self::load] to configure the specified frequency and then calls [Self::enable].
    pub fn start(&mut self, frequency: impl Into<Hertz>) {
        self.load(frequency);
        self.enable();
    }

    /// Return `Ok` if the timer has wrapped. Peripheral will automatically clear the
    /// flag and restart the time if configured correctly
    pub fn wait(&mut self) -> nb::Result<(), void::Void> {
        let cnt = self.tim.reg_block().cnt_value().read().bits();
        if (cnt > self.last_cnt) || cnt == 0 {
            self.last_cnt = self.rst_val;
            Ok(())
        } else {
            self.last_cnt = cnt;
            Err(nb::Error::WouldBlock)
        }
    }

    /// Load the count down timer with a timeout but do not start it.
    pub fn load(&mut self, timeout: impl Into<Hertz>) {
        self.tim
            .reg_block()
            .ctrl()
            .modify(|_, w| w.enable().clear_bit());
        self.curr_freq = timeout.into();
        self.rst_val = self.sys_clk.raw() / self.curr_freq.raw();
        self.set_reload(self.rst_val);
        self.set_count(self.rst_val);
    }

    #[inline(always)]
    pub fn set_reload(&mut self, val: u32) {
        self.tim
            .reg_block()
            .rst_value()
            .write(|w| unsafe { w.bits(val) });
    }

    #[inline(always)]
    pub fn set_count(&mut self, val: u32) {
        self.tim
            .reg_block()
            .cnt_value()
            .write(|w| unsafe { w.bits(val) });
    }

    #[inline(always)]
    pub fn count(&self) -> u32 {
        self.tim.reg_block().cnt_value().read().bits()
    }

    #[inline(always)]
    pub fn disable(&mut self) {
        self.tim
            .reg_block()
            .enable()
            .write(|w| unsafe { w.bits(0) });
    }

    /// Disable the counter, setting both enable and active bit to 0
    pub fn auto_disable(self, enable: bool) -> Self {
        if enable {
            self.tim
                .reg_block()
                .ctrl()
                .modify(|_, w| w.auto_disable().set_bit());
        } else {
            self.tim
                .reg_block()
                .ctrl()
                .modify(|_, w| w.auto_disable().clear_bit());
        }
        self
    }

    /// This option only applies when the Auto-Disable functionality is 0.
    ///
    /// The active bit is changed to 0 when count reaches 0, but the counter stays
    /// enabled. When Auto-Disable is 1, Auto-Deactivate is implied
    pub fn auto_deactivate(self, enable: bool) -> Self {
        if enable {
            self.tim
                .reg_block()
                .ctrl()
                .modify(|_, w| w.auto_deactivate().set_bit());
        } else {
            self.tim
                .reg_block()
                .ctrl()
                .modify(|_, w| w.auto_deactivate().clear_bit());
        }
        self
    }

    /// Configure the cascade parameters
    pub fn cascade_control(&mut self, ctrl: CascadeCtrl) {
        self.tim.reg_block().csd_ctrl().write(|w| {
            w.csden0().bit(ctrl.enb_start_src_csd0);
            w.csdinv0().bit(ctrl.inv_csd0);
            w.csden1().bit(ctrl.enb_start_src_csd1);
            w.csdinv1().bit(ctrl.inv_csd1);
            w.dcasop().bit(ctrl.dual_csd_op);
            w.csdtrg0().bit(ctrl.trg_csd0);
            w.csdtrg1().bit(ctrl.trg_csd1);
            w.csden2().bit(ctrl.enb_stop_src_csd2);
            w.csdinv2().bit(ctrl.inv_csd2);
            w.csdtrg2().bit(ctrl.trg_csd2)
        });
    }

    pub fn cascade_0_source(&mut self, src: CascadeSource) -> Result<(), InvalidCascadeSourceId> {
        let id = src.id()?;
        self.tim
            .reg_block()
            .cascade0()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn cascade_1_source(&mut self, src: CascadeSource) -> Result<(), InvalidCascadeSourceId> {
        let id = src.id()?;
        self.tim
            .reg_block()
            .cascade1()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn cascade_2_source(&mut self, src: CascadeSource) -> Result<(), InvalidCascadeSourceId> {
        let id = src.id()?;
        self.tim
            .reg_block()
            .cascade2()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn curr_freq(&self) -> Hertz {
        self.curr_freq
    }

    /// This function only clears the interrupt enable bit.
    ///
    /// It does not mask the interrupt in the NVIC or un-route the IRQ.
    #[inline(always)]
    pub fn disable_interrupt(&mut self) {
        self.tim
            .reg_block()
            .ctrl()
            .modify(|_, w| w.irq_enb().clear_bit());
    }

    /// Disables the TIM and the dedicated TIM clock.
    pub fn stop_with_clock_disable(self) {
        self.tim
            .reg_block()
            .ctrl()
            .write(|w| w.enable().clear_bit());
        let syscfg = unsafe { va108xx::Sysconfig::steal() };
        syscfg
            .tim_clk_enable()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.raw_id())) });
    }
}

/// CountDown implementation for TIMx
impl CountdownTimer {}

//==================================================================================================
// Delay implementations
//==================================================================================================
//
impl embedded_hal::delay::DelayNs for CountdownTimer {
    fn delay_ns(&mut self, ns: u32) {
        let ticks = (u64::from(ns)) * (u64::from(self.sys_clk.raw())) / 1_000_000_000;

        let full_cycles = ticks >> 32;
        let mut last_count;
        let mut new_count;
        if full_cycles > 0 {
            self.set_reload(u32::MAX);
            self.set_count(u32::MAX);
            self.enable();

            for _ in 0..full_cycles {
                // Always ensure that both values are the same at the start.
                new_count = self.count();
                last_count = new_count;
                loop {
                    new_count = self.count();
                    if new_count == 0 {
                        // Wait till timer has wrapped.
                        while self.count() == 0 {
                            cortex_m::asm::nop()
                        }
                        break;
                    }
                    // Timer has definitely wrapped.
                    if new_count > last_count {
                        break;
                    }
                    last_count = new_count;
                }
            }
        }
        let ticks = (ticks & u32::MAX as u64) as u32;
        self.disable();
        if ticks > 1 {
            self.set_reload(ticks);
            self.set_count(ticks);
            self.enable();
            last_count = ticks;

            loop {
                new_count = self.count();
                if new_count == 0 || (new_count > last_count) {
                    break;
                }
                last_count = new_count;
            }
        }

        self.disable();
    }
}

#[inline(always)]
pub fn enable_tim_clk(idx: u8) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .tim_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << idx)) });
}

#[inline(always)]
pub fn disable_tim_clk(idx: u8) {
    let syscfg = unsafe { va108xx::Sysconfig::steal() };
    syscfg
        .tim_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << idx)) });
}
