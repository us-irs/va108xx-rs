//! API for the TIM peripherals
//!
//! ## Examples
//!
//! - [MS and second tick implementation](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/timer-ticks.rs)
//! - [Cascade feature example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/cascade.rs)
pub use crate::IrqCfg;
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    enable_interrupt,
    gpio::{
        AltFunc1, AltFunc2, AltFunc3, DynPinId, Pin, PinId, PA0, PA1, PA10, PA11, PA12, PA13, PA14,
        PA15, PA2, PA24, PA25, PA26, PA27, PA28, PA29, PA3, PA30, PA31, PA4, PA5, PA6, PA7, PA8,
        PA9, PB0, PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB2, PB20, PB21,
        PB22, PB23, PB3, PB4, PB5, PB6,
    },
    pac::{self, tim0},
    time::Hertz,
    timer,
    typelevel::Sealed,
};
use core::cell::Cell;
use critical_section::Mutex;
use fugit::RateExtU32;

const IRQ_DST_NONE: u32 = 0xffffffff;
pub static MS_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

//==================================================================================================
// Defintions
//==================================================================================================

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

#[derive(Default, Debug, PartialEq, Eq, Copy, Clone)]
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

pub trait TimPin {
    const DYN: DynPinId;
}

pub trait ValidTim {
    // TIM ID ranging from 0 to 23 for 24 TIM peripherals
    const TIM_ID: u8;
}

macro_rules! tim_marker {
    ($TIMX:path, $ID:expr) => {
        impl ValidTim for $TIMX {
            const TIM_ID: u8 = $ID;
        }
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

pub trait ValidTimAndPin<PIN: TimPin, TIM: ValidTim>: Sealed {}

macro_rules! pin_and_tim {
    ($PAX:ident, $ALTFUNC:ident, $ID:expr, $TIMX:path) => {
        impl TimPin for Pin<$PAX, $ALTFUNC>
        where
            $PAX: PinId,
        {
            const DYN: DynPinId = $PAX::DYN;
        }

        impl<PIN: TimPin, TIM: ValidTim> ValidTimAndPin<PIN, TIM> for (Pin<$PAX, $ALTFUNC>, $TIMX)
        where
            Pin<$PAX, $ALTFUNC>: TimPin,
            $PAX: PinId,
        {
        }

        impl Sealed for (Pin<$PAX, $ALTFUNC>, $TIMX) {}
    };
}

pin_and_tim!(PA31, AltFunc2, 23, pac::Tim23);
pin_and_tim!(PA30, AltFunc2, 22, pac::Tim22);
pin_and_tim!(PA29, AltFunc2, 21, pac::Tim21);
pin_and_tim!(PA28, AltFunc2, 20, pac::Tim20);
pin_and_tim!(PA27, AltFunc2, 19, pac::Tim19);
pin_and_tim!(PA26, AltFunc2, 18, pac::Tim18);
pin_and_tim!(PA25, AltFunc2, 17, pac::Tim17);
pin_and_tim!(PA24, AltFunc2, 16, pac::Tim16);

pin_and_tim!(PA15, AltFunc1, 15, pac::Tim15);
pin_and_tim!(PA14, AltFunc1, 14, pac::Tim14);
pin_and_tim!(PA13, AltFunc1, 13, pac::Tim13);
pin_and_tim!(PA12, AltFunc1, 12, pac::Tim12);
pin_and_tim!(PA11, AltFunc1, 11, pac::Tim11);
pin_and_tim!(PA10, AltFunc1, 10, pac::Tim10);
pin_and_tim!(PA9, AltFunc1, 9, pac::Tim9);
pin_and_tim!(PA8, AltFunc1, 8, pac::Tim8);
pin_and_tim!(PA7, AltFunc1, 7, pac::Tim7);
pin_and_tim!(PA6, AltFunc1, 6, pac::Tim6);
pin_and_tim!(PA5, AltFunc1, 5, pac::Tim5);
pin_and_tim!(PA4, AltFunc1, 4, pac::Tim4);
pin_and_tim!(PA3, AltFunc1, 3, pac::Tim3);
pin_and_tim!(PA2, AltFunc1, 2, pac::Tim2);
pin_and_tim!(PA1, AltFunc1, 1, pac::Tim1);
pin_and_tim!(PA0, AltFunc1, 0, pac::Tim0);

pin_and_tim!(PB23, AltFunc3, 23, pac::Tim23);
pin_and_tim!(PB22, AltFunc3, 22, pac::Tim22);
pin_and_tim!(PB21, AltFunc3, 21, pac::Tim21);
pin_and_tim!(PB20, AltFunc3, 20, pac::Tim20);
pin_and_tim!(PB19, AltFunc3, 19, pac::Tim19);
pin_and_tim!(PB18, AltFunc3, 18, pac::Tim18);
pin_and_tim!(PB17, AltFunc3, 17, pac::Tim17);
pin_and_tim!(PB16, AltFunc3, 16, pac::Tim16);
pin_and_tim!(PB15, AltFunc3, 15, pac::Tim15);
pin_and_tim!(PB14, AltFunc3, 14, pac::Tim14);
pin_and_tim!(PB13, AltFunc3, 13, pac::Tim13);
pin_and_tim!(PB12, AltFunc3, 12, pac::Tim12);
pin_and_tim!(PB11, AltFunc3, 11, pac::Tim11);
pin_and_tim!(PB10, AltFunc3, 10, pac::Tim10);

pin_and_tim!(PB6, AltFunc3, 6, pac::Tim6);
pin_and_tim!(PB5, AltFunc3, 5, pac::Tim5);
pin_and_tim!(PB4, AltFunc3, 4, pac::Tim4);
pin_and_tim!(PB3, AltFunc3, 3, pac::Tim3);
pin_and_tim!(PB2, AltFunc3, 2, pac::Tim2);
pin_and_tim!(PB1, AltFunc3, 1, pac::Tim1);
pin_and_tim!(PB0, AltFunc3, 0, pac::Tim0);

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
/// Users should only implement the [`tim_id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that a each
/// pin ID is a singleton.
pub(super) unsafe trait TimRegInterface {
    fn tim_id(&self) -> u8;

    const PORT_BASE: *const tim0::RegisterBlock = pac::Tim0::ptr() as *const _;

    /// All 24 TIM blocks are identical. This helper functions returns the correct
    /// memory mapped peripheral depending on the TIM ID.
    #[inline(always)]
    fn reg(&self) -> &TimRegBlock {
        unsafe { &*Self::PORT_BASE.offset(self.tim_id() as isize) }
    }

    #[inline(always)]
    fn mask_32(&self) -> u32 {
        1 << self.tim_id()
    }

    /// Clear the reset bit of the TIM, holding it in reset
    ///
    /// # Safety
    ///
    /// Only the bit related to the corresponding TIM peripheral is modified
    #[inline]
    #[allow(dead_code)]
    fn clear_tim_reset_bit(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .sysconfig
                .tim_reset()
                .modify(|r, w| w.bits(r.bits() & !self.mask_32()))
        }
    }

    #[inline]
    #[allow(dead_code)]
    fn set_tim_reset_bit(&self) {
        unsafe {
            va108xx::Peripherals::steal()
                .sysconfig
                .tim_reset()
                .modify(|r, w| w.bits(r.bits() | self.mask_32()))
        }
    }
}

/// Provide a safe register interface for [`ValidTimAndPin`]s
///
/// This `struct` takes ownership of a [`ValidTimAndPin`] and provides an API to
/// access the corresponding registers.
pub(super) struct TimAndPinRegister<Pin: TimPin, Tim: ValidTim> {
    pin: Pin,
    tim: Tim,
}

pub(super) struct TimRegister<TIM: ValidTim> {
    tim: TIM,
}

impl<TIM: ValidTim> TimRegister<TIM> {
    #[inline]
    pub(super) unsafe fn new(tim: TIM) -> Self {
        TimRegister { tim }
    }

    pub(super) fn release(self) -> TIM {
        self.tim
    }
}

unsafe impl<TIM: ValidTim> TimRegInterface for TimRegister<TIM> {
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

impl<PIN: TimPin, TIM: ValidTim> TimAndPinRegister<PIN, TIM>
where
    (PIN, TIM): ValidTimAndPin<PIN, TIM>,
{
    #[inline]
    pub(super) unsafe fn new(pin: PIN, tim: TIM) -> Self {
        TimAndPinRegister { pin, tim }
    }

    pub(super) fn release(self) -> (PIN, TIM) {
        (self.pin, self.tim)
    }
}

unsafe impl<PIN: TimPin, TIM: ValidTim> TimRegInterface for TimAndPinRegister<PIN, TIM> {
    #[inline(always)]
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

pub(super) struct TimDynRegister {
    tim_id: u8,
    #[allow(dead_code)]
    pin_id: DynPinId,
}

impl<PIN: TimPin, TIM: ValidTim> From<TimAndPinRegister<PIN, TIM>> for TimDynRegister {
    fn from(_reg: TimAndPinRegister<PIN, TIM>) -> Self {
        Self {
            tim_id: TIM::TIM_ID,
            pin_id: PIN::DYN,
        }
    }
}

unsafe impl TimRegInterface for TimDynRegister {
    #[inline(always)]
    fn tim_id(&self) -> u8 {
        self.tim_id
    }
}

//==================================================================================================
// Timers
//==================================================================================================

/// Hardware timers
pub struct CountDownTimer<TIM: ValidTim> {
    tim: TimRegister<TIM>,
    curr_freq: Hertz,
    irq_cfg: Option<IrqCfg>,
    sys_clk: Hertz,
    rst_val: u32,
    last_cnt: u32,
    listening: bool,
}

#[inline(always)]
pub fn enable_tim_clk(syscfg: &mut pac::Sysconfig, idx: u8) {
    syscfg
        .tim_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << idx)) });
}

#[inline(always)]
pub fn disable_tim_clk(syscfg: &mut pac::Sysconfig, idx: u8) {
    syscfg
        .tim_clk_enable()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << idx)) });
}

unsafe impl<TIM: ValidTim> TimRegInterface for CountDownTimer<TIM> {
    fn tim_id(&self) -> u8 {
        TIM::TIM_ID
    }
}

impl<TIM: ValidTim> CountDownTimer<TIM> {
    /// Configures a TIM peripheral as a periodic count down timer
    pub fn new(syscfg: &mut pac::Sysconfig, sys_clk: impl Into<Hertz>, tim: TIM) -> Self {
        enable_tim_clk(syscfg, TIM::TIM_ID);
        let cd_timer = CountDownTimer {
            tim: unsafe { TimRegister::new(tim) },
            sys_clk: sys_clk.into(),
            irq_cfg: None,
            rst_val: 0,
            curr_freq: 0.Hz(),
            listening: false,
            last_cnt: 0,
        };
        cd_timer
            .tim
            .reg()
            .ctrl()
            .modify(|_, w| w.enable().set_bit());
        cd_timer
    }

    /// Listen for events. Depending on the IRQ configuration, this also activates the IRQ in the
    /// IRQSEL peripheral for the provided interrupt and unmasks the interrupt
    pub fn listen(
        &mut self,
        event: Event,
        irq_cfg: IrqCfg,
        irq_sel: Option<&mut pac::Irqsel>,
        sys_cfg: Option<&mut pac::Sysconfig>,
    ) {
        match event {
            Event::TimeOut => {
                cortex_m::peripheral::NVIC::mask(irq_cfg.irq);
                self.irq_cfg = Some(irq_cfg);
                if irq_cfg.route {
                    if let Some(sys_cfg) = sys_cfg {
                        enable_peripheral_clock(sys_cfg, PeripheralClocks::Irqsel);
                    }
                    if let Some(irq_sel) = irq_sel {
                        irq_sel
                            .tim0(TIM::TIM_ID as usize)
                            .write(|w| unsafe { w.bits(irq_cfg.irq as u32) });
                    }
                }
                self.listening = true;
            }
        }
    }

    pub fn unlisten(
        &mut self,
        event: Event,
        syscfg: &mut pac::Sysconfig,
        irqsel: &mut pac::Irqsel,
    ) {
        match event {
            Event::TimeOut => {
                enable_peripheral_clock(syscfg, PeripheralClocks::Irqsel);
                irqsel
                    .tim0(TIM::TIM_ID as usize)
                    .write(|w| unsafe { w.bits(IRQ_DST_NONE) });
                self.disable_interrupt();
                self.listening = false;
            }
        }
    }

    #[inline(always)]
    pub fn enable_interrupt(&mut self) {
        self.tim.reg().ctrl().modify(|_, w| w.irq_enb().set_bit());
    }

    #[inline(always)]
    pub fn disable_interrupt(&mut self) {
        self.tim.reg().ctrl().modify(|_, w| w.irq_enb().clear_bit());
    }

    pub fn release(self, syscfg: &mut pac::Sysconfig) -> TIM {
        self.tim.reg().ctrl().write(|w| w.enable().clear_bit());
        syscfg
            .tim_clk_enable()
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << TIM::TIM_ID)) });
        self.tim.release()
    }

    /// Load the count down timer with a timeout but do not start it.
    pub fn load(&mut self, timeout: impl Into<Hertz>) {
        self.tim.reg().ctrl().modify(|_, w| w.enable().clear_bit());
        self.curr_freq = timeout.into();
        self.rst_val = self.sys_clk.raw() / self.curr_freq.raw();
        self.set_reload(self.rst_val);
        self.set_count(self.rst_val);
    }

    #[inline(always)]
    pub fn set_reload(&mut self, val: u32) {
        self.tim.reg().rst_value().write(|w| unsafe { w.bits(val) });
    }

    #[inline(always)]
    pub fn set_count(&mut self, val: u32) {
        self.tim.reg().cnt_value().write(|w| unsafe { w.bits(val) });
    }

    #[inline(always)]
    pub fn count(&self) -> u32 {
        self.tim.reg().cnt_value().read().bits()
    }

    #[inline(always)]
    pub fn enable(&mut self) {
        if let Some(irq_cfg) = self.irq_cfg {
            self.enable_interrupt();
            if irq_cfg.enable {
                unsafe { enable_interrupt(irq_cfg.irq) };
            }
        }
        self.tim.reg().enable().write(|w| unsafe { w.bits(1) });
    }

    #[inline(always)]
    pub fn disable(&mut self) {
        self.tim.reg().enable().write(|w| unsafe { w.bits(0) });
    }

    /// Disable the counter, setting both enable and active bit to 0
    pub fn auto_disable(self, enable: bool) -> Self {
        if enable {
            self.tim
                .reg()
                .ctrl()
                .modify(|_, w| w.auto_disable().set_bit());
        } else {
            self.tim
                .reg()
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
                .reg()
                .ctrl()
                .modify(|_, w| w.auto_deactivate().set_bit());
        } else {
            self.tim
                .reg()
                .ctrl()
                .modify(|_, w| w.auto_deactivate().clear_bit());
        }
        self
    }

    /// Configure the cascade parameters
    pub fn cascade_control(&mut self, ctrl: CascadeCtrl) {
        self.tim.reg().csd_ctrl().write(|w| {
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
            .reg()
            .cascade0()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn cascade_1_source(&mut self, src: CascadeSource) -> Result<(), InvalidCascadeSourceId> {
        let id = src.id()?;
        self.tim
            .reg()
            .cascade1()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn cascade_2_source(&mut self, src: CascadeSource) -> Result<(), InvalidCascadeSourceId> {
        let id = src.id()?;
        self.tim
            .reg()
            .cascade2()
            .write(|w| unsafe { w.cassel().bits(id) });
        Ok(())
    }

    pub fn curr_freq(&self) -> Hertz {
        self.curr_freq
    }

    pub fn listening(&self) -> bool {
        self.listening
    }
}

/// CountDown implementation for TIMx
impl<TIM: ValidTim> CountDownTimer<TIM> {
    #[inline]
    pub fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        self.load(timeout);
        self.enable();
    }

    /// Return `Ok` if the timer has wrapped. Peripheral will automatically clear the
    /// flag and restart the time if configured correctly
    pub fn wait(&mut self) -> nb::Result<(), void::Void> {
        let cnt = self.tim.reg().cnt_value().read().bits();
        if (cnt > self.last_cnt) || cnt == 0 {
            self.last_cnt = self.rst_val;
            Ok(())
        } else {
            self.last_cnt = cnt;
            Err(nb::Error::WouldBlock)
        }
    }

    /// Returns [false] if the timer was not active, and true otherwise.
    pub fn cancel(&mut self) -> bool {
        if !self.tim.reg().ctrl().read().enable().bit_is_set() {
            return false;
        }
        self.tim.reg().ctrl().write(|w| w.enable().clear_bit());
        true
    }
}

impl<TIM: ValidTim> embedded_hal::delay::DelayNs for CountDownTimer<TIM> {
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

// Set up a millisecond timer on TIM0. Please note that the user still has to provide an IRQ handler
// which should call [default_ms_irq_handler].
pub fn set_up_ms_tick<TIM: ValidTim>(
    irq_cfg: IrqCfg,
    sys_cfg: &mut pac::Sysconfig,
    irq_sel: Option<&mut pac::Irqsel>,
    sys_clk: impl Into<Hertz>,
    tim0: TIM,
) -> CountDownTimer<TIM> {
    let mut ms_timer = CountDownTimer::new(sys_cfg, sys_clk, tim0);
    ms_timer.listen(timer::Event::TimeOut, irq_cfg, irq_sel, Some(sys_cfg));
    ms_timer.start(1000.Hz());
    ms_timer
}

pub fn set_up_ms_delay_provider<TIM: ValidTim>(
    sys_cfg: &mut pac::Sysconfig,
    sys_clk: impl Into<Hertz>,
    tim: TIM,
) -> CountDownTimer<TIM> {
    let mut provider = CountDownTimer::new(sys_cfg, sys_clk, tim);
    provider.start(1000.Hz());
    provider
}

/// This function can be called in a specified interrupt handler to increment
/// the MS counter
pub fn default_ms_irq_handler() {
    critical_section::with(|cs| {
        let mut ms = MS_COUNTER.borrow(cs).get();
        ms += 1;
        MS_COUNTER.borrow(cs).set(ms);
    });
}

/// Get the current MS tick count
pub fn get_ms_ticks() -> u32 {
    critical_section::with(|cs| MS_COUNTER.borrow(cs).get())
}

//==================================================================================================
// Delay implementations
//==================================================================================================

pub struct DelayMs(CountDownTimer<pac::Tim0>);

impl DelayMs {
    pub fn new(timer: CountDownTimer<pac::Tim0>) -> Option<Self> {
        if timer.curr_freq() != Hertz::from_raw(1000) || !timer.listening() {
            return None;
        }
        Some(Self(timer))
    }
}

/// This assumes that the user has already set up a MS tick timer in TIM0 as a system tick
/// with [`set_up_ms_delay_provider`]
impl embedded_hal::delay::DelayNs for DelayMs {
    fn delay_ns(&mut self, ns: u32) {
        let ns_as_ms = ns / 1_000_000;
        if self.0.curr_freq() != Hertz::from_raw(1000) || !self.0.listening() {
            return;
        }
        let start_time = get_ms_ticks();
        while get_ms_ticks() - start_time < ns_as_ms {
            cortex_m::asm::nop();
        }
    }
}
