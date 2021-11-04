#![feature(prelude_import)]
#![no_std]
#![allow(non_camel_case_types)]
#![allow(clippy::uninit_assumed_init)]
#[prelude_import]
use core::prelude::rust_2018::*;
#[macro_use]
extern crate core;
#[macro_use]
extern crate compiler_builtins;
pub use stm32f0;
#[cfg(any(feature = "stm32f030", feature = "stm32f070"))]
pub use stm32f0::stm32f0x0 as pac;
#[cfg(feature = "device-selected")]
pub mod adc {
    //! # API for the Analog to Digital converter
    //!
    //! Currently implements oneshot conversion with variable sampling times.
    //! Also references for the internal temperature sense, voltage
    //! reference and battery sense are provided.
    //!
    //! ## Example
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::pac;
    //! use crate::hal::prelude::*;
    //! use crate::hal::adc::Adc;
    //!
    //! cortex_m::interrupt::free(|cs| {
    //!     let mut p = pac::Peripherals::take().unwrap();
    //!     let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);
    //!
    //!     let gpioa = p.GPIOA.split(&mut rcc);
    //!
    //!     let mut led = gpioa.pa1.into_push_pull_pull_output(cs);
    //!     let mut an_in = gpioa.pa0.into_analog(cs);
    //!
    //!     let mut delay = Delay::new(cp.SYST, &rcc);
    //!
    //!     let mut adc = Adc::new(p.ADC, &mut rcc);
    //!
    //!     loop {
    //!         let val: u16 = adc.read(&mut an_in).unwrap();
    //!         if val < ((1 << 8) - 1) {
    //!             led.set_low();
    //!         } else {
    //!             led.set_high();
    //!         }
    //!         delay.delay_ms(50_u16);
    //!     }
    //! });
    //! ```
    const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
    const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
    const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
    const VDD_CALIB: u16 = 3300;
    use core::ptr;
    use embedded_hal::{
        adc::{Channel, OneShot},
        blocking::delay::DelayUs,
    };
    use crate::{
        delay::Delay,
        gpio::*,
        pac::{
            adc::{
                cfgr1::{ALIGN_A, RES_A},
                smpr::SMP_A,
            },
            ADC,
        },
        rcc::Rcc,
    };
    /// Analog to Digital converter interface
    pub struct Adc {
        rb: ADC,
        sample_time: AdcSampleTime,
        align: AdcAlign,
        precision: AdcPrecision,
    }
    /// ADC Sampling time
    ///
    /// Options for the sampling time, each is T + 0.5 ADC clock cycles.
    pub enum AdcSampleTime {
        /// 1.5 cycles sampling time
        T_1,
        /// 7.5 cycles sampling time
        T_7,
        /// 13.5 cycles sampling time
        T_13,
        /// 28.5 cycles sampling time
        T_28,
        /// 41.5 cycles sampling time
        T_41,
        /// 55.5 cycles sampling time
        T_55,
        /// 71.5 cycles sampling time
        T_71,
        /// 239.5 cycles sampling time
        T_239,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for AdcSampleTime {
        #[inline]
        fn clone(&self) -> AdcSampleTime {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for AdcSampleTime {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for AdcSampleTime {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&AdcSampleTime::T_1,) => ::core::fmt::Formatter::write_str(f, "T_1"),
                (&AdcSampleTime::T_7,) => ::core::fmt::Formatter::write_str(f, "T_7"),
                (&AdcSampleTime::T_13,) => ::core::fmt::Formatter::write_str(f, "T_13"),
                (&AdcSampleTime::T_28,) => ::core::fmt::Formatter::write_str(f, "T_28"),
                (&AdcSampleTime::T_41,) => ::core::fmt::Formatter::write_str(f, "T_41"),
                (&AdcSampleTime::T_55,) => ::core::fmt::Formatter::write_str(f, "T_55"),
                (&AdcSampleTime::T_71,) => ::core::fmt::Formatter::write_str(f, "T_71"),
                (&AdcSampleTime::T_239,) => ::core::fmt::Formatter::write_str(f, "T_239"),
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for AdcSampleTime {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for AdcSampleTime {
        #[inline]
        fn eq(&self, other: &AdcSampleTime) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl AdcSampleTime {
        /// Get the default sample time (currently 239.5 cycles)
        pub fn default() -> Self {
            AdcSampleTime::T_239
        }
    }
    impl From<AdcSampleTime> for SMP_A {
        fn from(val: AdcSampleTime) -> Self {
            match val {
                AdcSampleTime::T_1 => SMP_A::CYCLES1_5,
                AdcSampleTime::T_7 => SMP_A::CYCLES7_5,
                AdcSampleTime::T_13 => SMP_A::CYCLES13_5,
                AdcSampleTime::T_28 => SMP_A::CYCLES28_5,
                AdcSampleTime::T_41 => SMP_A::CYCLES41_5,
                AdcSampleTime::T_55 => SMP_A::CYCLES55_5,
                AdcSampleTime::T_71 => SMP_A::CYCLES71_5,
                AdcSampleTime::T_239 => SMP_A::CYCLES239_5,
            }
        }
    }
    /// ADC Result Alignment
    pub enum AdcAlign {
        /// Left aligned results (most significant bits)
        ///
        /// Results in all precisions returning a value in the range 0-65535.
        /// Depending on the precision the result will step by larger or smaller
        /// amounts.
        Left,
        /// Right aligned results (least significant bits)
        ///
        /// Results in all precisions returning values from 0-(2^bits-1) in
        /// steps of 1.
        Right,
        /// Left aligned results without correction of 6bit values.
        ///
        /// Returns left aligned results exactly as shown in RM0091 Fig.37.
        /// Where the values are left aligned within the u16, with the exception
        /// of 6 bit mode where the value is left aligned within the first byte of
        /// the u16.
        LeftAsRM,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for AdcAlign {
        #[inline]
        fn clone(&self) -> AdcAlign {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for AdcAlign {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for AdcAlign {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&AdcAlign::Left,) => ::core::fmt::Formatter::write_str(f, "Left"),
                (&AdcAlign::Right,) => ::core::fmt::Formatter::write_str(f, "Right"),
                (&AdcAlign::LeftAsRM,) => ::core::fmt::Formatter::write_str(f, "LeftAsRM"),
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for AdcAlign {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for AdcAlign {
        #[inline]
        fn eq(&self, other: &AdcAlign) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl AdcAlign {
        /// Get the default alignment (currently right aligned)
        pub fn default() -> Self {
            AdcAlign::Right
        }
    }
    impl From<AdcAlign> for ALIGN_A {
        fn from(val: AdcAlign) -> Self {
            match val {
                AdcAlign::Left => ALIGN_A::LEFT,
                AdcAlign::Right => ALIGN_A::RIGHT,
                AdcAlign::LeftAsRM => ALIGN_A::LEFT,
            }
        }
    }
    /// ADC Sampling Precision
    pub enum AdcPrecision {
        /// 12 bit precision
        B_12,
        /// 10 bit precision
        B_10,
        /// 8 bit precision
        B_8,
        /// 6 bit precision
        B_6,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for AdcPrecision {
        #[inline]
        fn clone(&self) -> AdcPrecision {
            {
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for AdcPrecision {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for AdcPrecision {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&AdcPrecision::B_12,) => ::core::fmt::Formatter::write_str(f, "B_12"),
                (&AdcPrecision::B_10,) => ::core::fmt::Formatter::write_str(f, "B_10"),
                (&AdcPrecision::B_8,) => ::core::fmt::Formatter::write_str(f, "B_8"),
                (&AdcPrecision::B_6,) => ::core::fmt::Formatter::write_str(f, "B_6"),
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for AdcPrecision {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for AdcPrecision {
        #[inline]
        fn eq(&self, other: &AdcPrecision) -> bool {
            {
                let __self_vi = ::core::intrinsics::discriminant_value(&*self);
                let __arg_1_vi = ::core::intrinsics::discriminant_value(&*other);
                if true && __self_vi == __arg_1_vi {
                    match (&*self, &*other) {
                        _ => true,
                    }
                } else {
                    false
                }
            }
        }
    }
    impl AdcPrecision {
        /// Get the default precision (currently 12 bit precision)
        pub fn default() -> Self {
            AdcPrecision::B_12
        }
    }
    impl From<AdcPrecision> for RES_A {
        fn from(val: AdcPrecision) -> Self {
            match val {
                AdcPrecision::B_12 => RES_A::TWELVEBIT,
                AdcPrecision::B_10 => RES_A::TENBIT,
                AdcPrecision::B_8 => RES_A::EIGHTBIT,
                AdcPrecision::B_6 => RES_A::SIXBIT,
            }
        }
    }
    impl Channel<Adc> for gpioa::PA0<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            0_u8
        }
    }
    impl Channel<Adc> for gpioa::PA1<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            1_u8
        }
    }
    impl Channel<Adc> for gpioa::PA2<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            2_u8
        }
    }
    impl Channel<Adc> for gpioa::PA3<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            3_u8
        }
    }
    impl Channel<Adc> for gpioa::PA4<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            4_u8
        }
    }
    impl Channel<Adc> for gpioa::PA5<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            5_u8
        }
    }
    impl Channel<Adc> for gpioa::PA6<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            6_u8
        }
    }
    impl Channel<Adc> for gpioa::PA7<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            7_u8
        }
    }
    impl Channel<Adc> for gpiob::PB0<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            8_u8
        }
    }
    impl Channel<Adc> for gpiob::PB1<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            9_u8
        }
    }
    impl Channel<Adc> for gpioc::PC0<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            10_u8
        }
    }
    impl Channel<Adc> for gpioc::PC1<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            11_u8
        }
    }
    impl Channel<Adc> for gpioc::PC2<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            12_u8
        }
    }
    impl Channel<Adc> for gpioc::PC3<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            13_u8
        }
    }
    impl Channel<Adc> for gpioc::PC4<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            14_u8
        }
    }
    impl Channel<Adc> for gpioc::PC5<Analog> {
        type ID = u8;
        fn channel() -> u8 {
            15_u8
        }
    }
    /// Internal temperature sensor (ADC Channel 16)
    pub struct VTemp;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for VTemp {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                VTemp => ::core::fmt::Formatter::write_str(f, "VTemp"),
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::default::Default for VTemp {
        #[inline]
        fn default() -> VTemp {
            VTemp {}
        }
    }
    /// Internal voltage reference (ADC Channel 17)
    pub struct VRef;
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for VRef {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                VRef => ::core::fmt::Formatter::write_str(f, "VRef"),
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::default::Default for VRef {
        #[inline]
        fn default() -> VRef {
            VRef {}
        }
    }
    impl Channel<Adc> for VTemp {
        type ID = u8;
        fn channel() -> u8 {
            16_u8
        }
    }
    impl Channel<Adc> for VRef {
        type ID = u8;
        fn channel() -> u8 {
            17_u8
        }
    }
    impl VTemp {
        /// Init a new VTemp
        pub fn new() -> Self {
            VTemp::default()
        }
        /// Enable the internal temperature sense, this has a wake up time
        /// t<sub>START</sub> which can be found in your micro's datasheet, you
        /// must wait at least that long after enabling before taking a reading.
        /// Remember to disable when not in use.
        pub fn enable(&mut self, adc: &mut Adc) {
            adc.rb.ccr.modify(|_, w| w.tsen().set_bit());
        }
        /// Disable the internal temperature sense.
        pub fn disable(&mut self, adc: &mut Adc) {
            adc.rb.ccr.modify(|_, w| w.tsen().clear_bit());
        }
        /// Checks if the temperature sensor is enabled, does not account for the
        /// t<sub>START</sub> time however.
        pub fn is_enabled(&self, adc: &Adc) -> bool {
            adc.rb.ccr.read().tsen().bit_is_set()
        }
        fn convert_temp(vtemp: u16, vdda: u16) -> i16 {
            let vtemp30_cal = i32::from(unsafe { ptr::read(VTEMPCAL30) }) * 100;
            let vtemp110_cal = i32::from(unsafe { ptr::read(VTEMPCAL110) }) * 100;
            let mut temperature = i32::from(vtemp) * 100;
            temperature = (temperature * (i32::from(vdda) / i32::from(VDD_CALIB))) - vtemp30_cal;
            temperature *= (110 - 30) * 100;
            temperature /= vtemp110_cal - vtemp30_cal;
            temperature += 3000;
            temperature as i16
        }
        /// Read the value of the internal temperature sensor and return the
        /// result in 100ths of a degree centigrade.
        ///
        /// Given a delay reference it will attempt to restrict to the
        /// minimum delay needed to ensure a 10 us t<sub>START</sub> value.
        /// Otherwise it will approximate the required delay using ADC reads.
        pub fn read(adc: &mut Adc, delay: Option<&mut Delay>) -> i16 {
            let mut vtemp = Self::new();
            let vtemp_preenable = vtemp.is_enabled(adc);
            if !vtemp_preenable {
                vtemp.enable(adc);
                if let Some(dref) = delay {
                    dref.delay_us(2_u16);
                } else {
                    VRef::read_vdda(adc);
                }
            }
            let vdda = VRef::read_vdda(adc);
            let prev_cfg = adc.default_cfg();
            let vtemp_val = adc.read(&mut vtemp).unwrap();
            if !vtemp_preenable {
                vtemp.disable(adc);
            }
            adc.restore_cfg(prev_cfg);
            Self::convert_temp(vtemp_val, vdda)
        }
    }
    impl VRef {
        /// Init a new VRef
        pub fn new() -> Self {
            VRef::default()
        }
        /// Enable the internal voltage reference, remember to disable when not in use.
        pub fn enable(&mut self, adc: &mut Adc) {
            adc.rb.ccr.modify(|_, w| w.vrefen().set_bit());
        }
        /// Disable the internal reference voltage.
        pub fn disable(&mut self, adc: &mut Adc) {
            adc.rb.ccr.modify(|_, w| w.vrefen().clear_bit());
        }
        /// Returns if the internal voltage reference is enabled.
        pub fn is_enabled(&self, adc: &Adc) -> bool {
            adc.rb.ccr.read().vrefen().bit_is_set()
        }
        /// Reads the value of VDDA in milli-volts
        pub fn read_vdda(adc: &mut Adc) -> u16 {
            let vrefint_cal = u32::from(unsafe { ptr::read(VREFCAL) });
            let mut vref = Self::new();
            let prev_cfg = adc.default_cfg();
            let vref_val: u32 = if vref.is_enabled(adc) {
                adc.read(&mut vref).unwrap()
            } else {
                vref.enable(adc);
                let ret = adc.read(&mut vref).unwrap();
                vref.disable(adc);
                ret
            };
            adc.restore_cfg(prev_cfg);
            (u32::from(VDD_CALIB) * vrefint_cal / vref_val) as u16
        }
    }
    /// A stored ADC config, can be restored by using the `Adc::restore_cfg` method
    pub struct StoredConfig(AdcSampleTime, AdcAlign, AdcPrecision);
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for StoredConfig {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for StoredConfig {
        #[inline]
        fn clone(&self) -> StoredConfig {
            {
                let _: ::core::clone::AssertParamIsClone<AdcSampleTime>;
                let _: ::core::clone::AssertParamIsClone<AdcAlign>;
                let _: ::core::clone::AssertParamIsClone<AdcPrecision>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for StoredConfig {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match *self {
                StoredConfig(ref __self_0_0, ref __self_0_1, ref __self_0_2) => {
                    let debug_trait_builder =
                        &mut ::core::fmt::Formatter::debug_tuple(f, "StoredConfig");
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_0));
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_1));
                    let _ = ::core::fmt::DebugTuple::field(debug_trait_builder, &&(*__self_0_2));
                    ::core::fmt::DebugTuple::finish(debug_trait_builder)
                }
            }
        }
    }
    impl ::core::marker::StructuralPartialEq for StoredConfig {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for StoredConfig {
        #[inline]
        fn eq(&self, other: &StoredConfig) -> bool {
            match *other {
                StoredConfig(ref __self_1_0, ref __self_1_1, ref __self_1_2) => match *self {
                    StoredConfig(ref __self_0_0, ref __self_0_1, ref __self_0_2) => {
                        (*__self_0_0) == (*__self_1_0)
                            && (*__self_0_1) == (*__self_1_1)
                            && (*__self_0_2) == (*__self_1_2)
                    }
                },
            }
        }
        #[inline]
        fn ne(&self, other: &StoredConfig) -> bool {
            match *other {
                StoredConfig(ref __self_1_0, ref __self_1_1, ref __self_1_2) => match *self {
                    StoredConfig(ref __self_0_0, ref __self_0_1, ref __self_0_2) => {
                        (*__self_0_0) != (*__self_1_0)
                            || (*__self_0_1) != (*__self_1_1)
                            || (*__self_0_2) != (*__self_1_2)
                    }
                },
            }
        }
    }
    impl Adc {
        /// Init a new Adc
        ///
        /// Sets all configurable parameters to defaults, enables the HSI14 clock
        /// for the ADC if it is not already enabled and performs a boot time
        /// calibration. As such this method may take an appreciable time to run.
        pub fn new(adc: ADC, rcc: &mut Rcc) -> Self {
            let mut s = Self {
                rb: adc,
                sample_time: AdcSampleTime::default(),
                align: AdcAlign::default(),
                precision: AdcPrecision::default(),
            };
            s.select_clock(rcc);
            s.calibrate();
            s
        }
        /// Saves a copy of the current ADC config
        pub fn save_cfg(&mut self) -> StoredConfig {
            StoredConfig(self.sample_time, self.align, self.precision)
        }
        /// Restores a stored config
        pub fn restore_cfg(&mut self, cfg: StoredConfig) {
            self.sample_time = cfg.0;
            self.align = cfg.1;
            self.precision = cfg.2;
        }
        /// Resets the ADC config to default, returning the existing config as
        /// a stored config.
        pub fn default_cfg(&mut self) -> StoredConfig {
            let cfg = self.save_cfg();
            self.sample_time = AdcSampleTime::default();
            self.align = AdcAlign::default();
            self.precision = AdcPrecision::default();
            cfg
        }
        /// Set the Adc sampling time
        ///
        /// Options can be found in [AdcSampleTime](crate::adc::AdcSampleTime).
        pub fn set_sample_time(&mut self, t_samp: AdcSampleTime) {
            self.sample_time = t_samp;
        }
        /// Set the Adc result alignment
        ///
        /// Options can be found in [AdcAlign](crate::adc::AdcAlign).
        pub fn set_align(&mut self, align: AdcAlign) {
            self.align = align;
        }
        /// Set the Adc precision
        ///
        /// Options can be found in [AdcPrecision](crate::adc::AdcPrecision).
        pub fn set_precision(&mut self, precision: AdcPrecision) {
            self.precision = precision;
        }
        /// Returns the largest possible sample value for the current settings
        pub fn max_sample(&self) -> u16 {
            match self.align {
                AdcAlign::Left => u16::max_value(),
                AdcAlign::LeftAsRM => match self.precision {
                    AdcPrecision::B_6 => u16::from(u8::max_value()),
                    _ => u16::max_value(),
                },
                AdcAlign::Right => match self.precision {
                    AdcPrecision::B_12 => (1 << 12) - 1,
                    AdcPrecision::B_10 => (1 << 10) - 1,
                    AdcPrecision::B_8 => (1 << 8) - 1,
                    AdcPrecision::B_6 => (1 << 6) - 1,
                },
            }
        }
        /// Read the value of a channel and converts the result to milli-volts
        pub fn read_abs_mv<PIN: Channel<Adc, ID = u8>>(&mut self, pin: &mut PIN) -> u16 {
            let vdda = u32::from(VRef::read_vdda(self));
            let v: u32 = self.read(pin).unwrap();
            let max_samp = u32::from(self.max_sample());
            (v * vdda / max_samp) as u16
        }
        fn calibrate(&mut self) {
            if self.rb.cr.read().aden().is_enabled() {
                self.rb.cr.modify(|_, w| w.addis().disable());
            }
            while self.rb.cr.read().aden().is_enabled() {}
            self.rb.cfgr1.modify(|_, w| w.dmaen().disabled());
            self.rb.cr.modify(|_, w| w.adcal().start_calibration());
            while self.rb.cr.read().adcal().is_calibrating() {}
        }
        fn select_clock(&mut self, rcc: &mut Rcc) {
            rcc.regs.apb2enr.modify(|_, w| w.adcen().enabled());
            rcc.regs.cr2.modify(|_, w| w.hsi14on().on());
            while rcc.regs.cr2.read().hsi14rdy().is_not_ready() {}
        }
        fn power_up(&mut self) {
            if self.rb.isr.read().adrdy().is_ready() {
                self.rb.isr.modify(|_, w| w.adrdy().clear());
            }
            self.rb.cr.modify(|_, w| w.aden().enabled());
            while self.rb.isr.read().adrdy().is_not_ready() {}
        }
        fn power_down(&mut self) {
            self.rb.cr.modify(|_, w| w.adstp().stop_conversion());
            while self.rb.cr.read().adstp().is_stopping() {}
            self.rb.cr.modify(|_, w| w.addis().disable());
            while self.rb.cr.read().aden().is_enabled() {}
        }
        fn convert(&mut self, chan: u8) -> u16 {
            self.rb.chselr.write(|w| unsafe { w.bits(1_u32 << chan) });
            self.rb
                .smpr
                .write(|w| w.smp().variant(self.sample_time.into()));
            self.rb.cfgr1.modify(|_, w| {
                w.res()
                    .variant(self.precision.into())
                    .align()
                    .variant(self.align.into())
            });
            self.rb.cr.modify(|_, w| w.adstart().start_conversion());
            while self.rb.isr.read().eoc().is_not_complete() {}
            let res = self.rb.dr.read().bits() as u16;
            if self.align == AdcAlign::Left && self.precision == AdcPrecision::B_6 {
                res << 8
            } else {
                res
            }
        }
    }
    impl<WORD, PIN> OneShot<Adc, WORD, PIN> for Adc
    where
        WORD: From<u16>,
        PIN: Channel<Adc, ID = u8>,
    {
        type Error = ();
        fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
            self.power_up();
            let res = self.convert(PIN::channel());
            self.power_down();
            Ok(res.into())
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod delay {
    //! API for delays with the systick timer
    //!
    //! Please be aware of potential overflows when using `delay_us`.
    //! E.g. at 48MHz the maximum delay is 89 seconds.
    //!
    //! Consider using the timers api as a more flexible interface
    //!
    //! # Example
    //!
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::pac;
    //! use crate::hal::prelude::*;
    //! use crate::hal::delay::Delay;
    //! use cortex_m::peripheral::Peripherals;
    //!
    //! let mut p = pac::Peripherals::take().unwrap();
    //! let mut cp = cortex_m::Peripherals::take().unwrap();
    //!
    //! let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);
    //! let mut delay = Delay::new(cp.SYST, &rcc);
    //! loop {
    //!     delay.delay_ms(1_000_u16);
    //! }
    //! ```
    use cast::{u16, u32};
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SYST;
    use crate::rcc::Rcc;
    use embedded_hal::blocking::delay::{DelayMs, DelayUs};
    /// System timer (SysTick) as a delay provider
    pub struct Delay {
        scale: u32,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Delay {
        #[inline]
        fn clone(&self) -> Delay {
            match *self {
                Delay {
                    scale: ref __self_0_0,
                } => Delay {
                    scale: ::core::clone::Clone::clone(&(*__self_0_0)),
                },
            }
        }
    }
    const SYSTICK_RANGE: u32 = 0x0100_0000;
    impl Delay {
        /// Configures the system timer (SysTick) as a delay provider
        pub fn new(mut syst: SYST, rcc: &Rcc) -> Delay {
            syst.set_clock_source(SystClkSource::Core);
            syst.set_reload(SYSTICK_RANGE - 1);
            syst.clear_current();
            syst.enable_counter();
            if !(rcc.clocks.hclk().0 >= 1_000_000) {
                ::core::panicking::panic("assertion failed: rcc.clocks.hclk().0 >= 1_000_000")
            };
            let scale = rcc.clocks.hclk().0 / 1_000_000;
            Delay { scale }
        }
    }
    impl DelayMs<u32> for Delay {
        fn delay_ms(&mut self, mut ms: u32) {
            const MAX_MS: u32 = 0x0000_FFFF;
            while ms != 0 {
                let current_ms = if ms <= MAX_MS { ms } else { MAX_MS };
                self.delay_us(current_ms * 1_000);
                ms -= current_ms;
            }
        }
    }
    impl DelayMs<u16> for Delay {
        fn delay_ms(&mut self, ms: u16) {
            self.delay_us(u32(ms) * 1_000);
        }
    }
    impl DelayMs<u8> for Delay {
        fn delay_ms(&mut self, ms: u8) {
            self.delay_ms(u16(ms));
        }
    }
    impl DelayUs<u32> for Delay {
        fn delay_us(&mut self, us: u32) {
            const MAX_TICKS: u32 = 0x007F_FFFF;
            let mut total_ticks = us * self.scale;
            while total_ticks != 0 {
                let current_ticks = if total_ticks <= MAX_TICKS {
                    total_ticks
                } else {
                    MAX_TICKS
                };
                let start_count = SYST::get_current();
                total_ticks -= current_ticks;
                while (start_count.wrapping_sub(SYST::get_current()) % SYSTICK_RANGE)
                    < current_ticks
                {}
            }
        }
    }
    impl DelayUs<u16> for Delay {
        fn delay_us(&mut self, us: u16) {
            self.delay_us(u32(us))
        }
    }
    impl DelayUs<u8> for Delay {
        fn delay_us(&mut self, us: u8) {
            self.delay_us(u32(us))
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod gpio {
    //! General Purpose Input / Output
    use core::convert::Infallible;
    use core::marker::PhantomData;
    use crate::rcc::Rcc;
    /// Extension trait to split a GPIO peripheral in independent pins and registers
    pub trait GpioExt {
        /// The parts to split the GPIO into
        type Parts;
        /// Splits the GPIO block into independent pins and registers
        fn split(self, rcc: &mut Rcc) -> Self::Parts;
    }
    trait GpioRegExt {
        fn is_low(&self, pos: u8) -> bool;
        fn is_set_low(&self, pos: u8) -> bool;
        fn set_high(&self, pos: u8);
        fn set_low(&self, pos: u8);
    }
    /// Alternate function 0
    pub struct AF0;
    /// Alternate function 1
    pub struct AF1;
    /// Alternate function 2
    pub struct AF2;
    /// Alternate function 3
    pub struct AF3;
    /// Alternate function 4
    pub struct AF4;
    /// Alternate function 5
    pub struct AF5;
    /// Alternate function 6
    pub struct AF6;
    /// Alternate function 7
    pub struct AF7;
    /// Alternate function mode (type state)
    pub struct Alternate<AF> {
        _mode: PhantomData<AF>,
    }
    /// Input mode (type state)
    pub struct Input<MODE> {
        _mode: PhantomData<MODE>,
    }
    /// Floating input (type state)
    pub struct Floating;
    /// Pulled down input (type state)
    pub struct PullDown;
    /// Pulled up input (type state)
    pub struct PullUp;
    /// Open drain input or output (type state)
    pub struct OpenDrain;
    /// Analog mode (type state)
    pub struct Analog;
    /// Output mode (type state)
    pub struct Output<MODE> {
        _mode: PhantomData<MODE>,
    }
    /// Push pull output (type state)
    pub struct PushPull;
    use embedded_hal::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};
    /// Fully erased pin
    pub struct Pin<MODE> {
        i: u8,
        port: *const dyn GpioRegExt,
        _mode: PhantomData<MODE>,
    }
    unsafe impl<MODE> Sync for Pin<MODE> {}
    unsafe impl<MODE> Send for Pin<MODE> {}
    impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
        #[inline(always)]
        fn is_set_high(&self) -> Result<bool, Self::Error> {
            self.is_set_low().map(|v| !v)
        }
        #[inline(always)]
        fn is_set_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*self.port).is_set_low(self.i) })
        }
    }
    impl<MODE> OutputPin for Pin<Output<MODE>> {
        type Error = Infallible;
        #[inline(always)]
        fn set_high(&mut self) -> Result<(), Self::Error> {
            unsafe { (*self.port).set_high(self.i) };
            Ok(())
        }
        #[inline(always)]
        fn set_low(&mut self) -> Result<(), Self::Error> {
            unsafe { (*self.port).set_low(self.i) }
            Ok(())
        }
    }
    impl<MODE> toggleable::Default for Pin<Output<MODE>> {}
    impl InputPin for Pin<Output<OpenDrain>> {
        type Error = Infallible;
        #[inline(always)]
        fn is_high(&self) -> Result<bool, Self::Error> {
            self.is_low().map(|v| !v)
        }
        #[inline(always)]
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*self.port).is_low(self.i) })
        }
    }
    impl<MODE> InputPin for Pin<Input<MODE>> {
        type Error = Infallible;
        #[inline(always)]
        fn is_high(&self) -> Result<bool, Self::Error> {
            self.is_low().map(|v| !v)
        }
        #[inline(always)]
        fn is_low(&self) -> Result<bool, Self::Error> {
            Ok(unsafe { (*self.port).is_low(self.i) })
        }
    }
    impl GpioRegExt for crate::pac::gpioa::RegisterBlock {
        fn is_low(&self, pos: u8) -> bool {
            self.idr.read().bits() & (1 << pos) == 0
        }
        fn is_set_low(&self, pos: u8) -> bool {
            self.odr.read().bits() & (1 << pos) == 0
        }
        fn set_high(&self, pos: u8) {
            unsafe { self.bsrr.write(|w| w.bits(1 << pos)) }
        }
        fn set_low(&self, pos: u8) {
            unsafe { self.bsrr.write(|w| w.bits(1 << (pos + 16))) }
        }
    }
    impl GpioRegExt for crate::pac::gpiof::RegisterBlock {
        fn is_low(&self, pos: u8) -> bool {
            self.idr.read().bits() & (1 << pos) == 0
        }
        fn is_set_low(&self, pos: u8) -> bool {
            self.odr.read().bits() & (1 << pos) == 0
        }
        fn set_high(&self, pos: u8) {
            unsafe { self.bsrr.write(|w| w.bits(1 << pos)) }
        }
        fn set_low(&self, pos: u8) {
            unsafe { self.bsrr.write(|w| w.bits(1 << (pos + 16))) }
        }
    }
    /// GPIO
    #[cfg(any(feature = "device-selected"))]
    pub mod gpioa {
        use core::marker::PhantomData;
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
        use crate::{rcc::Rcc, pac::GPIOA};
        use cortex_m::interrupt::CriticalSection;
        use super::{
            Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, Pin, GpioRegExt,
        };
        /// GPIO parts
        pub struct Parts {
            /// Pin
            pub pa0: PA0<Input<Floating>>,
            /// Pin
            pub pa1: PA1<Input<Floating>>,
            /// Pin
            pub pa2: PA2<Input<Floating>>,
            /// Pin
            pub pa3: PA3<Input<Floating>>,
            /// Pin
            pub pa4: PA4<Input<Floating>>,
            /// Pin
            pub pa5: PA5<Input<Floating>>,
            /// Pin
            pub pa6: PA6<Input<Floating>>,
            /// Pin
            pub pa7: PA7<Input<Floating>>,
            /// Pin
            pub pa8: PA8<Input<Floating>>,
            /// Pin
            pub pa9: PA9<Input<Floating>>,
            /// Pin
            pub pa10: PA10<Input<Floating>>,
            /// Pin
            pub pa11: PA11<Input<Floating>>,
            /// Pin
            pub pa12: PA12<Input<Floating>>,
            /// Pin
            pub pa13: PA13<Input<Floating>>,
            /// Pin
            pub pa14: PA14<Input<Floating>>,
            /// Pin
            pub pa15: PA15<Input<Floating>>,
        }
        impl GpioExt for GPIOA {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.regs.ahbenr.modify(|_, w| w.iopaen().set_bit());
                Parts {
                    pa0: PA0 { _mode: PhantomData },
                    pa1: PA1 { _mode: PhantomData },
                    pa2: PA2 { _mode: PhantomData },
                    pa3: PA3 { _mode: PhantomData },
                    pa4: PA4 { _mode: PhantomData },
                    pa5: PA5 { _mode: PhantomData },
                    pa6: PA6 { _mode: PhantomData },
                    pa7: PA7 { _mode: PhantomData },
                    pa8: PA8 { _mode: PhantomData },
                    pa9: PA9 { _mode: PhantomData },
                    pa10: PA10 { _mode: PhantomData },
                    pa11: PA11 { _mode: PhantomData },
                    pa12: PA12 { _mode: PhantomData },
                    pa13: PA13 { _mode: PhantomData },
                    pa14: PA14 { _mode: PhantomData },
                    pa15: PA15 { _mode: PhantomData },
                }
            }
        }
        fn _set_alternate_mode(index: usize, mode: u32) {
            let offset = 2 * index;
            let offset2 = 4 * index;
            unsafe {
                let reg = &(*GPIOA::ptr());
                if offset2 < 32 {
                    reg.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afrh.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                reg.moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
            }
        }
        /// Pin
        pub struct PA0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA0<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA0<Alternate<AF0>> {
                _set_alternate_mode(0, 0);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA0<Alternate<AF1>> {
                _set_alternate_mode(0, 1);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA0<Alternate<AF2>> {
                _set_alternate_mode(0, 2);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA0<Alternate<AF3>> {
                _set_alternate_mode(0, 3);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA0<Alternate<AF4>> {
                _set_alternate_mode(0, 4);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA0<Alternate<AF5>> {
                _set_alternate_mode(0, 5);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA0<Alternate<AF6>> {
                _set_alternate_mode(0, 6);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA0<Alternate<AF7>> {
                _set_alternate_mode(0, 7);
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA0 { _mode: PhantomData }
            }
        }
        impl PA0<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA0<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA0<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 0;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA0<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA0<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(0) })
            }
        }
        impl<MODE> OutputPin for PA0<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(0) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(0) })
            }
        }
        impl<MODE> toggleable::Default for PA0<Output<MODE>> {}
        impl InputPin for PA0<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(0) })
            }
        }
        impl<MODE> PA0<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA0<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(0) })
            }
        }
        /// Pin
        pub struct PA1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA1<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA1<Alternate<AF0>> {
                _set_alternate_mode(1, 0);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA1<Alternate<AF1>> {
                _set_alternate_mode(1, 1);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA1<Alternate<AF2>> {
                _set_alternate_mode(1, 2);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA1<Alternate<AF3>> {
                _set_alternate_mode(1, 3);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA1<Alternate<AF4>> {
                _set_alternate_mode(1, 4);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA1<Alternate<AF5>> {
                _set_alternate_mode(1, 5);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA1<Alternate<AF6>> {
                _set_alternate_mode(1, 6);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA1<Alternate<AF7>> {
                _set_alternate_mode(1, 7);
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA1 { _mode: PhantomData }
            }
        }
        impl PA1<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA1<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA1<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 1;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA1<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA1<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(1) })
            }
        }
        impl<MODE> OutputPin for PA1<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(1) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(1) })
            }
        }
        impl<MODE> toggleable::Default for PA1<Output<MODE>> {}
        impl InputPin for PA1<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(1) })
            }
        }
        impl<MODE> PA1<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA1<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(1) })
            }
        }
        /// Pin
        pub struct PA2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA2<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA2<Alternate<AF0>> {
                _set_alternate_mode(2, 0);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA2<Alternate<AF1>> {
                _set_alternate_mode(2, 1);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA2<Alternate<AF2>> {
                _set_alternate_mode(2, 2);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA2<Alternate<AF3>> {
                _set_alternate_mode(2, 3);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA2<Alternate<AF4>> {
                _set_alternate_mode(2, 4);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA2<Alternate<AF5>> {
                _set_alternate_mode(2, 5);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA2<Alternate<AF6>> {
                _set_alternate_mode(2, 6);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA2<Alternate<AF7>> {
                _set_alternate_mode(2, 7);
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA2 { _mode: PhantomData }
            }
        }
        impl PA2<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA2<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA2<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 2;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA2<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA2<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(2) })
            }
        }
        impl<MODE> OutputPin for PA2<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(2) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(2) })
            }
        }
        impl<MODE> toggleable::Default for PA2<Output<MODE>> {}
        impl InputPin for PA2<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(2) })
            }
        }
        impl<MODE> PA2<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA2<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(2) })
            }
        }
        /// Pin
        pub struct PA3<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA3<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA3<Alternate<AF0>> {
                _set_alternate_mode(3, 0);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA3<Alternate<AF1>> {
                _set_alternate_mode(3, 1);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA3<Alternate<AF2>> {
                _set_alternate_mode(3, 2);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA3<Alternate<AF3>> {
                _set_alternate_mode(3, 3);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA3<Alternate<AF4>> {
                _set_alternate_mode(3, 4);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA3<Alternate<AF5>> {
                _set_alternate_mode(3, 5);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA3<Alternate<AF6>> {
                _set_alternate_mode(3, 6);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA3<Alternate<AF7>> {
                _set_alternate_mode(3, 7);
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA3<Input<Floating>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA3<Input<PullDown>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA3<Input<PullUp>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA3<Analog> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA3<Output<OpenDrain>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA3 { _mode: PhantomData }
            }
        }
        impl PA3<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA3<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA3<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 3;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA3<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA3<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(3) })
            }
        }
        impl<MODE> OutputPin for PA3<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(3) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(3) })
            }
        }
        impl<MODE> toggleable::Default for PA3<Output<MODE>> {}
        impl InputPin for PA3<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(3) })
            }
        }
        impl<MODE> PA3<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA3<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(3) })
            }
        }
        /// Pin
        pub struct PA4<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA4<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA4<Alternate<AF0>> {
                _set_alternate_mode(4, 0);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA4<Alternate<AF1>> {
                _set_alternate_mode(4, 1);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA4<Alternate<AF2>> {
                _set_alternate_mode(4, 2);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA4<Alternate<AF3>> {
                _set_alternate_mode(4, 3);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA4<Alternate<AF4>> {
                _set_alternate_mode(4, 4);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA4<Alternate<AF5>> {
                _set_alternate_mode(4, 5);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA4<Alternate<AF6>> {
                _set_alternate_mode(4, 6);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA4<Alternate<AF7>> {
                _set_alternate_mode(4, 7);
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA4<Input<Floating>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA4<Input<PullDown>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA4<Input<PullUp>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA4<Analog> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA4<Output<OpenDrain>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA4 { _mode: PhantomData }
            }
        }
        impl PA4<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA4<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA4<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 4;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA4<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA4<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(4) })
            }
        }
        impl<MODE> OutputPin for PA4<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(4) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(4) })
            }
        }
        impl<MODE> toggleable::Default for PA4<Output<MODE>> {}
        impl InputPin for PA4<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(4) })
            }
        }
        impl<MODE> PA4<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA4<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(4) })
            }
        }
        /// Pin
        pub struct PA5<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA5<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA5<Alternate<AF0>> {
                _set_alternate_mode(5, 0);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA5<Alternate<AF1>> {
                _set_alternate_mode(5, 1);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA5<Alternate<AF2>> {
                _set_alternate_mode(5, 2);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA5<Alternate<AF3>> {
                _set_alternate_mode(5, 3);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA5<Alternate<AF4>> {
                _set_alternate_mode(5, 4);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA5<Alternate<AF5>> {
                _set_alternate_mode(5, 5);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA5<Alternate<AF6>> {
                _set_alternate_mode(5, 6);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA5<Alternate<AF7>> {
                _set_alternate_mode(5, 7);
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA5<Input<Floating>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA5<Input<PullDown>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA5<Input<PullUp>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA5<Analog> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA5<Output<OpenDrain>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA5 { _mode: PhantomData }
            }
        }
        impl PA5<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA5<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA5<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 5;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA5<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA5<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(5) })
            }
        }
        impl<MODE> OutputPin for PA5<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(5) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(5) })
            }
        }
        impl<MODE> toggleable::Default for PA5<Output<MODE>> {}
        impl InputPin for PA5<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(5) })
            }
        }
        impl<MODE> PA5<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA5<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(5) })
            }
        }
        /// Pin
        pub struct PA6<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA6<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA6<Alternate<AF0>> {
                _set_alternate_mode(6, 0);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA6<Alternate<AF1>> {
                _set_alternate_mode(6, 1);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA6<Alternate<AF2>> {
                _set_alternate_mode(6, 2);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA6<Alternate<AF3>> {
                _set_alternate_mode(6, 3);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA6<Alternate<AF4>> {
                _set_alternate_mode(6, 4);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA6<Alternate<AF5>> {
                _set_alternate_mode(6, 5);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA6<Alternate<AF6>> {
                _set_alternate_mode(6, 6);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA6<Alternate<AF7>> {
                _set_alternate_mode(6, 7);
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA6<Input<Floating>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA6<Input<PullDown>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA6<Input<PullUp>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA6<Analog> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA6<Output<OpenDrain>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA6 { _mode: PhantomData }
            }
        }
        impl PA6<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA6<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA6<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 6;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA6<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA6<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(6) })
            }
        }
        impl<MODE> OutputPin for PA6<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(6) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(6) })
            }
        }
        impl<MODE> toggleable::Default for PA6<Output<MODE>> {}
        impl InputPin for PA6<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(6) })
            }
        }
        impl<MODE> PA6<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA6<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(6) })
            }
        }
        /// Pin
        pub struct PA7<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA7<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA7<Alternate<AF0>> {
                _set_alternate_mode(7, 0);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA7<Alternate<AF1>> {
                _set_alternate_mode(7, 1);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA7<Alternate<AF2>> {
                _set_alternate_mode(7, 2);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA7<Alternate<AF3>> {
                _set_alternate_mode(7, 3);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA7<Alternate<AF4>> {
                _set_alternate_mode(7, 4);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA7<Alternate<AF5>> {
                _set_alternate_mode(7, 5);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA7<Alternate<AF6>> {
                _set_alternate_mode(7, 6);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA7<Alternate<AF7>> {
                _set_alternate_mode(7, 7);
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA7<Input<Floating>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA7<Input<PullDown>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA7<Input<PullUp>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA7<Analog> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA7<Output<OpenDrain>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA7 { _mode: PhantomData }
            }
        }
        impl PA7<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA7<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA7<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 7;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA7<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA7<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(7) })
            }
        }
        impl<MODE> OutputPin for PA7<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(7) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(7) })
            }
        }
        impl<MODE> toggleable::Default for PA7<Output<MODE>> {}
        impl InputPin for PA7<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(7) })
            }
        }
        impl<MODE> PA7<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA7<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(7) })
            }
        }
        /// Pin
        pub struct PA8<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA8<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA8<Alternate<AF0>> {
                _set_alternate_mode(8, 0);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA8<Alternate<AF1>> {
                _set_alternate_mode(8, 1);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA8<Alternate<AF2>> {
                _set_alternate_mode(8, 2);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA8<Alternate<AF3>> {
                _set_alternate_mode(8, 3);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA8<Alternate<AF4>> {
                _set_alternate_mode(8, 4);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA8<Alternate<AF5>> {
                _set_alternate_mode(8, 5);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA8<Alternate<AF6>> {
                _set_alternate_mode(8, 6);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA8<Alternate<AF7>> {
                _set_alternate_mode(8, 7);
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA8<Input<Floating>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA8<Input<PullDown>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA8<Input<PullUp>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA8<Analog> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA8<Output<OpenDrain>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA8 { _mode: PhantomData }
            }
        }
        impl PA8<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA8<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA8<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 8;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA8<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA8<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(8) })
            }
        }
        impl<MODE> OutputPin for PA8<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(8) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(8) })
            }
        }
        impl<MODE> toggleable::Default for PA8<Output<MODE>> {}
        impl InputPin for PA8<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(8) })
            }
        }
        impl<MODE> PA8<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA8<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(8) })
            }
        }
        /// Pin
        pub struct PA9<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA9<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA9<Alternate<AF0>> {
                _set_alternate_mode(9, 0);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA9<Alternate<AF1>> {
                _set_alternate_mode(9, 1);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA9<Alternate<AF2>> {
                _set_alternate_mode(9, 2);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA9<Alternate<AF3>> {
                _set_alternate_mode(9, 3);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA9<Alternate<AF4>> {
                _set_alternate_mode(9, 4);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA9<Alternate<AF5>> {
                _set_alternate_mode(9, 5);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA9<Alternate<AF6>> {
                _set_alternate_mode(9, 6);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA9<Alternate<AF7>> {
                _set_alternate_mode(9, 7);
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA9<Input<Floating>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA9<Input<PullDown>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA9<Input<PullUp>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA9<Analog> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA9<Output<OpenDrain>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA9 { _mode: PhantomData }
            }
        }
        impl PA9<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA9<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA9<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 9;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA9<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA9<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(9) })
            }
        }
        impl<MODE> OutputPin for PA9<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(9) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(9) })
            }
        }
        impl<MODE> toggleable::Default for PA9<Output<MODE>> {}
        impl InputPin for PA9<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(9) })
            }
        }
        impl<MODE> PA9<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA9<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(9) })
            }
        }
        /// Pin
        pub struct PA10<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA10<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA10<Alternate<AF0>> {
                _set_alternate_mode(10, 0);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA10<Alternate<AF1>> {
                _set_alternate_mode(10, 1);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA10<Alternate<AF2>> {
                _set_alternate_mode(10, 2);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA10<Alternate<AF3>> {
                _set_alternate_mode(10, 3);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA10<Alternate<AF4>> {
                _set_alternate_mode(10, 4);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA10<Alternate<AF5>> {
                _set_alternate_mode(10, 5);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA10<Alternate<AF6>> {
                _set_alternate_mode(10, 6);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA10<Alternate<AF7>> {
                _set_alternate_mode(10, 7);
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA10<Input<Floating>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA10<Input<PullDown>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA10<Input<PullUp>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA10<Analog> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA10<Output<OpenDrain>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA10 { _mode: PhantomData }
            }
        }
        impl PA10<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA10<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA10<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 10;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA10<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA10<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(10) })
            }
        }
        impl<MODE> OutputPin for PA10<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(10) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(10) })
            }
        }
        impl<MODE> toggleable::Default for PA10<Output<MODE>> {}
        impl InputPin for PA10<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(10) })
            }
        }
        impl<MODE> PA10<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA10<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(10) })
            }
        }
        /// Pin
        pub struct PA11<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA11<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA11<Alternate<AF0>> {
                _set_alternate_mode(11, 0);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA11<Alternate<AF1>> {
                _set_alternate_mode(11, 1);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA11<Alternate<AF2>> {
                _set_alternate_mode(11, 2);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA11<Alternate<AF3>> {
                _set_alternate_mode(11, 3);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA11<Alternate<AF4>> {
                _set_alternate_mode(11, 4);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA11<Alternate<AF5>> {
                _set_alternate_mode(11, 5);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA11<Alternate<AF6>> {
                _set_alternate_mode(11, 6);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA11<Alternate<AF7>> {
                _set_alternate_mode(11, 7);
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA11<Input<Floating>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA11<Input<PullDown>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA11<Input<PullUp>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA11<Analog> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA11<Output<OpenDrain>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA11 { _mode: PhantomData }
            }
        }
        impl PA11<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA11<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA11<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 11;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA11<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA11<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(11) })
            }
        }
        impl<MODE> OutputPin for PA11<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(11) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(11) })
            }
        }
        impl<MODE> toggleable::Default for PA11<Output<MODE>> {}
        impl InputPin for PA11<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(11) })
            }
        }
        impl<MODE> PA11<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA11<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(11) })
            }
        }
        /// Pin
        pub struct PA12<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA12<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA12<Alternate<AF0>> {
                _set_alternate_mode(12, 0);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA12<Alternate<AF1>> {
                _set_alternate_mode(12, 1);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA12<Alternate<AF2>> {
                _set_alternate_mode(12, 2);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA12<Alternate<AF3>> {
                _set_alternate_mode(12, 3);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA12<Alternate<AF4>> {
                _set_alternate_mode(12, 4);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA12<Alternate<AF5>> {
                _set_alternate_mode(12, 5);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA12<Alternate<AF6>> {
                _set_alternate_mode(12, 6);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA12<Alternate<AF7>> {
                _set_alternate_mode(12, 7);
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA12<Input<Floating>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA12<Input<PullDown>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA12<Input<PullUp>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA12<Analog> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA12<Output<OpenDrain>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA12 { _mode: PhantomData }
            }
        }
        impl PA12<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA12<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA12<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 12;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA12<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA12<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(12) })
            }
        }
        impl<MODE> OutputPin for PA12<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(12) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(12) })
            }
        }
        impl<MODE> toggleable::Default for PA12<Output<MODE>> {}
        impl InputPin for PA12<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(12) })
            }
        }
        impl<MODE> PA12<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA12<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(12) })
            }
        }
        /// Pin
        pub struct PA13<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA13<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA13<Alternate<AF0>> {
                _set_alternate_mode(13, 0);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA13<Alternate<AF1>> {
                _set_alternate_mode(13, 1);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA13<Alternate<AF2>> {
                _set_alternate_mode(13, 2);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA13<Alternate<AF3>> {
                _set_alternate_mode(13, 3);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA13<Alternate<AF4>> {
                _set_alternate_mode(13, 4);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA13<Alternate<AF5>> {
                _set_alternate_mode(13, 5);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA13<Alternate<AF6>> {
                _set_alternate_mode(13, 6);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA13<Alternate<AF7>> {
                _set_alternate_mode(13, 7);
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA13<Input<Floating>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA13<Input<PullDown>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA13<Input<PullUp>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA13<Analog> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA13<Output<OpenDrain>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA13 { _mode: PhantomData }
            }
        }
        impl PA13<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA13<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA13<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 13;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA13<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA13<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(13) })
            }
        }
        impl<MODE> OutputPin for PA13<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(13) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(13) })
            }
        }
        impl<MODE> toggleable::Default for PA13<Output<MODE>> {}
        impl InputPin for PA13<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(13) })
            }
        }
        impl<MODE> PA13<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA13<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(13) })
            }
        }
        /// Pin
        pub struct PA14<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA14<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA14<Alternate<AF0>> {
                _set_alternate_mode(14, 0);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA14<Alternate<AF1>> {
                _set_alternate_mode(14, 1);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA14<Alternate<AF2>> {
                _set_alternate_mode(14, 2);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA14<Alternate<AF3>> {
                _set_alternate_mode(14, 3);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA14<Alternate<AF4>> {
                _set_alternate_mode(14, 4);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA14<Alternate<AF5>> {
                _set_alternate_mode(14, 5);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA14<Alternate<AF6>> {
                _set_alternate_mode(14, 6);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA14<Alternate<AF7>> {
                _set_alternate_mode(14, 7);
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA14<Input<Floating>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA14<Input<PullDown>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA14<Input<PullUp>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA14<Analog> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA14<Output<OpenDrain>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA14 { _mode: PhantomData }
            }
        }
        impl PA14<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA14<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA14<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 14;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA14<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA14<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(14) })
            }
        }
        impl<MODE> OutputPin for PA14<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(14) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(14) })
            }
        }
        impl<MODE> toggleable::Default for PA14<Output<MODE>> {}
        impl InputPin for PA14<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(14) })
            }
        }
        impl<MODE> PA14<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA14<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(14) })
            }
        }
        /// Pin
        pub struct PA15<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PA15<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PA15<Alternate<AF0>> {
                _set_alternate_mode(15, 0);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PA15<Alternate<AF1>> {
                _set_alternate_mode(15, 1);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PA15<Alternate<AF2>> {
                _set_alternate_mode(15, 2);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PA15<Alternate<AF3>> {
                _set_alternate_mode(15, 3);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PA15<Alternate<AF4>> {
                _set_alternate_mode(15, 4);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PA15<Alternate<AF5>> {
                _set_alternate_mode(15, 5);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PA15<Alternate<AF6>> {
                _set_alternate_mode(15, 6);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PA15<Alternate<AF7>> {
                _set_alternate_mode(15, 7);
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PA15<Input<Floating>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PA15<Input<PullDown>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PA15<Input<PullUp>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PA15<Analog> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PA15<Output<OpenDrain>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PA15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PA15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PA15 { _mode: PhantomData }
            }
        }
        impl PA15<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PA15<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PA15<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 15;
                unsafe {
                    let reg = &(*GPIOA::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PA15<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PA15<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_set_low(15) })
            }
        }
        impl<MODE> OutputPin for PA15<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_high(15) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).set_low(15) })
            }
        }
        impl<MODE> toggleable::Default for PA15<Output<MODE>> {}
        impl InputPin for PA15<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(15) })
            }
        }
        impl<MODE> PA15<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOA::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PA15<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOA::ptr()).is_low(15) })
            }
        }
    }
    /// GPIO
    #[cfg(any(feature = "device-selected"))]
    pub mod gpiob {
        use core::marker::PhantomData;
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
        use crate::{rcc::Rcc, pac::GPIOB};
        use cortex_m::interrupt::CriticalSection;
        use super::{
            Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, Pin, GpioRegExt,
        };
        /// GPIO parts
        pub struct Parts {
            /// Pin
            pub pb0: PB0<Input<Floating>>,
            /// Pin
            pub pb1: PB1<Input<Floating>>,
            /// Pin
            pub pb2: PB2<Input<Floating>>,
            /// Pin
            pub pb3: PB3<Input<Floating>>,
            /// Pin
            pub pb4: PB4<Input<Floating>>,
            /// Pin
            pub pb5: PB5<Input<Floating>>,
            /// Pin
            pub pb6: PB6<Input<Floating>>,
            /// Pin
            pub pb7: PB7<Input<Floating>>,
            /// Pin
            pub pb8: PB8<Input<Floating>>,
            /// Pin
            pub pb9: PB9<Input<Floating>>,
            /// Pin
            pub pb10: PB10<Input<Floating>>,
            /// Pin
            pub pb11: PB11<Input<Floating>>,
            /// Pin
            pub pb12: PB12<Input<Floating>>,
            /// Pin
            pub pb13: PB13<Input<Floating>>,
            /// Pin
            pub pb14: PB14<Input<Floating>>,
            /// Pin
            pub pb15: PB15<Input<Floating>>,
        }
        impl GpioExt for GPIOB {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.regs.ahbenr.modify(|_, w| w.iopben().set_bit());
                Parts {
                    pb0: PB0 { _mode: PhantomData },
                    pb1: PB1 { _mode: PhantomData },
                    pb2: PB2 { _mode: PhantomData },
                    pb3: PB3 { _mode: PhantomData },
                    pb4: PB4 { _mode: PhantomData },
                    pb5: PB5 { _mode: PhantomData },
                    pb6: PB6 { _mode: PhantomData },
                    pb7: PB7 { _mode: PhantomData },
                    pb8: PB8 { _mode: PhantomData },
                    pb9: PB9 { _mode: PhantomData },
                    pb10: PB10 { _mode: PhantomData },
                    pb11: PB11 { _mode: PhantomData },
                    pb12: PB12 { _mode: PhantomData },
                    pb13: PB13 { _mode: PhantomData },
                    pb14: PB14 { _mode: PhantomData },
                    pb15: PB15 { _mode: PhantomData },
                }
            }
        }
        fn _set_alternate_mode(index: usize, mode: u32) {
            let offset = 2 * index;
            let offset2 = 4 * index;
            unsafe {
                let reg = &(*GPIOB::ptr());
                if offset2 < 32 {
                    reg.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afrh.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                reg.moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
            }
        }
        /// Pin
        pub struct PB0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB0<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB0<Alternate<AF0>> {
                _set_alternate_mode(0, 0);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB0<Alternate<AF1>> {
                _set_alternate_mode(0, 1);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB0<Alternate<AF2>> {
                _set_alternate_mode(0, 2);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB0<Alternate<AF3>> {
                _set_alternate_mode(0, 3);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB0<Alternate<AF4>> {
                _set_alternate_mode(0, 4);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB0<Alternate<AF5>> {
                _set_alternate_mode(0, 5);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB0<Alternate<AF6>> {
                _set_alternate_mode(0, 6);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB0<Alternate<AF7>> {
                _set_alternate_mode(0, 7);
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB0 { _mode: PhantomData }
            }
        }
        impl PB0<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB0<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB0<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 0;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB0<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB0<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(0) })
            }
        }
        impl<MODE> OutputPin for PB0<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(0) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(0) })
            }
        }
        impl<MODE> toggleable::Default for PB0<Output<MODE>> {}
        impl InputPin for PB0<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(0) })
            }
        }
        impl<MODE> PB0<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB0<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(0) })
            }
        }
        /// Pin
        pub struct PB1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB1<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB1<Alternate<AF0>> {
                _set_alternate_mode(1, 0);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB1<Alternate<AF1>> {
                _set_alternate_mode(1, 1);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB1<Alternate<AF2>> {
                _set_alternate_mode(1, 2);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB1<Alternate<AF3>> {
                _set_alternate_mode(1, 3);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB1<Alternate<AF4>> {
                _set_alternate_mode(1, 4);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB1<Alternate<AF5>> {
                _set_alternate_mode(1, 5);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB1<Alternate<AF6>> {
                _set_alternate_mode(1, 6);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB1<Alternate<AF7>> {
                _set_alternate_mode(1, 7);
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB1 { _mode: PhantomData }
            }
        }
        impl PB1<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB1<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB1<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 1;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB1<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB1<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(1) })
            }
        }
        impl<MODE> OutputPin for PB1<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(1) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(1) })
            }
        }
        impl<MODE> toggleable::Default for PB1<Output<MODE>> {}
        impl InputPin for PB1<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(1) })
            }
        }
        impl<MODE> PB1<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB1<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(1) })
            }
        }
        /// Pin
        pub struct PB2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB2<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB2<Alternate<AF0>> {
                _set_alternate_mode(2, 0);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB2<Alternate<AF1>> {
                _set_alternate_mode(2, 1);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB2<Alternate<AF2>> {
                _set_alternate_mode(2, 2);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB2<Alternate<AF3>> {
                _set_alternate_mode(2, 3);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB2<Alternate<AF4>> {
                _set_alternate_mode(2, 4);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB2<Alternate<AF5>> {
                _set_alternate_mode(2, 5);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB2<Alternate<AF6>> {
                _set_alternate_mode(2, 6);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB2<Alternate<AF7>> {
                _set_alternate_mode(2, 7);
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB2 { _mode: PhantomData }
            }
        }
        impl PB2<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB2<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB2<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 2;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB2<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB2<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(2) })
            }
        }
        impl<MODE> OutputPin for PB2<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(2) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(2) })
            }
        }
        impl<MODE> toggleable::Default for PB2<Output<MODE>> {}
        impl InputPin for PB2<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(2) })
            }
        }
        impl<MODE> PB2<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB2<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(2) })
            }
        }
        /// Pin
        pub struct PB3<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB3<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB3<Alternate<AF0>> {
                _set_alternate_mode(3, 0);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB3<Alternate<AF1>> {
                _set_alternate_mode(3, 1);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB3<Alternate<AF2>> {
                _set_alternate_mode(3, 2);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB3<Alternate<AF3>> {
                _set_alternate_mode(3, 3);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB3<Alternate<AF4>> {
                _set_alternate_mode(3, 4);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB3<Alternate<AF5>> {
                _set_alternate_mode(3, 5);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB3<Alternate<AF6>> {
                _set_alternate_mode(3, 6);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB3<Alternate<AF7>> {
                _set_alternate_mode(3, 7);
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB3<Input<Floating>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB3<Input<PullDown>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB3<Input<PullUp>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB3<Analog> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB3<Output<OpenDrain>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB3 { _mode: PhantomData }
            }
        }
        impl PB3<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB3<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB3<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 3;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB3<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB3<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(3) })
            }
        }
        impl<MODE> OutputPin for PB3<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(3) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(3) })
            }
        }
        impl<MODE> toggleable::Default for PB3<Output<MODE>> {}
        impl InputPin for PB3<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(3) })
            }
        }
        impl<MODE> PB3<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB3<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(3) })
            }
        }
        /// Pin
        pub struct PB4<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB4<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB4<Alternate<AF0>> {
                _set_alternate_mode(4, 0);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB4<Alternate<AF1>> {
                _set_alternate_mode(4, 1);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB4<Alternate<AF2>> {
                _set_alternate_mode(4, 2);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB4<Alternate<AF3>> {
                _set_alternate_mode(4, 3);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB4<Alternate<AF4>> {
                _set_alternate_mode(4, 4);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB4<Alternate<AF5>> {
                _set_alternate_mode(4, 5);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB4<Alternate<AF6>> {
                _set_alternate_mode(4, 6);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB4<Alternate<AF7>> {
                _set_alternate_mode(4, 7);
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB4<Input<Floating>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB4<Input<PullDown>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB4<Input<PullUp>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB4<Analog> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB4<Output<OpenDrain>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB4 { _mode: PhantomData }
            }
        }
        impl PB4<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB4<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB4<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 4;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB4<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB4<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(4) })
            }
        }
        impl<MODE> OutputPin for PB4<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(4) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(4) })
            }
        }
        impl<MODE> toggleable::Default for PB4<Output<MODE>> {}
        impl InputPin for PB4<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(4) })
            }
        }
        impl<MODE> PB4<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB4<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(4) })
            }
        }
        /// Pin
        pub struct PB5<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB5<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB5<Alternate<AF0>> {
                _set_alternate_mode(5, 0);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB5<Alternate<AF1>> {
                _set_alternate_mode(5, 1);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB5<Alternate<AF2>> {
                _set_alternate_mode(5, 2);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB5<Alternate<AF3>> {
                _set_alternate_mode(5, 3);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB5<Alternate<AF4>> {
                _set_alternate_mode(5, 4);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB5<Alternate<AF5>> {
                _set_alternate_mode(5, 5);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB5<Alternate<AF6>> {
                _set_alternate_mode(5, 6);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB5<Alternate<AF7>> {
                _set_alternate_mode(5, 7);
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB5<Input<Floating>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB5<Input<PullDown>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB5<Input<PullUp>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB5<Analog> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB5<Output<OpenDrain>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB5 { _mode: PhantomData }
            }
        }
        impl PB5<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB5<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB5<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 5;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB5<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB5<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(5) })
            }
        }
        impl<MODE> OutputPin for PB5<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(5) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(5) })
            }
        }
        impl<MODE> toggleable::Default for PB5<Output<MODE>> {}
        impl InputPin for PB5<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(5) })
            }
        }
        impl<MODE> PB5<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB5<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(5) })
            }
        }
        /// Pin
        pub struct PB6<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB6<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB6<Alternate<AF0>> {
                _set_alternate_mode(6, 0);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB6<Alternate<AF1>> {
                _set_alternate_mode(6, 1);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB6<Alternate<AF2>> {
                _set_alternate_mode(6, 2);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB6<Alternate<AF3>> {
                _set_alternate_mode(6, 3);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB6<Alternate<AF4>> {
                _set_alternate_mode(6, 4);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB6<Alternate<AF5>> {
                _set_alternate_mode(6, 5);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB6<Alternate<AF6>> {
                _set_alternate_mode(6, 6);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB6<Alternate<AF7>> {
                _set_alternate_mode(6, 7);
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB6<Input<Floating>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB6<Input<PullDown>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB6<Input<PullUp>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB6<Analog> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB6<Output<OpenDrain>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB6 { _mode: PhantomData }
            }
        }
        impl PB6<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB6<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB6<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 6;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB6<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB6<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(6) })
            }
        }
        impl<MODE> OutputPin for PB6<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(6) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(6) })
            }
        }
        impl<MODE> toggleable::Default for PB6<Output<MODE>> {}
        impl InputPin for PB6<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(6) })
            }
        }
        impl<MODE> PB6<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB6<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(6) })
            }
        }
        /// Pin
        pub struct PB7<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB7<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB7<Alternate<AF0>> {
                _set_alternate_mode(7, 0);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB7<Alternate<AF1>> {
                _set_alternate_mode(7, 1);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB7<Alternate<AF2>> {
                _set_alternate_mode(7, 2);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB7<Alternate<AF3>> {
                _set_alternate_mode(7, 3);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB7<Alternate<AF4>> {
                _set_alternate_mode(7, 4);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB7<Alternate<AF5>> {
                _set_alternate_mode(7, 5);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB7<Alternate<AF6>> {
                _set_alternate_mode(7, 6);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB7<Alternate<AF7>> {
                _set_alternate_mode(7, 7);
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB7<Input<Floating>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB7<Input<PullDown>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB7<Input<PullUp>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB7<Analog> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB7<Output<OpenDrain>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB7 { _mode: PhantomData }
            }
        }
        impl PB7<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB7<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB7<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 7;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB7<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB7<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(7) })
            }
        }
        impl<MODE> OutputPin for PB7<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(7) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(7) })
            }
        }
        impl<MODE> toggleable::Default for PB7<Output<MODE>> {}
        impl InputPin for PB7<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(7) })
            }
        }
        impl<MODE> PB7<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB7<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(7) })
            }
        }
        /// Pin
        pub struct PB8<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB8<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB8<Alternate<AF0>> {
                _set_alternate_mode(8, 0);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB8<Alternate<AF1>> {
                _set_alternate_mode(8, 1);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB8<Alternate<AF2>> {
                _set_alternate_mode(8, 2);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB8<Alternate<AF3>> {
                _set_alternate_mode(8, 3);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB8<Alternate<AF4>> {
                _set_alternate_mode(8, 4);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB8<Alternate<AF5>> {
                _set_alternate_mode(8, 5);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB8<Alternate<AF6>> {
                _set_alternate_mode(8, 6);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB8<Alternate<AF7>> {
                _set_alternate_mode(8, 7);
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB8<Input<Floating>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB8<Input<PullDown>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB8<Input<PullUp>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB8<Analog> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB8<Output<OpenDrain>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB8 { _mode: PhantomData }
            }
        }
        impl PB8<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB8<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB8<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 8;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB8<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB8<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(8) })
            }
        }
        impl<MODE> OutputPin for PB8<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(8) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(8) })
            }
        }
        impl<MODE> toggleable::Default for PB8<Output<MODE>> {}
        impl InputPin for PB8<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(8) })
            }
        }
        impl<MODE> PB8<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB8<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(8) })
            }
        }
        /// Pin
        pub struct PB9<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB9<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB9<Alternate<AF0>> {
                _set_alternate_mode(9, 0);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB9<Alternate<AF1>> {
                _set_alternate_mode(9, 1);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB9<Alternate<AF2>> {
                _set_alternate_mode(9, 2);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB9<Alternate<AF3>> {
                _set_alternate_mode(9, 3);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB9<Alternate<AF4>> {
                _set_alternate_mode(9, 4);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB9<Alternate<AF5>> {
                _set_alternate_mode(9, 5);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB9<Alternate<AF6>> {
                _set_alternate_mode(9, 6);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB9<Alternate<AF7>> {
                _set_alternate_mode(9, 7);
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB9<Input<Floating>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB9<Input<PullDown>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB9<Input<PullUp>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB9<Analog> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB9<Output<OpenDrain>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB9 { _mode: PhantomData }
            }
        }
        impl PB9<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB9<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB9<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 9;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB9<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB9<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(9) })
            }
        }
        impl<MODE> OutputPin for PB9<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(9) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(9) })
            }
        }
        impl<MODE> toggleable::Default for PB9<Output<MODE>> {}
        impl InputPin for PB9<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(9) })
            }
        }
        impl<MODE> PB9<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB9<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(9) })
            }
        }
        /// Pin
        pub struct PB10<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB10<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB10<Alternate<AF0>> {
                _set_alternate_mode(10, 0);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB10<Alternate<AF1>> {
                _set_alternate_mode(10, 1);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB10<Alternate<AF2>> {
                _set_alternate_mode(10, 2);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB10<Alternate<AF3>> {
                _set_alternate_mode(10, 3);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB10<Alternate<AF4>> {
                _set_alternate_mode(10, 4);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB10<Alternate<AF5>> {
                _set_alternate_mode(10, 5);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB10<Alternate<AF6>> {
                _set_alternate_mode(10, 6);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB10<Alternate<AF7>> {
                _set_alternate_mode(10, 7);
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB10<Input<Floating>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB10<Input<PullDown>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB10<Input<PullUp>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB10<Analog> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB10<Output<OpenDrain>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB10 { _mode: PhantomData }
            }
        }
        impl PB10<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB10<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB10<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 10;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB10<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB10<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(10) })
            }
        }
        impl<MODE> OutputPin for PB10<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(10) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(10) })
            }
        }
        impl<MODE> toggleable::Default for PB10<Output<MODE>> {}
        impl InputPin for PB10<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(10) })
            }
        }
        impl<MODE> PB10<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB10<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(10) })
            }
        }
        /// Pin
        pub struct PB11<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB11<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB11<Alternate<AF0>> {
                _set_alternate_mode(11, 0);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB11<Alternate<AF1>> {
                _set_alternate_mode(11, 1);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB11<Alternate<AF2>> {
                _set_alternate_mode(11, 2);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB11<Alternate<AF3>> {
                _set_alternate_mode(11, 3);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB11<Alternate<AF4>> {
                _set_alternate_mode(11, 4);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB11<Alternate<AF5>> {
                _set_alternate_mode(11, 5);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB11<Alternate<AF6>> {
                _set_alternate_mode(11, 6);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB11<Alternate<AF7>> {
                _set_alternate_mode(11, 7);
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB11<Input<Floating>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB11<Input<PullDown>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB11<Input<PullUp>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB11<Analog> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB11<Output<OpenDrain>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB11 { _mode: PhantomData }
            }
        }
        impl PB11<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB11<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB11<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 11;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB11<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB11<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(11) })
            }
        }
        impl<MODE> OutputPin for PB11<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(11) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(11) })
            }
        }
        impl<MODE> toggleable::Default for PB11<Output<MODE>> {}
        impl InputPin for PB11<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(11) })
            }
        }
        impl<MODE> PB11<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB11<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(11) })
            }
        }
        /// Pin
        pub struct PB12<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB12<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB12<Alternate<AF0>> {
                _set_alternate_mode(12, 0);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB12<Alternate<AF1>> {
                _set_alternate_mode(12, 1);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB12<Alternate<AF2>> {
                _set_alternate_mode(12, 2);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB12<Alternate<AF3>> {
                _set_alternate_mode(12, 3);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB12<Alternate<AF4>> {
                _set_alternate_mode(12, 4);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB12<Alternate<AF5>> {
                _set_alternate_mode(12, 5);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB12<Alternate<AF6>> {
                _set_alternate_mode(12, 6);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB12<Alternate<AF7>> {
                _set_alternate_mode(12, 7);
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB12<Input<Floating>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB12<Input<PullDown>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB12<Input<PullUp>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB12<Analog> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB12<Output<OpenDrain>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB12 { _mode: PhantomData }
            }
        }
        impl PB12<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB12<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB12<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 12;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB12<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB12<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(12) })
            }
        }
        impl<MODE> OutputPin for PB12<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(12) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(12) })
            }
        }
        impl<MODE> toggleable::Default for PB12<Output<MODE>> {}
        impl InputPin for PB12<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(12) })
            }
        }
        impl<MODE> PB12<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB12<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(12) })
            }
        }
        /// Pin
        pub struct PB13<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB13<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB13<Alternate<AF0>> {
                _set_alternate_mode(13, 0);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB13<Alternate<AF1>> {
                _set_alternate_mode(13, 1);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB13<Alternate<AF2>> {
                _set_alternate_mode(13, 2);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB13<Alternate<AF3>> {
                _set_alternate_mode(13, 3);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB13<Alternate<AF4>> {
                _set_alternate_mode(13, 4);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB13<Alternate<AF5>> {
                _set_alternate_mode(13, 5);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB13<Alternate<AF6>> {
                _set_alternate_mode(13, 6);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB13<Alternate<AF7>> {
                _set_alternate_mode(13, 7);
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB13<Input<Floating>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB13<Input<PullDown>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB13<Input<PullUp>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB13<Analog> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB13<Output<OpenDrain>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB13 { _mode: PhantomData }
            }
        }
        impl PB13<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB13<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB13<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 13;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB13<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB13<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(13) })
            }
        }
        impl<MODE> OutputPin for PB13<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(13) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(13) })
            }
        }
        impl<MODE> toggleable::Default for PB13<Output<MODE>> {}
        impl InputPin for PB13<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(13) })
            }
        }
        impl<MODE> PB13<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB13<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(13) })
            }
        }
        /// Pin
        pub struct PB14<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB14<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB14<Alternate<AF0>> {
                _set_alternate_mode(14, 0);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB14<Alternate<AF1>> {
                _set_alternate_mode(14, 1);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB14<Alternate<AF2>> {
                _set_alternate_mode(14, 2);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB14<Alternate<AF3>> {
                _set_alternate_mode(14, 3);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB14<Alternate<AF4>> {
                _set_alternate_mode(14, 4);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB14<Alternate<AF5>> {
                _set_alternate_mode(14, 5);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB14<Alternate<AF6>> {
                _set_alternate_mode(14, 6);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB14<Alternate<AF7>> {
                _set_alternate_mode(14, 7);
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB14<Input<Floating>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB14<Input<PullDown>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB14<Input<PullUp>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB14<Analog> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB14<Output<OpenDrain>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB14 { _mode: PhantomData }
            }
        }
        impl PB14<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB14<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB14<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 14;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB14<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB14<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(14) })
            }
        }
        impl<MODE> OutputPin for PB14<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(14) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(14) })
            }
        }
        impl<MODE> toggleable::Default for PB14<Output<MODE>> {}
        impl InputPin for PB14<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(14) })
            }
        }
        impl<MODE> PB14<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB14<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(14) })
            }
        }
        /// Pin
        pub struct PB15<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PB15<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PB15<Alternate<AF0>> {
                _set_alternate_mode(15, 0);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PB15<Alternate<AF1>> {
                _set_alternate_mode(15, 1);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PB15<Alternate<AF2>> {
                _set_alternate_mode(15, 2);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PB15<Alternate<AF3>> {
                _set_alternate_mode(15, 3);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PB15<Alternate<AF4>> {
                _set_alternate_mode(15, 4);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PB15<Alternate<AF5>> {
                _set_alternate_mode(15, 5);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PB15<Alternate<AF6>> {
                _set_alternate_mode(15, 6);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PB15<Alternate<AF7>> {
                _set_alternate_mode(15, 7);
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PB15<Input<Floating>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PB15<Input<PullDown>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PB15<Input<PullUp>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PB15<Analog> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PB15<Output<OpenDrain>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PB15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PB15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PB15 { _mode: PhantomData }
            }
        }
        impl PB15<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PB15<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PB15<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 15;
                unsafe {
                    let reg = &(*GPIOB::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PB15<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PB15<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_set_low(15) })
            }
        }
        impl<MODE> OutputPin for PB15<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_high(15) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).set_low(15) })
            }
        }
        impl<MODE> toggleable::Default for PB15<Output<MODE>> {}
        impl InputPin for PB15<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(15) })
            }
        }
        impl<MODE> PB15<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOB::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PB15<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOB::ptr()).is_low(15) })
            }
        }
    }
    /// GPIO
    #[cfg(any(
        feature = "stm32f030",
        feature = "stm32f051",
        feature = "stm32f058",
        feature = "stm32f070",
        feature = "stm32f071",
        feature = "stm32f072",
        feature = "stm32f078",
        feature = "stm32f091",
        feature = "stm32f098"
    ))]
    pub mod gpioc {
        use core::marker::PhantomData;
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
        use crate::{rcc::Rcc, pac::GPIOC};
        use cortex_m::interrupt::CriticalSection;
        use super::{
            Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, Pin, GpioRegExt,
        };
        /// GPIO parts
        pub struct Parts {
            /// Pin
            pub pc0: PC0<Input<Floating>>,
            /// Pin
            pub pc1: PC1<Input<Floating>>,
            /// Pin
            pub pc2: PC2<Input<Floating>>,
            /// Pin
            pub pc3: PC3<Input<Floating>>,
            /// Pin
            pub pc4: PC4<Input<Floating>>,
            /// Pin
            pub pc5: PC5<Input<Floating>>,
            /// Pin
            pub pc6: PC6<Input<Floating>>,
            /// Pin
            pub pc7: PC7<Input<Floating>>,
            /// Pin
            pub pc8: PC8<Input<Floating>>,
            /// Pin
            pub pc9: PC9<Input<Floating>>,
            /// Pin
            pub pc10: PC10<Input<Floating>>,
            /// Pin
            pub pc11: PC11<Input<Floating>>,
            /// Pin
            pub pc12: PC12<Input<Floating>>,
            /// Pin
            pub pc13: PC13<Input<Floating>>,
            /// Pin
            pub pc14: PC14<Input<Floating>>,
            /// Pin
            pub pc15: PC15<Input<Floating>>,
        }
        impl GpioExt for GPIOC {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.regs.ahbenr.modify(|_, w| w.iopcen().set_bit());
                Parts {
                    pc0: PC0 { _mode: PhantomData },
                    pc1: PC1 { _mode: PhantomData },
                    pc2: PC2 { _mode: PhantomData },
                    pc3: PC3 { _mode: PhantomData },
                    pc4: PC4 { _mode: PhantomData },
                    pc5: PC5 { _mode: PhantomData },
                    pc6: PC6 { _mode: PhantomData },
                    pc7: PC7 { _mode: PhantomData },
                    pc8: PC8 { _mode: PhantomData },
                    pc9: PC9 { _mode: PhantomData },
                    pc10: PC10 { _mode: PhantomData },
                    pc11: PC11 { _mode: PhantomData },
                    pc12: PC12 { _mode: PhantomData },
                    pc13: PC13 { _mode: PhantomData },
                    pc14: PC14 { _mode: PhantomData },
                    pc15: PC15 { _mode: PhantomData },
                }
            }
        }
        fn _set_alternate_mode(index: usize, mode: u32) {
            let offset = 2 * index;
            let offset2 = 4 * index;
            unsafe {
                let reg = &(*GPIOC::ptr());
                if offset2 < 32 {
                    reg.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afrh.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                reg.moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
            }
        }
        /// Pin
        pub struct PC0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC0<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC0<Alternate<AF0>> {
                _set_alternate_mode(0, 0);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC0<Alternate<AF1>> {
                _set_alternate_mode(0, 1);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC0<Alternate<AF2>> {
                _set_alternate_mode(0, 2);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC0<Alternate<AF3>> {
                _set_alternate_mode(0, 3);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC0<Alternate<AF4>> {
                _set_alternate_mode(0, 4);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC0<Alternate<AF5>> {
                _set_alternate_mode(0, 5);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC0<Alternate<AF6>> {
                _set_alternate_mode(0, 6);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC0<Alternate<AF7>> {
                _set_alternate_mode(0, 7);
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC0 { _mode: PhantomData }
            }
        }
        impl PC0<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC0<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC0<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 0;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC0<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC0<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(0) })
            }
        }
        impl<MODE> OutputPin for PC0<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(0) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(0) })
            }
        }
        impl<MODE> toggleable::Default for PC0<Output<MODE>> {}
        impl InputPin for PC0<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(0) })
            }
        }
        impl<MODE> PC0<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC0<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(0) })
            }
        }
        /// Pin
        pub struct PC1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC1<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC1<Alternate<AF0>> {
                _set_alternate_mode(1, 0);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC1<Alternate<AF1>> {
                _set_alternate_mode(1, 1);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC1<Alternate<AF2>> {
                _set_alternate_mode(1, 2);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC1<Alternate<AF3>> {
                _set_alternate_mode(1, 3);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC1<Alternate<AF4>> {
                _set_alternate_mode(1, 4);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC1<Alternate<AF5>> {
                _set_alternate_mode(1, 5);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC1<Alternate<AF6>> {
                _set_alternate_mode(1, 6);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC1<Alternate<AF7>> {
                _set_alternate_mode(1, 7);
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC1 { _mode: PhantomData }
            }
        }
        impl PC1<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC1<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC1<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 1;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC1<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC1<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(1) })
            }
        }
        impl<MODE> OutputPin for PC1<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(1) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(1) })
            }
        }
        impl<MODE> toggleable::Default for PC1<Output<MODE>> {}
        impl InputPin for PC1<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(1) })
            }
        }
        impl<MODE> PC1<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC1<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(1) })
            }
        }
        /// Pin
        pub struct PC2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC2<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC2<Alternate<AF0>> {
                _set_alternate_mode(2, 0);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC2<Alternate<AF1>> {
                _set_alternate_mode(2, 1);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC2<Alternate<AF2>> {
                _set_alternate_mode(2, 2);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC2<Alternate<AF3>> {
                _set_alternate_mode(2, 3);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC2<Alternate<AF4>> {
                _set_alternate_mode(2, 4);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC2<Alternate<AF5>> {
                _set_alternate_mode(2, 5);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC2<Alternate<AF6>> {
                _set_alternate_mode(2, 6);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC2<Alternate<AF7>> {
                _set_alternate_mode(2, 7);
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC2 { _mode: PhantomData }
            }
        }
        impl PC2<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC2<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC2<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 2;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC2<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC2<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(2) })
            }
        }
        impl<MODE> OutputPin for PC2<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(2) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(2) })
            }
        }
        impl<MODE> toggleable::Default for PC2<Output<MODE>> {}
        impl InputPin for PC2<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(2) })
            }
        }
        impl<MODE> PC2<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC2<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(2) })
            }
        }
        /// Pin
        pub struct PC3<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC3<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC3<Alternate<AF0>> {
                _set_alternate_mode(3, 0);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC3<Alternate<AF1>> {
                _set_alternate_mode(3, 1);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC3<Alternate<AF2>> {
                _set_alternate_mode(3, 2);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC3<Alternate<AF3>> {
                _set_alternate_mode(3, 3);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC3<Alternate<AF4>> {
                _set_alternate_mode(3, 4);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC3<Alternate<AF5>> {
                _set_alternate_mode(3, 5);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC3<Alternate<AF6>> {
                _set_alternate_mode(3, 6);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC3<Alternate<AF7>> {
                _set_alternate_mode(3, 7);
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC3<Input<Floating>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC3<Input<PullDown>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC3<Input<PullUp>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC3<Analog> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC3<Output<OpenDrain>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC3<Output<PushPull>> {
                let offset = 2 * 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 3)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC3 { _mode: PhantomData }
            }
        }
        impl PC3<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC3<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 3;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC3<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 3;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC3<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC3<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(3) })
            }
        }
        impl<MODE> OutputPin for PC3<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(3) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(3) })
            }
        }
        impl<MODE> toggleable::Default for PC3<Output<MODE>> {}
        impl InputPin for PC3<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(3) })
            }
        }
        impl<MODE> PC3<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 3,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC3<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(3) })
            }
        }
        /// Pin
        pub struct PC4<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC4<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC4<Alternate<AF0>> {
                _set_alternate_mode(4, 0);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC4<Alternate<AF1>> {
                _set_alternate_mode(4, 1);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC4<Alternate<AF2>> {
                _set_alternate_mode(4, 2);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC4<Alternate<AF3>> {
                _set_alternate_mode(4, 3);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC4<Alternate<AF4>> {
                _set_alternate_mode(4, 4);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC4<Alternate<AF5>> {
                _set_alternate_mode(4, 5);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC4<Alternate<AF6>> {
                _set_alternate_mode(4, 6);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC4<Alternate<AF7>> {
                _set_alternate_mode(4, 7);
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC4<Input<Floating>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC4<Input<PullDown>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC4<Input<PullUp>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC4<Analog> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC4<Output<OpenDrain>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC4<Output<PushPull>> {
                let offset = 2 * 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 4)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC4 { _mode: PhantomData }
            }
        }
        impl PC4<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC4<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 4;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC4<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 4;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC4<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC4<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(4) })
            }
        }
        impl<MODE> OutputPin for PC4<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(4) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(4) })
            }
        }
        impl<MODE> toggleable::Default for PC4<Output<MODE>> {}
        impl InputPin for PC4<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(4) })
            }
        }
        impl<MODE> PC4<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 4,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC4<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(4) })
            }
        }
        /// Pin
        pub struct PC5<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC5<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC5<Alternate<AF0>> {
                _set_alternate_mode(5, 0);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC5<Alternate<AF1>> {
                _set_alternate_mode(5, 1);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC5<Alternate<AF2>> {
                _set_alternate_mode(5, 2);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC5<Alternate<AF3>> {
                _set_alternate_mode(5, 3);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC5<Alternate<AF4>> {
                _set_alternate_mode(5, 4);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC5<Alternate<AF5>> {
                _set_alternate_mode(5, 5);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC5<Alternate<AF6>> {
                _set_alternate_mode(5, 6);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC5<Alternate<AF7>> {
                _set_alternate_mode(5, 7);
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC5<Input<Floating>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC5<Input<PullDown>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC5<Input<PullUp>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC5<Analog> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC5<Output<OpenDrain>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC5<Output<PushPull>> {
                let offset = 2 * 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 5)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC5 { _mode: PhantomData }
            }
        }
        impl PC5<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC5<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 5;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC5<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 5;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC5<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC5<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(5) })
            }
        }
        impl<MODE> OutputPin for PC5<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(5) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(5) })
            }
        }
        impl<MODE> toggleable::Default for PC5<Output<MODE>> {}
        impl InputPin for PC5<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(5) })
            }
        }
        impl<MODE> PC5<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 5,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC5<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(5) })
            }
        }
        /// Pin
        pub struct PC6<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC6<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC6<Alternate<AF0>> {
                _set_alternate_mode(6, 0);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC6<Alternate<AF1>> {
                _set_alternate_mode(6, 1);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC6<Alternate<AF2>> {
                _set_alternate_mode(6, 2);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC6<Alternate<AF3>> {
                _set_alternate_mode(6, 3);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC6<Alternate<AF4>> {
                _set_alternate_mode(6, 4);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC6<Alternate<AF5>> {
                _set_alternate_mode(6, 5);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC6<Alternate<AF6>> {
                _set_alternate_mode(6, 6);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC6<Alternate<AF7>> {
                _set_alternate_mode(6, 7);
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC6<Input<Floating>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC6<Input<PullDown>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC6<Input<PullUp>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC6<Analog> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC6<Output<OpenDrain>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC6<Output<PushPull>> {
                let offset = 2 * 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 6)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC6 { _mode: PhantomData }
            }
        }
        impl PC6<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC6<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 6;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC6<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 6;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC6<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC6<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(6) })
            }
        }
        impl<MODE> OutputPin for PC6<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(6) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(6) })
            }
        }
        impl<MODE> toggleable::Default for PC6<Output<MODE>> {}
        impl InputPin for PC6<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(6) })
            }
        }
        impl<MODE> PC6<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 6,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC6<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(6) })
            }
        }
        /// Pin
        pub struct PC7<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC7<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC7<Alternate<AF0>> {
                _set_alternate_mode(7, 0);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC7<Alternate<AF1>> {
                _set_alternate_mode(7, 1);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC7<Alternate<AF2>> {
                _set_alternate_mode(7, 2);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC7<Alternate<AF3>> {
                _set_alternate_mode(7, 3);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC7<Alternate<AF4>> {
                _set_alternate_mode(7, 4);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC7<Alternate<AF5>> {
                _set_alternate_mode(7, 5);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC7<Alternate<AF6>> {
                _set_alternate_mode(7, 6);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC7<Alternate<AF7>> {
                _set_alternate_mode(7, 7);
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC7<Input<Floating>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC7<Input<PullDown>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC7<Input<PullUp>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC7<Analog> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC7<Output<OpenDrain>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC7<Output<PushPull>> {
                let offset = 2 * 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 7)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC7 { _mode: PhantomData }
            }
        }
        impl PC7<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC7<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 7;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC7<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 7;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC7<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC7<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(7) })
            }
        }
        impl<MODE> OutputPin for PC7<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(7) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(7) })
            }
        }
        impl<MODE> toggleable::Default for PC7<Output<MODE>> {}
        impl InputPin for PC7<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(7) })
            }
        }
        impl<MODE> PC7<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 7,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC7<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(7) })
            }
        }
        /// Pin
        pub struct PC8<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC8<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC8<Alternate<AF0>> {
                _set_alternate_mode(8, 0);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC8<Alternate<AF1>> {
                _set_alternate_mode(8, 1);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC8<Alternate<AF2>> {
                _set_alternate_mode(8, 2);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC8<Alternate<AF3>> {
                _set_alternate_mode(8, 3);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC8<Alternate<AF4>> {
                _set_alternate_mode(8, 4);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC8<Alternate<AF5>> {
                _set_alternate_mode(8, 5);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC8<Alternate<AF6>> {
                _set_alternate_mode(8, 6);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC8<Alternate<AF7>> {
                _set_alternate_mode(8, 7);
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC8<Input<Floating>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC8<Input<PullDown>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC8<Input<PullUp>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC8<Analog> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC8<Output<OpenDrain>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC8<Output<PushPull>> {
                let offset = 2 * 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 8)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC8 { _mode: PhantomData }
            }
        }
        impl PC8<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC8<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 8;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC8<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 8;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC8<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC8<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(8) })
            }
        }
        impl<MODE> OutputPin for PC8<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(8) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(8) })
            }
        }
        impl<MODE> toggleable::Default for PC8<Output<MODE>> {}
        impl InputPin for PC8<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(8) })
            }
        }
        impl<MODE> PC8<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 8,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC8<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(8) })
            }
        }
        /// Pin
        pub struct PC9<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC9<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC9<Alternate<AF0>> {
                _set_alternate_mode(9, 0);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC9<Alternate<AF1>> {
                _set_alternate_mode(9, 1);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC9<Alternate<AF2>> {
                _set_alternate_mode(9, 2);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC9<Alternate<AF3>> {
                _set_alternate_mode(9, 3);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC9<Alternate<AF4>> {
                _set_alternate_mode(9, 4);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC9<Alternate<AF5>> {
                _set_alternate_mode(9, 5);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC9<Alternate<AF6>> {
                _set_alternate_mode(9, 6);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC9<Alternate<AF7>> {
                _set_alternate_mode(9, 7);
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC9<Input<Floating>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC9<Input<PullDown>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC9<Input<PullUp>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC9<Analog> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC9<Output<OpenDrain>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC9<Output<PushPull>> {
                let offset = 2 * 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 9)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC9 { _mode: PhantomData }
            }
        }
        impl PC9<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC9<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 9;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC9<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 9;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC9<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC9<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(9) })
            }
        }
        impl<MODE> OutputPin for PC9<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(9) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(9) })
            }
        }
        impl<MODE> toggleable::Default for PC9<Output<MODE>> {}
        impl InputPin for PC9<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(9) })
            }
        }
        impl<MODE> PC9<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 9,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC9<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(9) })
            }
        }
        /// Pin
        pub struct PC10<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC10<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC10<Alternate<AF0>> {
                _set_alternate_mode(10, 0);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC10<Alternate<AF1>> {
                _set_alternate_mode(10, 1);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC10<Alternate<AF2>> {
                _set_alternate_mode(10, 2);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC10<Alternate<AF3>> {
                _set_alternate_mode(10, 3);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC10<Alternate<AF4>> {
                _set_alternate_mode(10, 4);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC10<Alternate<AF5>> {
                _set_alternate_mode(10, 5);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC10<Alternate<AF6>> {
                _set_alternate_mode(10, 6);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC10<Alternate<AF7>> {
                _set_alternate_mode(10, 7);
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC10<Input<Floating>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC10<Input<PullDown>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC10<Input<PullUp>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC10<Analog> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC10<Output<OpenDrain>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC10<Output<PushPull>> {
                let offset = 2 * 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 10)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC10 { _mode: PhantomData }
            }
        }
        impl PC10<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC10<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 10;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC10<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 10;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC10<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC10<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(10) })
            }
        }
        impl<MODE> OutputPin for PC10<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(10) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(10) })
            }
        }
        impl<MODE> toggleable::Default for PC10<Output<MODE>> {}
        impl InputPin for PC10<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(10) })
            }
        }
        impl<MODE> PC10<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 10,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC10<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(10) })
            }
        }
        /// Pin
        pub struct PC11<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC11<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC11<Alternate<AF0>> {
                _set_alternate_mode(11, 0);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC11<Alternate<AF1>> {
                _set_alternate_mode(11, 1);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC11<Alternate<AF2>> {
                _set_alternate_mode(11, 2);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC11<Alternate<AF3>> {
                _set_alternate_mode(11, 3);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC11<Alternate<AF4>> {
                _set_alternate_mode(11, 4);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC11<Alternate<AF5>> {
                _set_alternate_mode(11, 5);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC11<Alternate<AF6>> {
                _set_alternate_mode(11, 6);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC11<Alternate<AF7>> {
                _set_alternate_mode(11, 7);
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC11<Input<Floating>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC11<Input<PullDown>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC11<Input<PullUp>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC11<Analog> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC11<Output<OpenDrain>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC11<Output<PushPull>> {
                let offset = 2 * 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 11)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC11 { _mode: PhantomData }
            }
        }
        impl PC11<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC11<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 11;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC11<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 11;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC11<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC11<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(11) })
            }
        }
        impl<MODE> OutputPin for PC11<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(11) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(11) })
            }
        }
        impl<MODE> toggleable::Default for PC11<Output<MODE>> {}
        impl InputPin for PC11<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(11) })
            }
        }
        impl<MODE> PC11<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 11,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC11<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(11) })
            }
        }
        /// Pin
        pub struct PC12<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC12<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC12<Alternate<AF0>> {
                _set_alternate_mode(12, 0);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC12<Alternate<AF1>> {
                _set_alternate_mode(12, 1);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC12<Alternate<AF2>> {
                _set_alternate_mode(12, 2);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC12<Alternate<AF3>> {
                _set_alternate_mode(12, 3);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC12<Alternate<AF4>> {
                _set_alternate_mode(12, 4);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC12<Alternate<AF5>> {
                _set_alternate_mode(12, 5);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC12<Alternate<AF6>> {
                _set_alternate_mode(12, 6);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC12<Alternate<AF7>> {
                _set_alternate_mode(12, 7);
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC12<Input<Floating>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC12<Input<PullDown>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC12<Input<PullUp>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC12<Analog> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC12<Output<OpenDrain>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC12<Output<PushPull>> {
                let offset = 2 * 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 12)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC12 { _mode: PhantomData }
            }
        }
        impl PC12<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC12<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 12;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC12<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 12;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC12<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC12<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(12) })
            }
        }
        impl<MODE> OutputPin for PC12<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(12) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(12) })
            }
        }
        impl<MODE> toggleable::Default for PC12<Output<MODE>> {}
        impl InputPin for PC12<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(12) })
            }
        }
        impl<MODE> PC12<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 12,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC12<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(12) })
            }
        }
        /// Pin
        pub struct PC13<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC13<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC13<Alternate<AF0>> {
                _set_alternate_mode(13, 0);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC13<Alternate<AF1>> {
                _set_alternate_mode(13, 1);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC13<Alternate<AF2>> {
                _set_alternate_mode(13, 2);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC13<Alternate<AF3>> {
                _set_alternate_mode(13, 3);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC13<Alternate<AF4>> {
                _set_alternate_mode(13, 4);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC13<Alternate<AF5>> {
                _set_alternate_mode(13, 5);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC13<Alternate<AF6>> {
                _set_alternate_mode(13, 6);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC13<Alternate<AF7>> {
                _set_alternate_mode(13, 7);
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC13<Input<Floating>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC13<Input<PullDown>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC13<Input<PullUp>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC13<Analog> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC13<Output<OpenDrain>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC13<Output<PushPull>> {
                let offset = 2 * 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 13)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC13 { _mode: PhantomData }
            }
        }
        impl PC13<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC13<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 13;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC13<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 13;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC13<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC13<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(13) })
            }
        }
        impl<MODE> OutputPin for PC13<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(13) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(13) })
            }
        }
        impl<MODE> toggleable::Default for PC13<Output<MODE>> {}
        impl InputPin for PC13<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(13) })
            }
        }
        impl<MODE> PC13<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 13,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC13<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(13) })
            }
        }
        /// Pin
        pub struct PC14<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC14<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC14<Alternate<AF0>> {
                _set_alternate_mode(14, 0);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC14<Alternate<AF1>> {
                _set_alternate_mode(14, 1);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC14<Alternate<AF2>> {
                _set_alternate_mode(14, 2);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC14<Alternate<AF3>> {
                _set_alternate_mode(14, 3);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC14<Alternate<AF4>> {
                _set_alternate_mode(14, 4);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC14<Alternate<AF5>> {
                _set_alternate_mode(14, 5);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC14<Alternate<AF6>> {
                _set_alternate_mode(14, 6);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC14<Alternate<AF7>> {
                _set_alternate_mode(14, 7);
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC14<Input<Floating>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC14<Input<PullDown>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC14<Input<PullUp>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC14<Analog> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC14<Output<OpenDrain>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC14<Output<PushPull>> {
                let offset = 2 * 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 14)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC14 { _mode: PhantomData }
            }
        }
        impl PC14<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC14<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 14;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC14<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 14;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC14<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC14<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(14) })
            }
        }
        impl<MODE> OutputPin for PC14<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(14) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(14) })
            }
        }
        impl<MODE> toggleable::Default for PC14<Output<MODE>> {}
        impl InputPin for PC14<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(14) })
            }
        }
        impl<MODE> PC14<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 14,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC14<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(14) })
            }
        }
        /// Pin
        pub struct PC15<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PC15<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PC15<Alternate<AF0>> {
                _set_alternate_mode(15, 0);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PC15<Alternate<AF1>> {
                _set_alternate_mode(15, 1);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PC15<Alternate<AF2>> {
                _set_alternate_mode(15, 2);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PC15<Alternate<AF3>> {
                _set_alternate_mode(15, 3);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PC15<Alternate<AF4>> {
                _set_alternate_mode(15, 4);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PC15<Alternate<AF5>> {
                _set_alternate_mode(15, 5);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PC15<Alternate<AF6>> {
                _set_alternate_mode(15, 6);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PC15<Alternate<AF7>> {
                _set_alternate_mode(15, 7);
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PC15<Input<Floating>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PC15<Input<PullDown>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PC15<Input<PullUp>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PC15<Analog> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PC15<Output<OpenDrain>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PC15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PC15<Output<PushPull>> {
                let offset = 2 * 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 15)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PC15 { _mode: PhantomData }
            }
        }
        impl PC15<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PC15<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 15;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PC15<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 15;
                unsafe {
                    let reg = &(*GPIOC::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PC15<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PC15<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_set_low(15) })
            }
        }
        impl<MODE> OutputPin for PC15<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_high(15) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).set_low(15) })
            }
        }
        impl<MODE> toggleable::Default for PC15<Output<MODE>> {}
        impl InputPin for PC15<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(15) })
            }
        }
        impl<MODE> PC15<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 15,
                    port: GPIOC::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PC15<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOC::ptr()).is_low(15) })
            }
        }
    }
    /// GPIO
    #[cfg(any(
        feature = "stm32f030",
        feature = "stm32f051",
        feature = "stm32f058",
        feature = "stm32f070"
    ))]
    pub mod gpiod {
        use core::marker::PhantomData;
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
        use crate::{rcc::Rcc, pac::GPIOD};
        use cortex_m::interrupt::CriticalSection;
        use super::{
            Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, Pin, GpioRegExt,
        };
        /// GPIO parts
        pub struct Parts {
            /// Pin
            pub pd2: PD2<Input<Floating>>,
        }
        impl GpioExt for GPIOD {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.regs.ahbenr.modify(|_, w| w.iopden().set_bit());
                Parts {
                    pd2: PD2 { _mode: PhantomData },
                }
            }
        }
        fn _set_alternate_mode(index: usize, mode: u32) {
            let offset = 2 * index;
            let offset2 = 4 * index;
            unsafe {
                let reg = &(*GPIOD::ptr());
                if offset2 < 32 {
                    reg.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afrh.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                reg.moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
            }
        }
        /// Pin
        pub struct PD2<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PD2<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PD2<Alternate<AF0>> {
                _set_alternate_mode(2, 0);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PD2<Alternate<AF1>> {
                _set_alternate_mode(2, 1);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PD2<Alternate<AF2>> {
                _set_alternate_mode(2, 2);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PD2<Alternate<AF3>> {
                _set_alternate_mode(2, 3);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PD2<Alternate<AF4>> {
                _set_alternate_mode(2, 4);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PD2<Alternate<AF5>> {
                _set_alternate_mode(2, 5);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PD2<Alternate<AF6>> {
                _set_alternate_mode(2, 6);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PD2<Alternate<AF7>> {
                _set_alternate_mode(2, 7);
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PD2<Input<Floating>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PD2<Input<PullDown>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PD2<Input<PullUp>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PD2<Analog> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PD2<Output<OpenDrain>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PD2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PD2<Output<PushPull>> {
                let offset = 2 * 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 2)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PD2 { _mode: PhantomData }
            }
        }
        impl PD2<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PD2<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 2;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PD2<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 2;
                unsafe {
                    let reg = &(*GPIOD::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PD2<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOD::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PD2<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOD::ptr()).is_set_low(2) })
            }
        }
        impl<MODE> OutputPin for PD2<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOD::ptr()).set_high(2) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOD::ptr()).set_low(2) })
            }
        }
        impl<MODE> toggleable::Default for PD2<Output<MODE>> {}
        impl InputPin for PD2<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOD::ptr()).is_low(2) })
            }
        }
        impl<MODE> PD2<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 2,
                    port: GPIOD::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PD2<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOD::ptr()).is_low(2) })
            }
        }
    }
    /// GPIO
    #[cfg(any(feature = "stm32f030xc", feature = "stm32f070"))]
    pub mod gpiof {
        use core::marker::PhantomData;
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, toggleable};
        use crate::{rcc::Rcc, pac::GPIOF};
        use cortex_m::interrupt::CriticalSection;
        use super::{
            Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output, PullDown, PullUp,
            PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, Pin, GpioRegExt,
        };
        /// GPIO parts
        pub struct Parts {
            /// Pin
            pub pf0: PF0<Input<Floating>>,
            /// Pin
            pub pf1: PF1<Input<Floating>>,
        }
        impl GpioExt for GPIOF {
            type Parts = Parts;
            fn split(self, rcc: &mut Rcc) -> Parts {
                rcc.regs.ahbenr.modify(|_, w| w.iopfen().set_bit());
                Parts {
                    pf0: PF0 { _mode: PhantomData },
                    pf1: PF1 { _mode: PhantomData },
                }
            }
        }
        fn _set_alternate_mode(index: usize, mode: u32) {
            let offset = 2 * index;
            let offset2 = 4 * index;
            unsafe {
                let reg = &(*GPIOF::ptr());
                if offset2 < 32 {
                    reg.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                } else {
                    let offset2 = offset2 - 32;
                    reg.afrh.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                    });
                }
                reg.moder
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
            }
        }
        /// Pin
        pub struct PF0<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PF0<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PF0<Alternate<AF0>> {
                _set_alternate_mode(0, 0);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PF0<Alternate<AF1>> {
                _set_alternate_mode(0, 1);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PF0<Alternate<AF2>> {
                _set_alternate_mode(0, 2);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PF0<Alternate<AF3>> {
                _set_alternate_mode(0, 3);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PF0<Alternate<AF4>> {
                _set_alternate_mode(0, 4);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PF0<Alternate<AF5>> {
                _set_alternate_mode(0, 5);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PF0<Alternate<AF6>> {
                _set_alternate_mode(0, 6);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PF0<Alternate<AF7>> {
                _set_alternate_mode(0, 7);
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PF0<Input<Floating>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PF0<Input<PullDown>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PF0<Input<PullUp>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PF0<Analog> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PF0<Output<OpenDrain>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PF0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PF0<Output<PushPull>> {
                let offset = 2 * 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 0)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF0 { _mode: PhantomData }
            }
        }
        impl PF0<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PF0<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 0;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PF0<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 0;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PF0<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOF::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PF0<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_set_low(0) })
            }
        }
        impl<MODE> OutputPin for PF0<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).set_high(0) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).set_low(0) })
            }
        }
        impl<MODE> toggleable::Default for PF0<Output<MODE>> {}
        impl InputPin for PF0<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_low(0) })
            }
        }
        impl<MODE> PF0<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 0,
                    port: GPIOF::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PF0<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_low(0) })
            }
        }
        /// Pin
        pub struct PF1<MODE> {
            _mode: PhantomData<MODE>,
        }
        impl<MODE> PF1<MODE> {
            /// Configures the pin to operate in AF0 mode
            pub fn into_alternate_af0(self, _cs: &CriticalSection) -> PF1<Alternate<AF0>> {
                _set_alternate_mode(1, 0);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF1 mode
            pub fn into_alternate_af1(self, _cs: &CriticalSection) -> PF1<Alternate<AF1>> {
                _set_alternate_mode(1, 1);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF2 mode
            pub fn into_alternate_af2(self, _cs: &CriticalSection) -> PF1<Alternate<AF2>> {
                _set_alternate_mode(1, 2);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF3 mode
            pub fn into_alternate_af3(self, _cs: &CriticalSection) -> PF1<Alternate<AF3>> {
                _set_alternate_mode(1, 3);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF4 mode
            pub fn into_alternate_af4(self, _cs: &CriticalSection) -> PF1<Alternate<AF4>> {
                _set_alternate_mode(1, 4);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF5 mode
            pub fn into_alternate_af5(self, _cs: &CriticalSection) -> PF1<Alternate<AF5>> {
                _set_alternate_mode(1, 5);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF6 mode
            pub fn into_alternate_af6(self, _cs: &CriticalSection) -> PF1<Alternate<AF6>> {
                _set_alternate_mode(1, 6);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate in AF7 mode
            pub fn into_alternate_af7(self, _cs: &CriticalSection) -> PF1<Alternate<AF7>> {
                _set_alternate_mode(1, 7);
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a floating input pin
            pub fn into_floating_input(self, _cs: &CriticalSection) -> PF1<Input<Floating>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled down input pin
            pub fn into_pull_down_input(self, _cs: &CriticalSection) -> PF1<Input<PullDown>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as a pulled up input pin
            pub fn into_pull_up_input(self, _cs: &CriticalSection) -> PF1<Input<PullUp>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an analog pin
            pub fn into_analog(self, _cs: &CriticalSection) -> PF1<Analog> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an open drain output pin
            pub fn into_open_drain_output(self, _cs: &CriticalSection) -> PF1<Output<OpenDrain>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin
            pub fn into_push_pull_output(self, _cs: &CriticalSection) -> PF1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
            /// Configures the pin to operate as an push pull output pin with quick fall
            /// and rise times
            pub fn into_push_pull_output_hs(self, _cs: &CriticalSection) -> PF1<Output<PushPull>> {
                let offset = 2 * 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset)));
                    reg.otyper.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.ospeedr.modify(|r, w| w.bits(r.bits() & !(0b1 << 1)));
                    reg.moder
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset)));
                }
                PF1 { _mode: PhantomData }
            }
        }
        impl PF1<Output<OpenDrain>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(&mut self, _cs: &CriticalSection, on: bool) {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
            }
        }
        impl<AF> PF1<Alternate<AF>> {
            /// Enables / disables the internal pull up
            pub fn internal_pull_up(self, _cs: &CriticalSection, on: bool) -> Self {
                let offset = 2 * 1;
                let value = if on { 0b01 } else { 0b00 };
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.pupdr
                        .modify(|r, w| w.bits((r.bits() & !(0b11 << offset)) | (value << offset)));
                }
                self
            }
        }
        impl<AF> PF1<Alternate<AF>> {
            /// Turns pin alternate configuration pin into open drain
            pub fn set_open_drain(self, _cs: &CriticalSection) -> Self {
                let offset = 1;
                unsafe {
                    let reg = &(*GPIOF::ptr());
                    reg.otyper.modify(|r, w| w.bits(r.bits() | (1 << offset)));
                }
                self
            }
        }
        impl<MODE> PF1<Output<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Output<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOF::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> StatefulOutputPin for PF1<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                self.is_set_low().map(|v| !v)
            }
            fn is_set_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_set_low(1) })
            }
        }
        impl<MODE> OutputPin for PF1<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).set_high(1) })
            }
            fn set_low(&mut self) -> Result<(), Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).set_low(1) })
            }
        }
        impl<MODE> toggleable::Default for PF1<Output<MODE>> {}
        impl InputPin for PF1<Output<OpenDrain>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_low(1) })
            }
        }
        impl<MODE> PF1<Input<MODE>> {
            /// Erases the pin number from the type
            ///
            /// This is useful when you want to collect the pins into an array where you
            /// need all the elements to have the same type
            pub fn downgrade(self) -> Pin<Input<MODE>> {
                Pin {
                    i: 1,
                    port: GPIOF::ptr() as *const dyn GpioRegExt,
                    _mode: self._mode,
                }
            }
        }
        impl<MODE> InputPin for PF1<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Self::Error> {
                self.is_low().map(|v| !v)
            }
            fn is_low(&self) -> Result<bool, Self::Error> {
                Ok(unsafe { (*GPIOF::ptr()).is_low(1) })
            }
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod i2c {
    use core::ops::Deref;
    use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
    use crate::{
        gpio::*,
        rcc::Rcc,
        time::{KiloHertz, U32Ext},
    };
    /// I2C abstraction
    pub struct I2c<I2C, SCLPIN, SDAPIN> {
        i2c: I2C,
        pins: (SCLPIN, SDAPIN),
    }
    pub trait SclPin<I2C> {}
    pub trait SdaPin<I2C> {}
    impl SclPin<crate::pac::I2C1> for gpiob::PB6<Alternate<AF1>> {}
    impl SclPin<crate::pac::I2C1> for gpiob::PB8<Alternate<AF1>> {}
    impl SdaPin<crate::pac::I2C1> for gpiob::PB7<Alternate<AF1>> {}
    impl SdaPin<crate::pac::I2C1> for gpiob::PB9<Alternate<AF1>> {}
    pub enum Error {
        OVERRUN,
        NACK,
        BUS,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::OVERRUN,) => ::core::fmt::Formatter::write_str(f, "OVERRUN"),
                (&Error::NACK,) => ::core::fmt::Formatter::write_str(f, "NACK"),
                (&Error::BUS,) => ::core::fmt::Formatter::write_str(f, "BUS"),
            }
        }
    }
    use crate::pac::I2C1;
    impl<SCLPIN, SDAPIN> I2c<I2C1, SCLPIN, SDAPIN> {
        pub fn i2c1(i2c: I2C1, pins: (SCLPIN, SDAPIN), speed: KiloHertz, rcc: &mut Rcc) -> Self
        where
            SCLPIN: SclPin<I2C1>,
            SDAPIN: SdaPin<I2C1>,
        {
            rcc.regs.apb1enr.modify(|_, w| w.i2c1en().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());
            I2c { i2c, pins }.i2c_init(speed)
        }
    }
    #[allow(dead_code)]
    type I2cRegisterBlock = crate::pac::i2c1::RegisterBlock;
    impl<I2C, SCLPIN, SDAPIN> I2c<I2C, SCLPIN, SDAPIN>
    where
        I2C: Deref<Target = I2cRegisterBlock>,
    {
        fn i2c_init(self, speed: KiloHertz) -> Self {
            use core::cmp;
            self.i2c.cr1.modify(|_, w| w.pe().clear_bit());
            let presc;
            let scldel;
            let sdadel;
            let sclh;
            let scll;
            const FREQ: u32 = 8_000_000;
            if speed <= 100_u32.khz() {
                presc = 1;
                scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
                sclh = scll - 4;
                sdadel = 2;
                scldel = 4;
            } else {
                presc = 0;
                scll = cmp::max((((FREQ >> presc) >> 1) / speed.0) - 1, 255) as u8;
                sclh = scll - 6;
                sdadel = 1;
                scldel = 3;
            }
            self.i2c.timingr.write(|w| {
                w.presc()
                    .bits(presc)
                    .scldel()
                    .bits(scldel)
                    .sdadel()
                    .bits(sdadel)
                    .sclh()
                    .bits(sclh)
                    .scll()
                    .bits(scll)
            });
            self.i2c.cr1.modify(|_, w| w.pe().set_bit());
            self
        }
        pub fn release(self) -> (I2C, (SCLPIN, SDAPIN)) {
            (self.i2c, self.pins)
        }
        fn check_and_clear_error_flags(
            &self,
            isr: &crate::stm32::i2c1::isr::R,
        ) -> Result<(), Error> {
            if isr.ovr().bit_is_set() {
                self.i2c.icr.write(|w| w.ovrcf().set_bit());
                return Err(Error::OVERRUN);
            }
            if isr.arlo().bit_is_set() | isr.berr().bit_is_set() {
                self.i2c
                    .icr
                    .write(|w| w.arlocf().set_bit().berrcf().set_bit());
                return Err(Error::BUS);
            }
            if isr.nackf().bit_is_set() {
                self.i2c
                    .icr
                    .write(|w| w.stopcf().set_bit().nackcf().set_bit());
                return Err(Error::NACK);
            }
            Ok(())
        }
        fn send_byte(&self, byte: u8) -> Result<(), Error> {
            loop {
                let isr = self.i2c.isr.read();
                self.check_and_clear_error_flags(&isr)?;
                if isr.txis().bit_is_set() {
                    break;
                }
            }
            self.i2c.txdr.write(|w| unsafe { w.bits(u32::from(byte)) });
            self.check_and_clear_error_flags(&self.i2c.isr.read())?;
            Ok(())
        }
        fn recv_byte(&self) -> Result<u8, Error> {
            loop {
                let isr = self.i2c.isr.read();
                self.check_and_clear_error_flags(&isr)?;
                if isr.rxne().bit_is_set() {
                    break;
                }
            }
            let value = self.i2c.rxdr.read().bits() as u8;
            Ok(value)
        }
    }
    impl<I2C, SCLPIN, SDAPIN> WriteRead for I2c<I2C, SCLPIN, SDAPIN>
    where
        I2C: Deref<Target = I2cRegisterBlock>,
    {
        type Error = Error;
        fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
            self.i2c.cr2.modify(|_, w| {
                w.sadd()
                    .bits(u16::from(addr) << 1)
                    .nbytes()
                    .bits(bytes.len() as u8)
                    .rd_wrn()
                    .clear_bit()
                    .autoend()
                    .clear_bit()
            });
            self.i2c.cr2.modify(|_, w| w.start().set_bit());
            loop {
                let isr = self.i2c.isr.read();
                self.check_and_clear_error_flags(&isr)?;
                if isr.txis().bit_is_set() || isr.tc().bit_is_set() {
                    break;
                }
            }
            for c in bytes {
                self.send_byte(*c)?;
            }
            loop {
                let isr = self.i2c.isr.read();
                self.check_and_clear_error_flags(&isr)?;
                if isr.tc().bit_is_set() {
                    break;
                }
            }
            self.i2c.cr2.modify(|_, w| {
                w.sadd()
                    .bits(u16::from(addr) << 1)
                    .nbytes()
                    .bits(buffer.len() as u8)
                    .rd_wrn()
                    .set_bit()
            });
            self.i2c.cr2.modify(|_, w| w.start().set_bit());
            self.i2c.cr2.modify(|_, w| w.autoend().set_bit());
            for c in buffer.iter_mut() {
                *c = self.recv_byte()?;
            }
            self.check_and_clear_error_flags(&self.i2c.isr.read())?;
            Ok(())
        }
    }
    impl<I2C, SCLPIN, SDAPIN> Read for I2c<I2C, SCLPIN, SDAPIN>
    where
        I2C: Deref<Target = I2cRegisterBlock>,
    {
        type Error = Error;
        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
            self.i2c.cr2.modify(|_, w| {
                w.sadd()
                    .bits(u16::from(addr) << 1)
                    .nbytes()
                    .bits(buffer.len() as u8)
                    .rd_wrn()
                    .set_bit()
            });
            self.i2c.cr2.modify(|_, w| w.start().set_bit());
            self.i2c.cr2.modify(|_, w| w.autoend().set_bit());
            for c in buffer.iter_mut() {
                *c = self.recv_byte()?;
            }
            self.check_and_clear_error_flags(&self.i2c.isr.read())?;
            Ok(())
        }
    }
    impl<I2C, SCLPIN, SDAPIN> Write for I2c<I2C, SCLPIN, SDAPIN>
    where
        I2C: Deref<Target = I2cRegisterBlock>,
    {
        type Error = Error;
        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
            self.i2c.cr2.modify(|_, w| {
                w.sadd()
                    .bits(u16::from(addr) << 1)
                    .nbytes()
                    .bits(bytes.len() as u8)
                    .rd_wrn()
                    .clear_bit()
                    .autoend()
                    .set_bit()
            });
            self.i2c.cr2.modify(|_, w| w.start().set_bit());
            for c in bytes {
                self.send_byte(*c)?;
            }
            self.check_and_clear_error_flags(&self.i2c.isr.read())?;
            Ok(())
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod prelude {
    pub use embedded_hal::prelude::*;
    pub use embedded_hal::watchdog::Watchdog as _stm32f0xx_hal_embedded_hal_watchdog_Watchdog;
    pub use embedded_hal::watchdog::WatchdogEnable as _stm32f0xx_hal_embedded_hal_watchdog_WatchdogEnable;
    pub use embedded_hal::adc::OneShot as _embedded_hal_adc_OneShot;
    pub use embedded_hal::digital::v2::InputPin as _embedded_hal_gpio_InputPin;
    pub use embedded_hal::digital::v2::OutputPin as _embedded_hal_gpio_OutputPin;
    pub use embedded_hal::digital::v2::StatefulOutputPin as _embedded_hal_gpio_StatefulOutputPin;
    pub use embedded_hal::digital::v2::ToggleableOutputPin as _embedded_hal_gpio_ToggleableOutputPin;
    pub use crate::gpio::GpioExt as _stm32f0xx_hal_gpio_GpioExt;
    pub use crate::rcc::RccExt as _stm32f0xx_hal_rcc_RccExt;
    pub use crate::time::U32Ext as _stm32f0xx_hal_time_U32Ext;
}
#[cfg(feature = "device-selected")]
pub mod pwm {
    use cast::{u16, u32};
    use core::{marker::PhantomData, mem::MaybeUninit};
    use crate::rcc::Rcc;
    use crate::time::Hertz;
    use embedded_hal as hal;
    pub trait Pins<TIM, P> {
        const C1: bool = false;
        const C2: bool = false;
        const C3: bool = false;
        const C4: bool = false;
        type Channels;
    }
    use crate::timers::PinC1;
    use crate::timers::PinC2;
    use crate::timers::PinC3;
    use crate::timers::PinC4;
    pub struct C1;
    pub struct C2;
    pub struct C3;
    pub struct C4;
    pub struct PwmChannels<TIM, CHANNELS> {
        _channel: PhantomData<CHANNELS>,
        _tim: PhantomData<TIM>,
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P2, P3, P4> Pins<TIM, (C1, C2, C3, C4)> for (P1, P2, P3, P4)
    where
        P1: PinC1<TIM>,
        P2: PinC2<TIM>,
        P3: PinC3<TIM>,
        P4: PinC4<TIM>,
    {
        const C1: bool = true;
        const C2: bool = true;
        const C3: bool = true;
        const C4: bool = true;
        type Channels = (
            PwmChannels<TIM, C1>,
            PwmChannels<TIM, C2>,
            PwmChannels<TIM, C3>,
            PwmChannels<TIM, C4>,
        );
    }
    #[allow(unused_parens)]
    impl<TIM, P2, P3, P4> Pins<TIM, (C2, C3, C4)> for (P2, P3, P4)
    where
        P2: PinC2<TIM>,
        P3: PinC3<TIM>,
        P4: PinC4<TIM>,
    {
        const C2: bool = true;
        const C3: bool = true;
        const C4: bool = true;
        type Channels = (
            PwmChannels<TIM, C2>,
            PwmChannels<TIM, C3>,
            PwmChannels<TIM, C4>,
        );
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P3, P4> Pins<TIM, (C1, C3, C4)> for (P1, P3, P4)
    where
        P1: PinC1<TIM>,
        P3: PinC3<TIM>,
        P4: PinC4<TIM>,
    {
        const C1: bool = true;
        const C3: bool = true;
        const C4: bool = true;
        type Channels = (
            PwmChannels<TIM, C1>,
            PwmChannels<TIM, C3>,
            PwmChannels<TIM, C4>,
        );
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P2, P4> Pins<TIM, (C1, C2, C4)> for (P1, P2, P4)
    where
        P1: PinC1<TIM>,
        P2: PinC2<TIM>,
        P4: PinC4<TIM>,
    {
        const C1: bool = true;
        const C2: bool = true;
        const C4: bool = true;
        type Channels = (
            PwmChannels<TIM, C1>,
            PwmChannels<TIM, C2>,
            PwmChannels<TIM, C4>,
        );
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P2, P3> Pins<TIM, (C1, C2, C3)> for (P1, P2, P3)
    where
        P1: PinC1<TIM>,
        P2: PinC2<TIM>,
        P3: PinC3<TIM>,
    {
        const C1: bool = true;
        const C2: bool = true;
        const C3: bool = true;
        type Channels = (
            PwmChannels<TIM, C1>,
            PwmChannels<TIM, C2>,
            PwmChannels<TIM, C3>,
        );
    }
    #[allow(unused_parens)]
    impl<TIM, P3, P4> Pins<TIM, (C3, C4)> for (P3, P4)
    where
        P3: PinC3<TIM>,
        P4: PinC4<TIM>,
    {
        const C3: bool = true;
        const C4: bool = true;
        type Channels = (PwmChannels<TIM, C3>, PwmChannels<TIM, C4>);
    }
    #[allow(unused_parens)]
    impl<TIM, P2, P4> Pins<TIM, (C2, C4)> for (P2, P4)
    where
        P2: PinC2<TIM>,
        P4: PinC4<TIM>,
    {
        const C2: bool = true;
        const C4: bool = true;
        type Channels = (PwmChannels<TIM, C2>, PwmChannels<TIM, C4>);
    }
    #[allow(unused_parens)]
    impl<TIM, P2, P3> Pins<TIM, (C2, C3)> for (P2, P3)
    where
        P2: PinC2<TIM>,
        P3: PinC3<TIM>,
    {
        const C2: bool = true;
        const C3: bool = true;
        type Channels = (PwmChannels<TIM, C2>, PwmChannels<TIM, C3>);
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P4> Pins<TIM, (C1, C4)> for (P1, P4)
    where
        P1: PinC1<TIM>,
        P4: PinC4<TIM>,
    {
        const C1: bool = true;
        const C4: bool = true;
        type Channels = (PwmChannels<TIM, C1>, PwmChannels<TIM, C4>);
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P3> Pins<TIM, (C1, C3)> for (P1, P3)
    where
        P1: PinC1<TIM>,
        P3: PinC3<TIM>,
    {
        const C1: bool = true;
        const C3: bool = true;
        type Channels = (PwmChannels<TIM, C1>, PwmChannels<TIM, C3>);
    }
    #[allow(unused_parens)]
    impl<TIM, P1, P2> Pins<TIM, (C1, C2)> for (P1, P2)
    where
        P1: PinC1<TIM>,
        P2: PinC2<TIM>,
    {
        const C1: bool = true;
        const C2: bool = true;
        type Channels = (PwmChannels<TIM, C1>, PwmChannels<TIM, C2>);
    }
    #[allow(unused_parens)]
    impl<TIM, P1> Pins<TIM, (C1)> for (P1)
    where
        P1: PinC1<TIM>,
    {
        const C1: bool = true;
        type Channels = (PwmChannels<TIM, C1>);
    }
    #[allow(unused_parens)]
    impl<TIM, P2> Pins<TIM, (C2)> for (P2)
    where
        P2: PinC2<TIM>,
    {
        const C2: bool = true;
        type Channels = (PwmChannels<TIM, C2>);
    }
    #[allow(unused_parens)]
    impl<TIM, P3> Pins<TIM, (C3)> for (P3)
    where
        P3: PinC3<TIM>,
    {
        const C3: bool = true;
        type Channels = (PwmChannels<TIM, C3>);
    }
    #[allow(unused_parens)]
    impl<TIM, P4> Pins<TIM, (C4)> for (P4)
    where
        P4: PinC4<TIM>,
    {
        const C4: bool = true;
        type Channels = (PwmChannels<TIM, C4>);
    }
    impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>> PinC1<TIM> for (P1, P2) {}
    impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>> PinC2<TIM> for (P1, P2) {}
    impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>> PinC3<TIM> for (P1, P2) {}
    impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>> PinC4<TIM> for (P1, P2) {}
    impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>, P3: PinC1<TIM>> PinC1<TIM> for (P1, P2, P3) {}
    impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>, P3: PinC2<TIM>> PinC2<TIM> for (P1, P2, P3) {}
    impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>, P3: PinC3<TIM>> PinC3<TIM> for (P1, P2, P3) {}
    impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>, P3: PinC4<TIM>> PinC4<TIM> for (P1, P2, P3) {}
    impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>, P3: PinC1<TIM>, P4: PinC1<TIM>> PinC1<TIM>
        for (P1, P2, P3, P4)
    {
    }
    impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>, P3: PinC2<TIM>, P4: PinC2<TIM>> PinC2<TIM>
        for (P1, P2, P3, P4)
    {
    }
    impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>, P3: PinC3<TIM>, P4: PinC3<TIM>> PinC3<TIM>
        for (P1, P2, P3, P4)
    {
    }
    impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>, P3: PinC4<TIM>, P4: PinC4<TIM>> PinC4<TIM>
        for (P1, P2, P3, P4)
    {
    }
    use crate::pac::*;
    pub fn tim1<P, PINS, T>(tim: TIM1, _pins: PINS, rcc: &mut Rcc, freq: T) -> PINS::Channels
    where
        PINS: Pins<TIM1, P>,
        T: Into<Hertz>,
    {
        rcc.regs.apb2enr.modify(|_, w| w.tim1en().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());
        if PINS::C1 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1());
        }
        if PINS::C2 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1());
        }
        if PINS::C3 {
            tim.ccmr2_output()
                .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1());
        }
        if PINS::C4 {
            tim.ccmr2_output()
                .modify(|_, w| w.oc4pe().set_bit().oc4m().pwm_mode1());
        }
        let tclk = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };
        let ticks = tclk / freq.into().0;
        let psc = u16((ticks - 1) / (1 << 16)).unwrap();
        tim.psc.write(|w| w.psc().bits(psc));
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
        tim.cr1.modify(|_, w| w.arpe().set_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.egr.write(|w| w.ug().set_bit());
        tim.cr1.modify(|_, w| w.urs().clear_bit());
        tim.bdtr.modify(|_, w| w.aoe().set_bit());
        tim.cr1.write(|w| {
            w.cms()
                .bits(0b00)
                .dir()
                .clear_bit()
                .opm()
                .clear_bit()
                .cen()
                .set_bit()
        });
        unsafe { MaybeUninit::uninit().assume_init() }
    }
    impl hal::PwmPin for PwmChannels<TIM1, C1> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc1e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc1e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).ccr1.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM1::ptr()).ccr1.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM1, C2> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc2e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc2e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).ccr2.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM1::ptr()).ccr2.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM1, C3> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc3e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc3e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).ccr3.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM1::ptr()).ccr3.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM1, C4> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc4e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM1::ptr())).ccer.modify(|_, w| w.cc4e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).ccr4.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM1::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM1::ptr()).ccr4.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    pub fn tim3<P, PINS, T>(tim: TIM3, _pins: PINS, rcc: &mut Rcc, freq: T) -> PINS::Channels
    where
        PINS: Pins<TIM3, P>,
        T: Into<Hertz>,
    {
        rcc.regs.apb1enr.modify(|_, w| w.tim3en().set_bit());
        rcc.regs.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
        rcc.regs.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
        if PINS::C1 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().pwm_mode1());
        }
        if PINS::C2 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc2pe().set_bit().oc2m().pwm_mode1());
        }
        if PINS::C3 {
            tim.ccmr2_output()
                .modify(|_, w| w.oc3pe().set_bit().oc3m().pwm_mode1());
        }
        if PINS::C4 {
            tim.ccmr2_output()
                .modify(|_, w| w.oc4pe().set_bit().oc4m().pwm_mode1());
        }
        let tclk = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };
        let ticks = tclk / freq.into().0;
        let psc = u16((ticks - 1) / (1 << 16)).unwrap();
        tim.psc.write(|w| w.psc().bits(psc));
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
        tim.cr1.modify(|_, w| w.arpe().set_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.egr.write(|w| w.ug().set_bit());
        tim.cr1.modify(|_, w| w.urs().clear_bit());
        tim.cr1.write(|w| {
            w.cms()
                .bits(0b00)
                .dir()
                .clear_bit()
                .opm()
                .clear_bit()
                .cen()
                .set_bit()
        });
        unsafe { MaybeUninit::uninit().assume_init() }
    }
    impl hal::PwmPin for PwmChannels<TIM3, C1> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc1e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc1e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).ccr1.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM3::ptr()).ccr1.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM3, C2> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc2e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc2e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).ccr2.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM3::ptr()).ccr2.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM3, C3> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc3e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc3e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).ccr3.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM3::ptr()).ccr3.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    impl hal::PwmPin for PwmChannels<TIM3, C4> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc4e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM3::ptr())).ccer.modify(|_, w| w.cc4e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).ccr4.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM3::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM3::ptr()).ccr4.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    pub fn tim14<P, PINS, T>(tim: TIM14, _pins: PINS, rcc: &mut Rcc, freq: T) -> PINS::Channels
    where
        PINS: Pins<TIM14, P>,
        T: Into<Hertz>,
    {
        rcc.regs.apb1enr.modify(|_, w| w.tim14en().set_bit());
        rcc.regs.apb1rstr.modify(|_, w| w.tim14rst().set_bit());
        rcc.regs.apb1rstr.modify(|_, w| w.tim14rst().clear_bit());
        if PINS::C1 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
        }
        let tclk = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };
        let ticks = tclk / freq.into().0;
        let psc = u16((ticks - 1) / (1 << 16)).unwrap();
        tim.psc.write(|w| w.psc().bits(psc));
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
        tim.cr1.modify(|_, w| w.arpe().set_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.egr.write(|w| w.ug().set_bit());
        tim.cr1.modify(|_, w| w.urs().clear_bit());
        tim.cr1.write(|w| w.cen().set_bit());
        unsafe { MaybeUninit::uninit().assume_init() }
    }
    impl hal::PwmPin for PwmChannels<TIM14, C1> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM14::ptr())).ccer.modify(|_, w| w.cc1e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM14::ptr())).ccer.modify(|_, w| w.cc1e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM14::ptr()).ccr1.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM14::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM14::ptr()).ccr1.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    pub fn tim16<P, PINS, T>(tim: TIM16, _pins: PINS, rcc: &mut Rcc, freq: T) -> PINS::Channels
    where
        PINS: Pins<TIM16, P>,
        T: Into<Hertz>,
    {
        rcc.regs.apb2enr.modify(|_, w| w.tim16en().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim16rst().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim16rst().clear_bit());
        if PINS::C1 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
        }
        let tclk = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };
        let ticks = tclk / freq.into().0;
        let psc = u16((ticks - 1) / (1 << 16)).unwrap();
        tim.psc.write(|w| w.psc().bits(psc));
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
        tim.cr1.modify(|_, w| w.arpe().set_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.egr.write(|w| w.ug().set_bit());
        tim.cr1.modify(|_, w| w.urs().clear_bit());
        tim.bdtr.modify(|_, w| w.aoe().set_bit());
        tim.cr1.write(|w| w.opm().clear_bit().cen().set_bit());
        unsafe { MaybeUninit::uninit().assume_init() }
    }
    impl hal::PwmPin for PwmChannels<TIM16, C1> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM16::ptr())).ccer.modify(|_, w| w.cc1e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM16::ptr())).ccer.modify(|_, w| w.cc1e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM16::ptr()).ccr1.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM16::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM16::ptr()).ccr1.write(|w| w.ccr().bits(duty.into())) }
        }
    }
    pub fn tim17<P, PINS, T>(tim: TIM17, _pins: PINS, rcc: &mut Rcc, freq: T) -> PINS::Channels
    where
        PINS: Pins<TIM17, P>,
        T: Into<Hertz>,
    {
        rcc.regs.apb2enr.modify(|_, w| w.tim17en().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim17rst().set_bit());
        rcc.regs.apb2rstr.modify(|_, w| w.tim17rst().clear_bit());
        if PINS::C1 {
            tim.ccmr1_output()
                .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(6));
        }
        let tclk = if rcc.clocks.hclk().0 == rcc.clocks.pclk().0 {
            rcc.clocks.pclk().0
        } else {
            rcc.clocks.pclk().0 * 2
        };
        let ticks = tclk / freq.into().0;
        let psc = u16((ticks - 1) / (1 << 16)).unwrap();
        tim.psc.write(|w| w.psc().bits(psc));
        let arr = u16(ticks / u32(psc + 1)).unwrap();
        tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
        tim.cr1.modify(|_, w| w.arpe().set_bit());
        tim.cr1.modify(|_, w| w.urs().set_bit());
        tim.egr.write(|w| w.ug().set_bit());
        tim.cr1.modify(|_, w| w.urs().clear_bit());
        tim.bdtr.modify(|_, w| w.aoe().set_bit());
        tim.cr1.write(|w| w.opm().clear_bit().cen().set_bit());
        unsafe { MaybeUninit::uninit().assume_init() }
    }
    impl hal::PwmPin for PwmChannels<TIM17, C1> {
        type Duty = u16;
        fn disable(&mut self) {
            unsafe { (*(TIM17::ptr())).ccer.modify(|_, w| w.cc1e().clear_bit()) };
        }
        fn enable(&mut self) {
            unsafe { (*(TIM17::ptr())).ccer.modify(|_, w| w.cc1e().set_bit()) };
        }
        fn get_duty(&self) -> u16 {
            unsafe { (*TIM17::ptr()).ccr1.read().ccr().bits() as u16 }
        }
        fn get_max_duty(&self) -> u16 {
            unsafe { (*TIM17::ptr()).arr.read().arr().bits() as u16 }
        }
        fn set_duty(&mut self, duty: u16) {
            unsafe { (*TIM17::ptr()).ccr1.write(|w| w.ccr().bits(duty.into())) }
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod rcc {
    use crate::pac::RCC;
    use crate::time::Hertz;
    /// Extension trait that sets up the `RCC` peripheral
    pub trait RccExt {
        /// Configure the clocks of the RCC peripheral
        fn configure(self) -> CFGR;
    }
    impl RccExt for RCC {
        fn configure(self) -> CFGR {
            CFGR {
                hclk: None,
                pclk: None,
                sysclk: None,
                clock_src: SysClkSource::HSI,
                #[cfg(feature = "stm32f070")]
                usb_src: USBClockSource::Disabled,
                rcc: self,
            }
        }
    }
    /// Constrained RCC peripheral
    pub struct Rcc {
        pub clocks: Clocks,
        pub(crate) regs: RCC,
    }
    pub enum HSEBypassMode {
        /// Not bypassed: for crystals
        NotBypassed,
        /// Bypassed: for external clock sources
        Bypassed,
    }
    #[cfg(any(
        feature = "stm32f042",
        feature = "stm32f048",
        feature = "stm32f070",
        feature = "stm32f072",
        feature = "stm32f078",
    ))]
    pub enum USBClockSource {
        #[cfg(feature = "stm32f070")]
        /// USB peripheral's tranceiver is disabled
        Disabled,
        /// PLL output is used as USB peripheral tranceiver clock
        PLL,
    }
    /// RCC for F0x0 devices
    #[cfg(any(feature = "stm32f030", feature = "stm32f070",))]
    mod inner {
        use crate::pac::{rcc::cfgr::SW_A, RCC};
        pub(super) const HSI: u32 = 8_000_000;
        #[cfg(any(
            feature = "stm32f030x4",
            feature = "stm32f030x6",
            feature = "stm32f030x8",
            feature = "stm32f070",
            feature = "stm32f030xc"
        ))]
        pub(super) const RCC_PLLSRC_PREDIV1_SUPPORT: bool = false;
        pub(super) enum SysClkSource {
            HSI,
            /// High-speed external clock(freq,bypassed)
            HSE(u32, super::HSEBypassMode),
        }
        pub(super) fn get_freq(c_src: &SysClkSource) -> u32 {
            match c_src {
                SysClkSource::HSE(freq, _) => *freq,
                _ => HSI,
            }
        }
        pub(super) fn enable_clock(rcc: &mut RCC, c_src: &SysClkSource) {
            match c_src {
                SysClkSource::HSE(_, bypassed) => {
                    match bypassed {
                        super::HSEBypassMode::NotBypassed => {
                            rcc.cr
                                .modify(|_, w| w.csson().on().hseon().on().hsebyp().not_bypassed());
                        }
                        super::HSEBypassMode::Bypassed => {
                            rcc.cr
                                .modify(|_, w| w.csson().on().hseon().on().hsebyp().bypassed());
                        }
                    }
                    while !rcc.cr.read().hserdy().bit_is_set() {}
                }
                SysClkSource::HSI => {
                    rcc.cr.write(|w| w.hsion().set_bit());
                    while rcc.cr.read().hsirdy().bit_is_clear() {}
                }
            }
        }
        pub(super) fn enable_pll(
            rcc: &mut RCC,
            c_src: &SysClkSource,
            pllmul_bits: u8,
            ppre_bits: u8,
            hpre_bits: u8,
        ) {
            let pllsrc_bit: bool = match c_src {
                SysClkSource::HSI => false,
                SysClkSource::HSE(_, _) => true,
            };
            rcc.cfgr
                .modify(|_, w| w.pllsrc().bit(pllsrc_bit).pllmul().bits(pllmul_bits));
            rcc.cr.modify(|_, w| w.pllon().set_bit());
            while rcc.cr.read().pllrdy().bit_is_clear() {}
            rcc.cfgr.modify(|_, w| unsafe {
                w.ppre().bits(ppre_bits).hpre().bits(hpre_bits).sw().pll()
            });
        }
        pub(super) fn get_sww(c_src: &SysClkSource) -> SW_A {
            match c_src {
                SysClkSource::HSI => SW_A::HSI,
                SysClkSource::HSE(_, _) => SW_A::HSE,
            }
        }
    }
    use self::inner::SysClkSource;
    pub struct CFGR {
        hclk: Option<u32>,
        pclk: Option<u32>,
        sysclk: Option<u32>,
        clock_src: SysClkSource,
        #[cfg(any(
            feature = "stm32f042",
            feature = "stm32f048",
            feature = "stm32f070",
            feature = "stm32f072",
            feature = "stm32f078",
        ))]
        usb_src: USBClockSource,
        rcc: RCC,
    }
    impl CFGR {
        pub fn hse<F>(mut self, freq: F, bypass: HSEBypassMode) -> Self
        where
            F: Into<Hertz>,
        {
            self.clock_src = SysClkSource::HSE(freq.into().0, bypass);
            self
        }
        pub fn hclk<F>(mut self, freq: F) -> Self
        where
            F: Into<Hertz>,
        {
            self.hclk = Some(freq.into().0);
            self
        }
        pub fn pclk<F>(mut self, freq: F) -> Self
        where
            F: Into<Hertz>,
        {
            self.pclk = Some(freq.into().0);
            self
        }
        pub fn sysclk<F>(mut self, freq: F) -> Self
        where
            F: Into<Hertz>,
        {
            self.sysclk = Some(freq.into().0);
            self
        }
        #[cfg(any(
            feature = "stm32f042",
            feature = "stm32f048",
            feature = "stm32f070",
            feature = "stm32f072",
            feature = "stm32f078",
        ))]
        /// Set the USB clock source (only valid for STM32F0xx with USB)
        pub fn usbsrc(mut self, src: USBClockSource) -> Self {
            self.usb_src = src;
            self
        }
        pub fn freeze(mut self, flash: &mut crate::pac::FLASH) -> Rcc {
            let sysclk = self.sysclk.unwrap_or(self::inner::HSI);
            let r_sysclk;
            let pllmul_bits;
            let src_clk_freq = self::inner::get_freq(&self.clock_src);
            if sysclk == src_clk_freq {
                pllmul_bits = None;
                r_sysclk = src_clk_freq;
            } else {
                let pllprediv = match (&self.clock_src, self::inner::RCC_PLLSRC_PREDIV1_SUPPORT) {
                    (self::inner::SysClkSource::HSI, false) => 2,
                    (_, _) => 1,
                };
                let pllmul = (2 * pllprediv * self.sysclk.unwrap_or(src_clk_freq) + src_clk_freq)
                    / src_clk_freq
                    / 2;
                let pllmul = core::cmp::min(core::cmp::max(pllmul, 2), 16);
                r_sysclk = pllmul * src_clk_freq / pllprediv;
                pllmul_bits = Some(pllmul as u8 - 2)
            }
            let hpre_bits = self
                .hclk
                .map(|hclk| match r_sysclk / hclk {
                    0 => ::core::panicking::panic("internal error: entered unreachable code"),
                    1 => 0b0111,
                    2 => 0b1000,
                    3..=5 => 0b1001,
                    6..=11 => 0b1010,
                    12..=39 => 0b1011,
                    40..=95 => 0b1100,
                    96..=191 => 0b1101,
                    192..=383 => 0b1110,
                    _ => 0b1111,
                })
                .unwrap_or(0b0111);
            let hclk = r_sysclk / (1 << (hpre_bits - 0b0111));
            let ppre_bits = self
                .pclk
                .map(|pclk| match hclk / pclk {
                    0 => ::core::panicking::panic("internal error: entered unreachable code"),
                    1 => 0b011,
                    2 => 0b100,
                    3..=5 => 0b101,
                    6..=11 => 0b110,
                    _ => 0b111,
                })
                .unwrap_or(0b011);
            let ppre: u8 = 1 << (ppre_bits - 0b011);
            let pclk = hclk / cast::u32(ppre);
            unsafe {
                flash.acr.write(|w| {
                    w.latency().bits(if r_sysclk <= 24_000_000 {
                        0b000
                    } else if r_sysclk <= 48_000_000 {
                        0b001
                    } else {
                        0b010
                    })
                })
            }
            self::inner::enable_clock(&mut self.rcc, &self.clock_src);
            #[cfg(feature = "stm32f070")]
            {
                match self.usb_src {
                    USBClockSource::Disabled => self.rcc.cfgr3.modify(|_, w| w.usbsw().clear_bit()),
                    USBClockSource::PLL => self.rcc.cfgr3.modify(|_, w| w.usbsw().set_bit()),
                }
            }
            if let Some(pllmul_bits) = pllmul_bits {
                self::inner::enable_pll(
                    &mut self.rcc,
                    &self.clock_src,
                    pllmul_bits,
                    ppre_bits,
                    hpre_bits,
                );
            } else {
                let sw_var = self::inner::get_sww(&self.clock_src);
                self.rcc.cfgr.modify(|_, w| unsafe {
                    w.ppre()
                        .bits(ppre_bits)
                        .hpre()
                        .bits(hpre_bits)
                        .sw()
                        .variant(sw_var)
                });
            }
            Rcc {
                clocks: Clocks {
                    hclk: Hertz(hclk),
                    pclk: Hertz(pclk),
                    sysclk: Hertz(sysclk),
                },
                regs: self.rcc,
            }
        }
    }
    /// Frozen clock frequencies
    ///
    /// The existence of this value indicates that the clock configuration can no longer be changed
    pub struct Clocks {
        hclk: Hertz,
        pclk: Hertz,
        sysclk: Hertz,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Clocks {
        #[inline]
        fn clone(&self) -> Clocks {
            {
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                let _: ::core::clone::AssertParamIsClone<Hertz>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Clocks {}
    impl Clocks {
        /// Returns the frequency of the AHB
        pub fn hclk(&self) -> Hertz {
            self.hclk
        }
        /// Returns the frequency of the APB
        pub fn pclk(&self) -> Hertz {
            self.pclk
        }
        /// Returns the system (core) frequency
        pub fn sysclk(&self) -> Hertz {
            self.sysclk
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod serial {
    //! API for the integrated USART ports
    //!
    //! This only implements the usual asynchronous bidirectional 8-bit transfers.
    //!
    //! It's possible to use a read-only/write-only serial implementation with
    //! `usartXrx`/`usartXtx`.
    //!
    //! # Examples
    //! Echo
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::prelude::*;
    //! use crate::hal::serial::Serial;
    //! use crate::hal::pac;
    //!
    //! use nb::block;
    //!
    //! cortex_m::interrupt::free(|cs| {
    //!     let rcc = p.RCC.configure().sysclk(48.mhz()).freeze();
    //!
    //!     let gpioa = p.GPIOA.split(&mut rcc);
    //!
    //!     let tx = gpioa.pa9.into_alternate_af1(cs);
    //!     let rx = gpioa.pa10.into_alternate_af1(cs);
    //!
    //!     let mut serial = Serial::usart1(p.USART1, (tx, rx), 115_200.bps(), &mut rcc);
    //!
    //!     loop {
    //!         let received = block!(serial.read()).unwrap();
    //!         block!(serial.write(received)).ok();
    //!     }
    //! });
    //! ```
    //!
    //! Hello World
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::prelude::*;
    //! use crate::hal::serial::Serial;
    //! use crate::hal::pac;
    //!
    //! use nb::block;
    //!
    //! cortex_m::interrupt::free(|cs| {
    //!     let rcc = p.RCC.configure().sysclk(48.mhz()).freeze();
    //!
    //!     let gpioa = p.GPIOA.split(&mut rcc);
    //!
    //!     let tx = gpioa.pa9.into_alternate_af1(cs);
    //!
    //!     let mut serial = Serial::usart1tx(p.USART1, tx, 115_200.bps(), &mut rcc);
    //!
    //!     loop {
    //!         serial.write_str("Hello World!\r\n");
    //!     }
    //! });
    //! ```
    use core::{
        convert::Infallible,
        fmt::{Result, Write},
        ops::Deref,
    };
    use embedded_hal::prelude::*;
    use crate::{gpio::*, rcc::Rcc, time::Bps};
    use core::marker::PhantomData;
    /// Serial error
    #[non_exhaustive]
    pub enum Error {
        /// Framing error
        Framing,
        /// Noise error
        Noise,
        /// RX buffer overrun
        Overrun,
        /// Parity check error
        Parity,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::Framing,) => ::core::fmt::Formatter::write_str(f, "Framing"),
                (&Error::Noise,) => ::core::fmt::Formatter::write_str(f, "Noise"),
                (&Error::Overrun,) => ::core::fmt::Formatter::write_str(f, "Overrun"),
                (&Error::Parity,) => ::core::fmt::Formatter::write_str(f, "Parity"),
            }
        }
    }
    /// Interrupt event
    pub enum Event {
        /// New data has been received
        Rxne,
        /// New data can be sent
        Txe,
        /// Idle line state detected
        Idle,
    }
    pub trait TxPin<USART> {}
    pub trait RxPin<USART> {}
    impl TxPin<crate::pac::USART1> for gpioa::PA9<Alternate<AF1>> {}
    impl TxPin<crate::pac::USART1> for gpiob::PB6<Alternate<AF0>> {}
    impl RxPin<crate::pac::USART1> for gpioa::PA10<Alternate<AF1>> {}
    impl RxPin<crate::pac::USART1> for gpiob::PB7<Alternate<AF0>> {}
    impl TxPin<crate::pac::USART2> for gpioa::PA2<Alternate<AF1>> {}
    impl TxPin<crate::pac::USART2> for gpioa::PA14<Alternate<AF1>> {}
    impl RxPin<crate::pac::USART2> for gpioa::PA3<Alternate<AF1>> {}
    impl RxPin<crate::pac::USART2> for gpioa::PA15<Alternate<AF1>> {}
    /// Serial abstraction
    pub struct Serial<USART, TXPIN, RXPIN> {
        usart: USART,
        pins: (TXPIN, RXPIN),
    }
    type SerialRegisterBlock = crate::pac::usart1::RegisterBlock;
    /// Serial receiver
    pub struct Rx<USART> {
        usart: *const SerialRegisterBlock,
        _instance: PhantomData<USART>,
    }
    unsafe impl<USART> Send for Rx<USART> {}
    /// Serial transmitter
    pub struct Tx<USART> {
        usart: *const SerialRegisterBlock,
        _instance: PhantomData<USART>,
    }
    unsafe impl<USART> Send for Tx<USART> {}
    use crate::pac::USART1;
    impl<TXPIN, RXPIN> Serial<USART1, TXPIN, RXPIN>
    where
        TXPIN: TxPin<USART1>,
        RXPIN: RxPin<USART1>,
    {
        /// Creates a new serial instance
        pub fn usart1(usart: USART1, pins: (TXPIN, RXPIN), baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let mut serial = Serial { usart, pins };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.te().set_bit().re().set_bit().ue().set_bit());
            serial
        }
    }
    impl<TXPIN> Serial<USART1, TXPIN, ()>
    where
        TXPIN: TxPin<USART1>,
    {
        /// Creates a new tx-only serial instance
        pub fn usart1tx(usart: USART1, txpin: TXPIN, baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let rxpin = ();
            let mut serial = Serial {
                usart,
                pins: (txpin, rxpin),
            };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.te().set_bit().ue().set_bit());
            serial
        }
    }
    impl<RXPIN> Serial<USART1, (), RXPIN>
    where
        RXPIN: RxPin<USART1>,
    {
        /// Creates a new rx-only serial instance
        pub fn usart1rx(usart: USART1, rxpin: RXPIN, baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let txpin = ();
            let mut serial = Serial {
                usart,
                pins: (txpin, rxpin),
            };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.re().set_bit().ue().set_bit());
            serial
        }
    }
    impl<TXPIN, RXPIN> Serial<USART1, TXPIN, RXPIN> {
        fn configure(&mut self, baud_rate: Bps, rcc: &mut Rcc) {
            rcc.regs.apb2enr.modify(|_, w| w.usart1en().set_bit());
            let brr = rcc.clocks.pclk().0 / baud_rate.0;
            self.usart.brr.write(|w| unsafe { w.bits(brr) });
            self.usart.cr2.reset();
            self.usart.cr3.reset();
        }
        /// Starts listening for an interrupt event
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
            }
        }
        /// Stop listening for an interrupt event
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
            }
        }
    }
    use crate::pac::USART2;
    impl<TXPIN, RXPIN> Serial<USART2, TXPIN, RXPIN>
    where
        TXPIN: TxPin<USART2>,
        RXPIN: RxPin<USART2>,
    {
        /// Creates a new serial instance
        pub fn usart2(usart: USART2, pins: (TXPIN, RXPIN), baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let mut serial = Serial { usart, pins };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.te().set_bit().re().set_bit().ue().set_bit());
            serial
        }
    }
    impl<TXPIN> Serial<USART2, TXPIN, ()>
    where
        TXPIN: TxPin<USART2>,
    {
        /// Creates a new tx-only serial instance
        pub fn usart2tx(usart: USART2, txpin: TXPIN, baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let rxpin = ();
            let mut serial = Serial {
                usart,
                pins: (txpin, rxpin),
            };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.te().set_bit().ue().set_bit());
            serial
        }
    }
    impl<RXPIN> Serial<USART2, (), RXPIN>
    where
        RXPIN: RxPin<USART2>,
    {
        /// Creates a new rx-only serial instance
        pub fn usart2rx(usart: USART2, rxpin: RXPIN, baud_rate: Bps, rcc: &mut Rcc) -> Self {
            let txpin = ();
            let mut serial = Serial {
                usart,
                pins: (txpin, rxpin),
            };
            serial.configure(baud_rate, rcc);
            serial
                .usart
                .cr1
                .modify(|_, w| w.re().set_bit().ue().set_bit());
            serial
        }
    }
    impl<TXPIN, RXPIN> Serial<USART2, TXPIN, RXPIN> {
        fn configure(&mut self, baud_rate: Bps, rcc: &mut Rcc) {
            rcc.regs.apb1enr.modify(|_, w| w.usart2en().set_bit());
            let brr = rcc.clocks.pclk().0 / baud_rate.0;
            self.usart.brr.write(|w| unsafe { w.bits(brr) });
            self.usart.cr2.reset();
            self.usart.cr3.reset();
        }
        /// Starts listening for an interrupt event
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().set_bit()),
            }
        }
        /// Stop listening for an interrupt event
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
                Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
                Event::Idle => self.usart.cr1.modify(|_, w| w.idleie().clear_bit()),
            }
        }
    }
    impl<USART> embedded_hal::serial::Read<u8> for Rx<USART>
    where
        USART: Deref<Target = SerialRegisterBlock>,
    {
        type Error = Error;
        /// Tries to read a byte from the uart
        fn read(&mut self) -> nb::Result<u8, Error> {
            read(self.usart)
        }
    }
    impl<USART, TXPIN, RXPIN> embedded_hal::serial::Read<u8> for Serial<USART, TXPIN, RXPIN>
    where
        USART: Deref<Target = SerialRegisterBlock>,
        RXPIN: RxPin<USART>,
    {
        type Error = Error;
        /// Tries to read a byte from the uart
        fn read(&mut self) -> nb::Result<u8, Error> {
            read(&*self.usart)
        }
    }
    impl<USART> embedded_hal::serial::Write<u8> for Tx<USART>
    where
        USART: Deref<Target = SerialRegisterBlock>,
    {
        type Error = Infallible;
        /// Ensures that none of the previously written words are still buffered
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            flush(self.usart)
        }
        /// Tries to write a byte to the uart
        /// Fails if the transmit buffer is full
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            write(self.usart, byte)
        }
    }
    impl<USART, TXPIN, RXPIN> embedded_hal::serial::Write<u8> for Serial<USART, TXPIN, RXPIN>
    where
        USART: Deref<Target = SerialRegisterBlock>,
        TXPIN: TxPin<USART>,
    {
        type Error = Infallible;
        /// Ensures that none of the previously written words are still buffered
        fn flush(&mut self) -> nb::Result<(), Self::Error> {
            flush(&*self.usart)
        }
        /// Tries to write a byte to the uart
        /// Fails if the transmit buffer is full
        fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
            write(&*self.usart, byte)
        }
    }
    impl<USART, TXPIN, RXPIN> Serial<USART, TXPIN, RXPIN>
    where
        USART: Deref<Target = SerialRegisterBlock>,
    {
        /// Splits the UART Peripheral in a Tx and an Rx part
        /// This is required for sending/receiving
        pub fn split(self) -> (Tx<USART>, Rx<USART>)
        where
            TXPIN: TxPin<USART>,
            RXPIN: RxPin<USART>,
        {
            (
                Tx {
                    usart: &*self.usart,
                    _instance: PhantomData,
                },
                Rx {
                    usart: &*self.usart,
                    _instance: PhantomData,
                },
            )
        }
        pub fn release(self) -> (USART, (TXPIN, RXPIN)) {
            (self.usart, self.pins)
        }
    }
    impl<USART> Write for Tx<USART>
    where
        Tx<USART>: embedded_hal::serial::Write<u8>,
    {
        fn write_str(&mut self, s: &str) -> Result {
            s.as_bytes()
                .iter()
                .try_for_each(|c| loop {
                    #[allow(unreachable_patterns)]
                    match self.write(*c) {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                })
                .map_err(|_| core::fmt::Error)
        }
    }
    impl<USART, TXPIN, RXPIN> Write for Serial<USART, TXPIN, RXPIN>
    where
        USART: Deref<Target = SerialRegisterBlock>,
        TXPIN: TxPin<USART>,
    {
        fn write_str(&mut self, s: &str) -> Result {
            s.as_bytes()
                .iter()
                .try_for_each(|c| loop {
                    #[allow(unreachable_patterns)]
                    match self.write(*c) {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                })
                .map_err(|_| core::fmt::Error)
        }
    }
    /// Ensures that none of the previously written words are still buffered
    fn flush(usart: *const SerialRegisterBlock) -> nb::Result<(), Infallible> {
        let isr = unsafe { (*usart).isr.read() };
        if isr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
    /// Tries to write a byte to the UART
    /// Returns `Err(WouldBlock)` if the transmit buffer is full
    fn write(usart: *const SerialRegisterBlock, byte: u8) -> nb::Result<(), Infallible> {
        let isr = unsafe { (*usart).isr.read() };
        if isr.txe().bit_is_set() {
            unsafe { (*usart).tdr.write(|w| w.tdr().bits(byte as u16)) }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
    /// Tries to read a byte from the UART
    fn read(usart: *const SerialRegisterBlock) -> nb::Result<u8, Error> {
        let isr = unsafe { (*usart).isr.read() };
        let icr = unsafe { &(*usart).icr };
        if isr.pe().bit_is_set() {
            icr.write(|w| w.pecf().set_bit());
            Err(nb::Error::Other(Error::Parity))
        } else if isr.fe().bit_is_set() {
            icr.write(|w| w.fecf().set_bit());
            Err(nb::Error::Other(Error::Framing))
        } else if isr.nf().bit_is_set() {
            icr.write(|w| w.ncf().set_bit());
            Err(nb::Error::Other(Error::Noise))
        } else if isr.ore().bit_is_set() {
            icr.write(|w| w.orecf().set_bit());
            Err(nb::Error::Other(Error::Overrun))
        } else if isr.rxne().bit_is_set() {
            Ok(unsafe { (*usart).rdr.read().rdr().bits() as u8 })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod spi {
    //! API for the integrate SPI peripherals
    //!
    //! The spi bus acts as the master (generating the clock) and you need to handle the CS separately.
    //!
    //! The most significant bit is transmitted first & only 8-bit transfers are supported
    //!
    //! # Example
    //! Echo incoming data in the next transfer
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::pac;
    //! use crate::hal::prelude::*;
    //! use crate::hal::spi::{Spi, Mode, Phase, Polarity};
    //!
    //! cortex_m::interrupt::free(|cs| {
    //!     let mut p = pac::Peripherals::take().unwrap();
    //!     let mut rcc = p.RCC.constrain().freeze(&mut p.FLASH);
    //!
    //!     let gpioa = p.GPIOA.split(&mut rcc);
    //!
    //!     // Configure pins for SPI
    //!     let sck = gpioa.pa5.into_alternate_af0(cs);
    //!     let miso = gpioa.pa6.into_alternate_af0(cs);
    //!     let mosi = gpioa.pa7.into_alternate_af0(cs);
    //!
    //!     // Configure SPI with 1MHz rate
    //!     let mut spi = Spi::spi1(p.SPI1, (sck, miso, mosi), Mode {
    //!         polarity: Polarity::IdleHigh,
    //!         phase: Phase::CaptureOnSecondTransition,
    //!     }, 1.mhz(), &mut rcc);
    //!
    //!     let mut data = [0];
    //!     loop {
    //!         spi.transfer(&mut data).unwrap();
    //!     }
    //! });
    //! ```
    use core::marker::PhantomData;
    use core::{ops::Deref, ptr};
    pub use embedded_hal::spi::{Mode, Phase, Polarity};
    use crate::pac::SPI1;
    use crate::gpio::*;
    use crate::rcc::{Clocks, Rcc};
    use crate::time::Hertz;
    /// Typestate for 8-bit transfer size
    pub struct EightBit;
    /// Typestate for 16-bit transfer size
    pub struct SixteenBit;
    /// SPI error
    #[non_exhaustive]
    pub enum Error {
        /// Overrun occurred
        Overrun,
        /// Mode fault occurred
        ModeFault,
        /// CRC error
        Crc,
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::fmt::Debug for Error {
        fn fmt(&self, f: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
            match (&*self,) {
                (&Error::Overrun,) => ::core::fmt::Formatter::write_str(f, "Overrun"),
                (&Error::ModeFault,) => ::core::fmt::Formatter::write_str(f, "ModeFault"),
                (&Error::Crc,) => ::core::fmt::Formatter::write_str(f, "Crc"),
            }
        }
    }
    /// SPI abstraction
    pub struct Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> {
        spi: SPI,
        pins: (SCKPIN, MISOPIN, MOSIPIN),
        _width: PhantomData<WIDTH>,
    }
    pub trait SckPin<SPI> {}
    pub trait MisoPin<SPI> {}
    pub trait MosiPin<SPI> {}
    impl SckPin<crate::pac::SPI1> for gpioa::PA5<Alternate<AF0>> {}
    impl SckPin<crate::pac::SPI1> for gpiob::PB3<Alternate<AF0>> {}
    impl MisoPin<crate::pac::SPI1> for gpioa::PA6<Alternate<AF0>> {}
    impl MisoPin<crate::pac::SPI1> for gpiob::PB4<Alternate<AF0>> {}
    impl MosiPin<crate::pac::SPI1> for gpioa::PA7<Alternate<AF0>> {}
    impl MosiPin<crate::pac::SPI1> for gpiob::PB5<Alternate<AF0>> {}
    impl<SCKPIN, MISOPIN, MOSIPIN> Spi<SPI1, SCKPIN, MISOPIN, MOSIPIN, EightBit> {
        /// Creates a new spi instance
        pub fn spi1<F>(
            spi: SPI1,
            pins: (SCKPIN, MISOPIN, MOSIPIN),
            mode: Mode,
            speed: F,
            rcc: &mut Rcc,
        ) -> Self
        where
            SCKPIN: SckPin<SPI1>,
            MISOPIN: MisoPin<SPI1>,
            MOSIPIN: MosiPin<SPI1>,
            F: Into<Hertz>,
        {
            rcc.regs.apb2enr.modify(|_, w| w.spi1en().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.spi1rst().clear_bit());
            Spi::<SPI1, SCKPIN, MISOPIN, MOSIPIN, EightBit> {
                spi,
                pins,
                _width: PhantomData,
            }
            .spi_init(mode, speed, rcc.clocks)
            .into_8bit_width()
        }
    }
    #[allow(dead_code)]
    type SpiRegisterBlock = crate::pac::spi1::RegisterBlock;
    impl<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
    where
        SPI: Deref<Target = SpiRegisterBlock>,
    {
        fn spi_init<F>(self, mode: Mode, speed: F, clocks: Clocks) -> Self
        where
            F: Into<Hertz>,
        {
            self.spi.cr1.modify(|_, w| w.spe().clear_bit());
            let br = match clocks.pclk().0 / speed.into().0 {
                0 => ::core::panicking::panic("internal error: entered unreachable code"),
                1..=2 => 0b000,
                3..=5 => 0b001,
                6..=11 => 0b010,
                12..=23 => 0b011,
                24..=47 => 0b100,
                48..=95 => 0b101,
                96..=191 => 0b110,
                _ => 0b111,
            };
            self.spi.cr1.write(|w| {
                w.cpha()
                    .bit(mode.phase == Phase::CaptureOnSecondTransition)
                    .cpol()
                    .bit(mode.polarity == Polarity::IdleHigh)
                    .mstr()
                    .set_bit()
                    .br()
                    .bits(br)
                    .lsbfirst()
                    .clear_bit()
                    .ssm()
                    .set_bit()
                    .ssi()
                    .set_bit()
                    .rxonly()
                    .clear_bit()
                    .bidimode()
                    .clear_bit()
                    .spe()
                    .set_bit()
            });
            self
        }
        pub fn into_8bit_width(self) -> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, EightBit> {
            self.spi
                .cr2
                .write(|w| w.frxth().set_bit().ds().eight_bit().ssoe().clear_bit());
            Spi {
                spi: self.spi,
                pins: self.pins,
                _width: PhantomData,
            }
        }
        pub fn into_16bit_width(self) -> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, SixteenBit> {
            self.spi
                .cr2
                .write(|w| w.frxth().set_bit().ds().sixteen_bit().ssoe().clear_bit());
            Spi {
                spi: self.spi,
                pins: self.pins,
                _width: PhantomData,
            }
        }
        fn set_send_only(&mut self) {
            self.spi
                .cr1
                .modify(|_, w| w.bidimode().set_bit().bidioe().set_bit());
        }
        fn set_bidi(&mut self) {
            self.spi
                .cr1
                .modify(|_, w| w.bidimode().clear_bit().bidioe().clear_bit());
        }
        fn check_read(&mut self) -> nb::Result<(), Error> {
            let sr = self.spi.sr.read();
            Err(if sr.ovr().bit_is_set() {
                nb::Error::Other(Error::Overrun)
            } else if sr.modf().bit_is_set() {
                nb::Error::Other(Error::ModeFault)
            } else if sr.crcerr().bit_is_set() {
                nb::Error::Other(Error::Crc)
            } else if sr.rxne().bit_is_set() {
                return Ok(());
            } else {
                nb::Error::WouldBlock
            })
        }
        fn send_buffer_size(&mut self) -> u8 {
            match self.spi.sr.read().ftlvl().bits() {
                0 => 4,
                1 => 3,
                2 => 2,
                _ => 0,
            }
        }
        fn check_send(&mut self) -> nb::Result<(), Error> {
            let sr = self.spi.sr.read();
            Err(if sr.ovr().bit_is_set() {
                nb::Error::Other(Error::Overrun)
            } else if sr.modf().bit_is_set() {
                nb::Error::Other(Error::ModeFault)
            } else if sr.crcerr().bit_is_set() {
                nb::Error::Other(Error::Crc)
            } else if sr.txe().bit_is_set() && sr.bsy().bit_is_clear() {
                return Ok(());
            } else {
                nb::Error::WouldBlock
            })
        }
        fn read_u8(&mut self) -> u8 {
            unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u8) }
        }
        fn send_u8(&mut self, byte: u8) {
            unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u8, byte) }
        }
        fn read_u16(&mut self) -> u16 {
            unsafe { ptr::read_volatile(&self.spi.dr as *const _ as *const u16) }
        }
        fn send_u16(&mut self, byte: u16) {
            unsafe { ptr::write_volatile(&self.spi.dr as *const _ as *mut u16, byte) }
        }
        pub fn release(self) -> (SPI, (SCKPIN, MISOPIN, MOSIPIN)) {
            (self.spi, self.pins)
        }
    }
    impl<SPI, SCKPIN, MISOPIN, MOSIPIN> ::embedded_hal::blocking::spi::Transfer<u8>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, EightBit>
    where
        SPI: Deref<Target = SpiRegisterBlock>,
    {
        type Error = Error;
        fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
            self.set_bidi();
            for word in words.iter_mut() {
                loop {
                    #[allow(unreachable_patterns)]
                    match self.check_send() {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                }?;
                self.send_u8(*word);
                loop {
                    #[allow(unreachable_patterns)]
                    match self.check_read() {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                }?;
                *word = self.read_u8();
            }
            Ok(words)
        }
    }
    impl<SPI, SCKPIN, MISOPIN, MOSIPIN> ::embedded_hal::blocking::spi::Write<u8>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, EightBit>
    where
        SPI: Deref<Target = SpiRegisterBlock>,
    {
        type Error = Error;
        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            let mut bufcap: u8 = 0;
            self.set_send_only();
            loop {
                #[allow(unreachable_patterns)]
                match self.check_send() {
                    Err(::nb::Error::Other(e)) =>
                    {
                        #[allow(unreachable_code)]
                        break Err(e)
                    }
                    Err(::nb::Error::WouldBlock) => {}
                    Ok(x) => break Ok(x),
                }
            }?;
            for word in words {
                while bufcap == 0 {
                    bufcap = self.send_buffer_size();
                }
                self.send_u8(*word);
                bufcap -= 1;
            }
            loop {
                #[allow(unreachable_patterns)]
                match self.check_send() {
                    Err(::nb::Error::Other(e)) =>
                    {
                        #[allow(unreachable_code)]
                        break Err(e)
                    }
                    Err(::nb::Error::WouldBlock) => {}
                    Ok(x) => break Ok(x),
                }
            }
            .ok();
            Ok(())
        }
    }
    impl<SPI, SCKPIN, MISOPIN, MOSIPIN> ::embedded_hal::blocking::spi::Transfer<u16>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, SixteenBit>
    where
        SPI: Deref<Target = SpiRegisterBlock>,
    {
        type Error = Error;
        fn transfer<'w>(&mut self, words: &'w mut [u16]) -> Result<&'w [u16], Self::Error> {
            self.set_bidi();
            for word in words.iter_mut() {
                loop {
                    #[allow(unreachable_patterns)]
                    match self.check_send() {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                }?;
                self.send_u16(*word);
                loop {
                    #[allow(unreachable_patterns)]
                    match self.check_read() {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                }?;
                *word = self.read_u16();
            }
            Ok(words)
        }
    }
    impl<SPI, SCKPIN, MISOPIN, MOSIPIN> ::embedded_hal::blocking::spi::Write<u16>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, SixteenBit>
    where
        SPI: Deref<Target = SpiRegisterBlock>,
    {
        type Error = Error;
        fn write(&mut self, words: &[u16]) -> Result<(), Self::Error> {
            self.set_send_only();
            for word in words {
                loop {
                    #[allow(unreachable_patterns)]
                    match self.check_send() {
                        Err(::nb::Error::Other(e)) =>
                        {
                            #[allow(unreachable_code)]
                            break Err(e)
                        }
                        Err(::nb::Error::WouldBlock) => {}
                        Ok(x) => break Ok(x),
                    }
                }?;
                self.send_u16(*word);
            }
            loop {
                #[allow(unreachable_patterns)]
                match self.check_send() {
                    Err(::nb::Error::Other(e)) =>
                    {
                        #[allow(unreachable_code)]
                        break Err(e)
                    }
                    Err(::nb::Error::WouldBlock) => {}
                    Ok(x) => break Ok(x),
                }
            }
            .ok();
            Ok(())
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod time {
    /// Bits per second
    pub struct Bps(pub u32);
    impl ::core::marker::StructuralPartialEq for Bps {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Bps {
        #[inline]
        fn eq(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &Bps) -> bool {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for Bps {
        #[inline]
        fn partial_cmp(&self, other: &Bps) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                Bps(ref __self_1_0) => match *self {
                    Bps(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Bps {
        #[inline]
        fn clone(&self) -> Bps {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Bps {}
    pub struct Hertz(pub u32);
    impl ::core::marker::StructuralPartialEq for Hertz {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for Hertz {
        #[inline]
        fn eq(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &Hertz) -> bool {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for Hertz {
        #[inline]
        fn partial_cmp(&self, other: &Hertz) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                Hertz(ref __self_1_0) => match *self {
                    Hertz(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for Hertz {
        #[inline]
        fn clone(&self) -> Hertz {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for Hertz {}
    pub struct KiloHertz(pub u32);
    impl ::core::marker::StructuralPartialEq for KiloHertz {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for KiloHertz {
        #[inline]
        fn eq(&self, other: &KiloHertz) -> bool {
            match *other {
                KiloHertz(ref __self_1_0) => match *self {
                    KiloHertz(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &KiloHertz) -> bool {
            match *other {
                KiloHertz(ref __self_1_0) => match *self {
                    KiloHertz(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for KiloHertz {
        #[inline]
        fn partial_cmp(&self, other: &KiloHertz) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                KiloHertz(ref __self_1_0) => match *self {
                    KiloHertz(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for KiloHertz {
        #[inline]
        fn clone(&self) -> KiloHertz {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for KiloHertz {}
    pub struct MegaHertz(pub u32);
    impl ::core::marker::StructuralPartialEq for MegaHertz {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for MegaHertz {
        #[inline]
        fn eq(&self, other: &MegaHertz) -> bool {
            match *other {
                MegaHertz(ref __self_1_0) => match *self {
                    MegaHertz(ref __self_0_0) => (*__self_0_0) == (*__self_1_0),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &MegaHertz) -> bool {
            match *other {
                MegaHertz(ref __self_1_0) => match *self {
                    MegaHertz(ref __self_0_0) => (*__self_0_0) != (*__self_1_0),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for MegaHertz {
        #[inline]
        fn partial_cmp(&self, other: &MegaHertz) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                MegaHertz(ref __self_1_0) => match *self {
                    MegaHertz(ref __self_0_0) => {
                        match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0)) {
                            ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                            }
                            cmp => cmp,
                        }
                    }
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for MegaHertz {
        #[inline]
        fn clone(&self) -> MegaHertz {
            {
                let _: ::core::clone::AssertParamIsClone<u32>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for MegaHertz {}
    /// Extension trait that adds convenience methods to the `u32` type
    pub trait U32Ext {
        /// Wrap in `Bps`
        fn bps(self) -> Bps;
        /// Wrap in `Hertz`
        fn hz(self) -> Hertz;
        /// Wrap in `KiloHertz`
        fn khz(self) -> KiloHertz;
        /// Wrap in `MegaHertz`
        fn mhz(self) -> MegaHertz;
    }
    impl U32Ext for u32 {
        fn bps(self) -> Bps {
            Bps(self)
        }
        fn hz(self) -> Hertz {
            Hertz(self)
        }
        fn khz(self) -> KiloHertz {
            KiloHertz(self)
        }
        fn mhz(self) -> MegaHertz {
            MegaHertz(self)
        }
    }
    impl Into<Hertz> for KiloHertz {
        fn into(self) -> Hertz {
            Hertz(self.0 * 1_000)
        }
    }
    impl Into<Hertz> for MegaHertz {
        fn into(self) -> Hertz {
            Hertz(self.0 * 1_000_000)
        }
    }
    impl Into<KiloHertz> for MegaHertz {
        fn into(self) -> KiloHertz {
            KiloHertz(self.0 * 1_000)
        }
    }
}
#[cfg(feature = "device-selected")]
pub mod timers {
    //! API for the integrated timers
    //!
    //! This only implements basic functions, a lot of things are missing
    //!
    //! # Example
    //! Blink the led with 1Hz
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::pac;
    //! use crate::hal::prelude::*;
    //! use crate::hal::time::*;
    //! use crate::hal::timers::*;
    //! use nb::block;
    //!
    //! cortex_m::interrupt::free(|cs| {
    //!     let mut p = pac::Peripherals::take().unwrap();
    //!     let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);
    //!
    //!     let gpioa = p.GPIOA.split(&mut rcc);
    //!
    //!     let mut led = gpioa.pa1.into_push_pull_pull_output(cs);
    //!
    //!     let mut timer = Timer::tim1(p.TIM1, Hertz(1), &mut rcc);
    //!     loop {
    //!         led.toggle();
    //!         block!(timer.wait()).ok();
    //!     }
    //! });
    //! ```
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SYST;
    use crate::rcc::{Clocks, Rcc};
    use crate::time::Hertz;
    use embedded_hal::timer::{CountDown, Periodic};
    use void::Void;
    /// Hardware timers
    pub struct Timer<TIM> {
        clocks: Clocks,
        tim: TIM,
    }
    /// Interrupt events
    pub enum Event {
        /// Timer timed out / count down ended
        TimeOut,
    }
    impl Timer<SYST> {
        /// Configures the SYST clock as a periodic count down timer
        pub fn syst<T>(mut syst: SYST, timeout: T, rcc: &Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            syst.set_clock_source(SystClkSource::Core);
            let mut timer = Timer {
                tim: syst,
                clocks: rcc.clocks,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: &Event) {
            match event {
                Event::TimeOut => self.tim.enable_interrupt(),
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: &Event) {
            match event {
                Event::TimeOut => self.tim.disable_interrupt(),
            }
        }
    }
    /// Use the systick as a timer
    ///
    /// Be aware that intervals less than 4 Hertz may not function properly
    impl CountDown for Timer<SYST> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            let rvr = self.clocks.sysclk().0 / timeout.into().0 - 1;
            if !(rvr < (1 << 24)) {
                ::core::panicking::panic("assertion failed: rvr < (1 << 24)")
            };
            self.tim.set_reload(rvr);
            self.tim.clear_current();
            self.tim.enable_counter();
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.has_wrapped() {
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }
    impl Periodic for Timer<SYST> {}
    use crate::pac::TIM1;
    impl Timer<TIM1> {
        /// Configures a TIM peripheral as a periodic count down timer
        pub fn tim1<T>(tim: TIM1, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.regs.apb2enr.modify(|_, w| w.tim1en().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());
            let mut timer = Timer {
                clocks: rcc.clocks,
                tim,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
            }
        }
        /// Releases the TIM peripheral
        pub fn release(self) -> TIM1 {
            let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            rcc.apb2enr.modify(|_, w| w.tim1en().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM1> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let frequency = timeout.into().0;
            let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                self.clocks.pclk().0
            } else {
                self.clocks.pclk().0 * 2
            };
            let ticks = tclk / frequency;
            let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| w.psc().bits(psc));
            let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
            self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM1> {}
    use crate::pac::TIM3;
    impl Timer<TIM3> {
        /// Configures a TIM peripheral as a periodic count down timer
        pub fn tim3<T>(tim: TIM3, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.regs.apb1enr.modify(|_, w| w.tim3en().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.tim3rst().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.tim3rst().clear_bit());
            let mut timer = Timer {
                clocks: rcc.clocks,
                tim,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
            }
        }
        /// Releases the TIM peripheral
        pub fn release(self) -> TIM3 {
            let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            rcc.apb1enr.modify(|_, w| w.tim3en().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM3> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let frequency = timeout.into().0;
            let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                self.clocks.pclk().0
            } else {
                self.clocks.pclk().0 * 2
            };
            let ticks = tclk / frequency;
            let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| w.psc().bits(psc));
            let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
            self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM3> {}
    use crate::pac::TIM14;
    impl Timer<TIM14> {
        /// Configures a TIM peripheral as a periodic count down timer
        pub fn tim14<T>(tim: TIM14, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.regs.apb1enr.modify(|_, w| w.tim14en().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.tim14rst().set_bit());
            rcc.regs.apb1rstr.modify(|_, w| w.tim14rst().clear_bit());
            let mut timer = Timer {
                clocks: rcc.clocks,
                tim,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
            }
        }
        /// Releases the TIM peripheral
        pub fn release(self) -> TIM14 {
            let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            rcc.apb1enr.modify(|_, w| w.tim14en().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM14> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let frequency = timeout.into().0;
            let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                self.clocks.pclk().0
            } else {
                self.clocks.pclk().0 * 2
            };
            let ticks = tclk / frequency;
            let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| w.psc().bits(psc));
            let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
            self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM14> {}
    use crate::pac::TIM16;
    impl Timer<TIM16> {
        /// Configures a TIM peripheral as a periodic count down timer
        pub fn tim16<T>(tim: TIM16, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.regs.apb2enr.modify(|_, w| w.tim16en().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim16rst().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim16rst().clear_bit());
            let mut timer = Timer {
                clocks: rcc.clocks,
                tim,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
            }
        }
        /// Releases the TIM peripheral
        pub fn release(self) -> TIM16 {
            let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            rcc.apb2enr.modify(|_, w| w.tim16en().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM16> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let frequency = timeout.into().0;
            let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                self.clocks.pclk().0
            } else {
                self.clocks.pclk().0 * 2
            };
            let ticks = tclk / frequency;
            let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| w.psc().bits(psc));
            let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
            self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM16> {}
    use crate::pac::TIM17;
    impl Timer<TIM17> {
        /// Configures a TIM peripheral as a periodic count down timer
        pub fn tim17<T>(tim: TIM17, timeout: T, rcc: &mut Rcc) -> Self
        where
            T: Into<Hertz>,
        {
            rcc.regs.apb2enr.modify(|_, w| w.tim17en().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim17rst().set_bit());
            rcc.regs.apb2rstr.modify(|_, w| w.tim17rst().clear_bit());
            let mut timer = Timer {
                clocks: rcc.clocks,
                tim,
            };
            timer.start(timeout);
            timer
        }
        /// Starts listening for an `event`
        pub fn listen(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().set_bit());
                }
            }
        }
        /// Stops listening for an `event`
        pub fn unlisten(&mut self, event: Event) {
            match event {
                Event::TimeOut => {
                    self.tim.dier.write(|w| w.uie().clear_bit());
                }
            }
        }
        /// Releases the TIM peripheral
        pub fn release(self) -> TIM17 {
            let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            rcc.apb2enr.modify(|_, w| w.tim17en().clear_bit());
            self.tim
        }
    }
    impl CountDown for Timer<TIM17> {
        type Time = Hertz;
        /// Start the timer with a `timeout`
        fn start<T>(&mut self, timeout: T)
        where
            T: Into<Hertz>,
        {
            self.tim.cr1.modify(|_, w| w.cen().clear_bit());
            self.tim.cnt.reset();
            let frequency = timeout.into().0;
            let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                self.clocks.pclk().0
            } else {
                self.clocks.pclk().0 * 2
            };
            let ticks = tclk / frequency;
            let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
            self.tim.psc.write(|w| w.psc().bits(psc));
            let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
            self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
            self.tim.cr1.modify(|_, w| w.cen().set_bit());
        }
        /// Return `Ok` if the timer has wrapped
        /// Automatically clears the flag and restarts the time
        fn wait(&mut self) -> nb::Result<(), Void> {
            if self.tim.sr.read().uif().bit_is_clear() {
                Err(nb::Error::WouldBlock)
            } else {
                self.tim.sr.modify(|_, w| w.uif().clear_bit());
                Ok(())
            }
        }
    }
    impl Periodic for Timer<TIM17> {}
    use crate::gpio::{AF0, AF1, AF2, AF4, AF5};
    use crate::gpio::{gpioa::*, gpiob::*, Alternate};
    pub trait PinC1<TIM> {}
    pub trait PinC2<TIM> {}
    pub trait PinC3<TIM> {}
    pub trait PinC4<TIM> {}
    impl PinC1<TIM1> for PA8<Alternate<AF2>> {}
    impl PinC2<TIM1> for PA9<Alternate<AF2>> {}
    impl PinC3<TIM1> for PA10<Alternate<AF2>> {}
    impl PinC4<TIM1> for PA11<Alternate<AF2>> {}
    impl PinC1<TIM3> for PA6<Alternate<AF1>> {}
    impl PinC2<TIM3> for PA7<Alternate<AF1>> {}
    impl PinC1<TIM3> for PB4<Alternate<AF1>> {}
    impl PinC2<TIM3> for PB5<Alternate<AF1>> {}
    impl PinC3<TIM3> for PB0<Alternate<AF1>> {}
    impl PinC4<TIM3> for PB1<Alternate<AF1>> {}
    impl PinC1<TIM14> for PA4<Alternate<AF4>> {}
    impl PinC1<TIM14> for PA7<Alternate<AF4>> {}
    impl PinC1<TIM14> for PB1<Alternate<AF0>> {}
    impl PinC1<TIM16> for PA6<Alternate<AF5>> {}
    impl PinC1<TIM16> for PB8<Alternate<AF2>> {}
    impl PinC1<TIM17> for PA7<Alternate<AF5>> {}
    impl PinC1<TIM17> for PB9<Alternate<AF2>> {}
    #[cfg(any(
        feature = "stm32f030",
        feature = "stm32f051",
        feature = "stm32f058",
        feature = "stm32f070",
        feature = "stm32f071",
        feature = "stm32f072",
        feature = "stm32f078",
        feature = "stm32f091",
        feature = "stm32f098"
    ))]
    use crate::gpio::gpioc::*;
    impl PinC1<TIM3> for PC6<Alternate<AF0>> {}
    impl PinC2<TIM3> for PC7<Alternate<AF0>> {}
    impl PinC3<TIM3> for PC8<Alternate<AF0>> {}
    impl PinC4<TIM3> for PC9<Alternate<AF0>> {}
}
#[cfg(feature = "device-selected")]
pub mod watchdog {
    //! API for the IWDG
    //!
    //! You can activate the watchdog by calling `start` or the setting appropriate
    //! device option bit when programming.
    //!
    //! After activating the watchdog, you'll have to regularly `feed` the watchdog.
    //! If more time than `timeout` has gone by since the last `feed`, your
    //! microcontroller will be reset.
    //!
    //! This is useful if you fear that your program may get stuck. In that case it
    //! won't feed the watchdog anymore, the watchdog will reset the microcontroller
    //! and thus your program will function again.
    //!
    //! **Attention**:
    //!
    //! The IWDG runs on a separate 40kHz low-accuracy clock (30kHz-60kHz). You may
    //! want to some buffer in your interval.
    //!
    //! Per default the iwdg continues to run even when you stopped execution of code via a debugger.
    //! You may want to disable the watchdog when the cpu is stopped
    //!
    //! ``` ignore
    //! let dbgmcu = p.DBGMCU;
    //! dbgmcu.apb1_fz.modify(|_, w| w.dbg_iwdg_stop().set_bit());
    //! ```
    //!
    //! # Example
    //! ``` no_run
    //! use stm32f0xx_hal as hal;
    //!
    //! use crate::hal::pac;
    //! use crate::hal::prelude::*;
    //! use crate::hal:watchdog::Watchdog;
    //! use crate::hal:time::Hertz;
    //!
    //! let mut p = pac::Peripherals::take().unwrap();
    //!
    //! let mut iwdg = Watchdog::new(p.iwdg);
    //! iwdg.start(Hertz(100));
    //! loop {}
    //! // Whoops, got stuck, the watchdog issues a reset after 10 ms
    //! iwdg.feed();
    //! ```
    use embedded_hal::watchdog;
    use crate::pac::IWDG;
    use crate::time::Hertz;
    /// Watchdog instance
    pub struct Watchdog {
        iwdg: IWDG,
    }
    impl watchdog::Watchdog for Watchdog {
        /// Feed the watchdog, so that at least one `period` goes by before the next
        /// reset
        fn feed(&mut self) {
            self.iwdg.kr.write(|w| w.key().reset());
        }
    }
    /// Timeout configuration for the IWDG
    pub struct IwdgTimeout {
        psc: u8,
        reload: u16,
    }
    impl ::core::marker::StructuralPartialEq for IwdgTimeout {}
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialEq for IwdgTimeout {
        #[inline]
        fn eq(&self, other: &IwdgTimeout) -> bool {
            match *other {
                IwdgTimeout {
                    psc: ref __self_1_0,
                    reload: ref __self_1_1,
                } => match *self {
                    IwdgTimeout {
                        psc: ref __self_0_0,
                        reload: ref __self_0_1,
                    } => (*__self_0_0) == (*__self_1_0) && (*__self_0_1) == (*__self_1_1),
                },
            }
        }
        #[inline]
        fn ne(&self, other: &IwdgTimeout) -> bool {
            match *other {
                IwdgTimeout {
                    psc: ref __self_1_0,
                    reload: ref __self_1_1,
                } => match *self {
                    IwdgTimeout {
                        psc: ref __self_0_0,
                        reload: ref __self_0_1,
                    } => (*__self_0_0) != (*__self_1_0) || (*__self_0_1) != (*__self_1_1),
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::cmp::PartialOrd for IwdgTimeout {
        #[inline]
        fn partial_cmp(
            &self,
            other: &IwdgTimeout,
        ) -> ::core::option::Option<::core::cmp::Ordering> {
            match *other {
                IwdgTimeout {
                    psc: ref __self_1_0,
                    reload: ref __self_1_1,
                } => match *self {
                    IwdgTimeout {
                        psc: ref __self_0_0,
                        reload: ref __self_0_1,
                    } => match ::core::cmp::PartialOrd::partial_cmp(&(*__self_0_0), &(*__self_1_0))
                    {
                        ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                            match ::core::cmp::PartialOrd::partial_cmp(
                                &(*__self_0_1),
                                &(*__self_1_1),
                            ) {
                                ::core::option::Option::Some(::core::cmp::Ordering::Equal) => {
                                    ::core::option::Option::Some(::core::cmp::Ordering::Equal)
                                }
                                cmp => cmp,
                            }
                        }
                        cmp => cmp,
                    },
                },
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::clone::Clone for IwdgTimeout {
        #[inline]
        fn clone(&self) -> IwdgTimeout {
            {
                let _: ::core::clone::AssertParamIsClone<u8>;
                let _: ::core::clone::AssertParamIsClone<u16>;
                *self
            }
        }
    }
    #[automatically_derived]
    #[allow(unused_qualifications)]
    impl ::core::marker::Copy for IwdgTimeout {}
    impl Into<IwdgTimeout> for Hertz {
        /// This converts the value so it's usable by the IWDG
        /// Due to conversion losses, the specified frequency is a maximum
        ///
        /// It can also only represent values < 10000 Hertz
        fn into(self) -> IwdgTimeout {
            let mut time = 40_000 / 4 / self.0;
            let mut psc = 0;
            let mut reload = 0;
            while psc < 7 {
                reload = time;
                if reload < 0x1000 {
                    break;
                }
                psc += 1;
                time /= 2;
            }
            let reload = reload as u16;
            IwdgTimeout { psc, reload }
        }
    }
    impl Watchdog {
        pub fn new(iwdg: IWDG) -> Self {
            Self { iwdg }
        }
    }
    impl watchdog::WatchdogEnable for Watchdog {
        type Time = IwdgTimeout;
        fn start<T>(&mut self, period: T)
        where
            T: Into<IwdgTimeout>,
        {
            let time: IwdgTimeout = period.into();
            self.iwdg.kr.write(|w| w.key().reset());
            self.iwdg.kr.write(|w| w.key().start());
            self.iwdg.kr.write(|w| w.key().enable());
            while self.iwdg.sr.read().pvu().bit() {}
            self.iwdg.pr.write(|w| w.pr().bits(time.psc));
            while self.iwdg.sr.read().rvu().bit() {}
            self.iwdg.rlr.write(|w| w.rl().bits(time.reload));
            while self.iwdg.sr.read().bits() != 0 {}
            self.iwdg.kr.write(|w| w.key().reset());
        }
    }
}
#[cfg(feature = "device-selected")]
#[deprecated(since = "0.17.0", note = "please use `pac` instead")]
pub use pac as stm32;
