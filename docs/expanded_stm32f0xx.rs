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
