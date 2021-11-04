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
    }