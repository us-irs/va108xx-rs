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