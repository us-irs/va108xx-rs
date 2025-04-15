pub use vorago_shared_periphs::gpio::{Pin, PinIdProvider, Port};

use crate::{sysconfig::reset_peripheral_for_cycles, PeripheralSelect};

macro_rules! pin_id {
    ($Id:ident, $Port:path, $num:literal) => {
        // Need paste macro to use ident in doc attribute
        paste::paste! {
            #[doc = "Pin ID representing pin " $Id]
            #[derive(Debug)]
            pub enum $Id {}

            impl $crate::sealed::Sealed for $Id {}
            impl PinId for $Id {
                const PORT: Port = $Port;
                const OFFSET: usize = $num;
            }
        }
    };
}

impl<I: PinIdProvider + crate::sealed::Sealed> crate::sealed::Sealed for Pin<I> {}

pin_id!(Pa0, Port::A, 0);
pin_id!(Pa1, Port::A, 1);
pin_id!(Pa2, Port::A, 2);
pin_id!(Pa3, Port::A, 3);
pin_id!(Pa4, Port::A, 4);
pin_id!(Pa5, Port::A, 5);
pin_id!(Pa6, Port::A, 6);
pin_id!(Pa7, Port::A, 7);
pin_id!(Pa8, Port::A, 8);
pin_id!(Pa9, Port::A, 9);
pin_id!(Pa10, Port::A, 10);
pin_id!(Pa11, Port::A, 11);
pin_id!(Pa12, Port::A, 12);
pin_id!(Pa13, Port::A, 13);
pin_id!(Pa14, Port::A, 14);
pin_id!(Pa15, Port::A, 15);
pin_id!(Pa16, Port::A, 16);
pin_id!(Pa17, Port::A, 17);
pin_id!(Pa18, Port::A, 18);
pin_id!(Pa19, Port::A, 19);
pin_id!(Pa20, Port::A, 20);
pin_id!(Pa21, Port::A, 21);
pin_id!(Pa22, Port::A, 22);
pin_id!(Pa23, Port::A, 23);
pin_id!(Pa24, Port::A, 24);
pin_id!(Pa25, Port::A, 25);
pin_id!(Pa26, Port::A, 26);
pin_id!(Pa27, Port::A, 27);
pin_id!(Pa28, Port::A, 28);
pin_id!(Pa29, Port::A, 29);
pin_id!(Pa30, Port::A, 30);
pin_id!(Pa31, Port::A, 31);

pin_id!(Pb0, Port::B, 0);
pin_id!(Pb1, Port::B, 1);
pin_id!(Pb2, Port::B, 2);
pin_id!(Pb3, Port::B, 3);
pin_id!(Pb4, Port::B, 4);
pin_id!(Pb5, Port::B, 5);
pin_id!(Pb6, Port::B, 6);
pin_id!(Pb7, Port::B, 7);
pin_id!(Pb8, Port::B, 8);
pin_id!(Pb9, Port::B, 9);
pin_id!(Pb10, Port::B, 10);
pin_id!(Pb11, Port::B, 11);
pin_id!(Pb12, Port::B, 12);
pin_id!(Pb13, Port::B, 13);
pin_id!(Pb14, Port::B, 14);
pin_id!(Pb15, Port::B, 15);
pin_id!(Pb16, Port::B, 16);
pin_id!(Pb17, Port::B, 17);
pin_id!(Pb18, Port::B, 18);
pin_id!(Pb19, Port::B, 19);
pin_id!(Pb20, Port::B, 20);
pin_id!(Pb21, Port::B, 21);
pin_id!(Pb22, Port::B, 22);
pin_id!(Pb23, Port::B, 23);

pub struct PinsA {
    pub pa0: Pin<Pa0>,
    pub pa1: Pin<Pa1>,
    pub pa2: Pin<Pa2>,
    pub pa3: Pin<Pa3>,
    pub pa4: Pin<Pa4>,
    pub pa5: Pin<Pa5>,
    pub pa6: Pin<Pa6>,
    pub pa7: Pin<Pa7>,
    pub pa8: Pin<Pa8>,
    pub pa9: Pin<Pa9>,
    pub pa10: Pin<Pa10>,
    pub pa11: Pin<Pa11>,
    pub pa12: Pin<Pa12>,
    pub pa13: Pin<Pa13>,
    pub pa14: Pin<Pa14>,
    pub pa15: Pin<Pa15>,
    pub pa16: Pin<Pa16>,
    pub pa17: Pin<Pa17>,
    pub pa18: Pin<Pa18>,
    pub pa19: Pin<Pa19>,
    pub pa20: Pin<Pa20>,
    pub pa21: Pin<Pa21>,
    pub pa22: Pin<Pa22>,
    pub pa23: Pin<Pa23>,
    pub pa24: Pin<Pa24>,
    pub pa25: Pin<Pa25>,
    pub pa26: Pin<Pa26>,
    pub pa27: Pin<Pa27>,
    pub pa28: Pin<Pa28>,
    pub pa29: Pin<Pa29>,
    pub pa30: Pin<Pa30>,
    pub pa31: Pin<Pa31>,
}

impl PinsA {
    pub fn new(_port_a: va108xx::Porta) -> Self {
        let syscfg = unsafe { va108xx::Sysconfig::steal() };
        reset_peripheral_for_cycles(PeripheralSelect::PortA, 2);
        syscfg.peripheral_clk_enable().modify(|_, w| {
            w.porta().set_bit();
            w.gpio().set_bit();
            w.ioconfig().set_bit()
        });
        Self {
            pa0: Pin::new(),
            pa1: Pin::new(),
            pa2: Pin::new(),
            pa3: Pin::new(),
            pa4: Pin::new(),
            pa5: Pin::new(),
            pa6: Pin::new(),
            pa7: Pin::new(),
            pa8: Pin::new(),
            pa9: Pin::new(),
            pa10: Pin::new(),
            pa11: Pin::new(),
            pa12: Pin::new(),
            pa13: Pin::new(),
            pa14: Pin::new(),
            pa15: Pin::new(),
            pa16: Pin::new(),
            pa17: Pin::new(),
            pa18: Pin::new(),
            pa19: Pin::new(),
            pa20: Pin::new(),
            pa21: Pin::new(),
            pa22: Pin::new(),
            pa23: Pin::new(),
            pa24: Pin::new(),
            pa25: Pin::new(),
            pa26: Pin::new(),
            pa27: Pin::new(),
            pa28: Pin::new(),
            pa29: Pin::new(),
            pa30: Pin::new(),
            pa31: Pin::new(),
        }
    }
}

pub struct PinsB {
    pub pb0: Pin<Pb0>,
    pub pb1: Pin<Pb1>,
    pub pb2: Pin<Pb2>,
    pub pb3: Pin<Pb3>,
    pub pb4: Pin<Pb4>,
    pub pb5: Pin<Pb5>,
    pub pb6: Pin<Pb6>,
    pub pb7: Pin<Pb7>,
    pub pb8: Pin<Pb8>,
    pub pb9: Pin<Pb9>,
    pub pb10: Pin<Pb10>,
    pub pb11: Pin<Pb11>,
    pub pb12: Pin<Pb12>,
    pub pb13: Pin<Pb13>,
    pub pb14: Pin<Pb14>,
    pub pb15: Pin<Pb15>,
    pub pb16: Pin<Pb16>,
    pub pb17: Pin<Pb17>,
    pub pb18: Pin<Pb18>,
    pub pb19: Pin<Pb19>,
    pub pb20: Pin<Pb20>,
    pub pb21: Pin<Pb21>,
    pub pb22: Pin<Pb22>,
    pub pb23: Pin<Pb23>,
}

impl PinsB {
    pub fn new(_port_b: va108xx::Portb) -> Self {
        let syscfg = unsafe { va108xx::Sysconfig::steal() };
        reset_peripheral_for_cycles(PeripheralSelect::PortB, 2);
        syscfg.peripheral_clk_enable().modify(|_, w| {
            w.portb().set_bit();
            w.gpio().set_bit();
            w.ioconfig().set_bit()
        });
        Self {
            pb0: Pin::new(),
            pb1: Pin::new(),
            pb2: Pin::new(),
            pb3: Pin::new(),
            pb4: Pin::new(),
            pb5: Pin::new(),
            pb6: Pin::new(),
            pb7: Pin::new(),
            pb8: Pin::new(),
            pb9: Pin::new(),
            pb10: Pin::new(),
            pb11: Pin::new(),
            pb12: Pin::new(),
            pb13: Pin::new(),
            pb14: Pin::new(),
            pb15: Pin::new(),
            pb16: Pin::new(),
            pb17: Pin::new(),
            pb18: Pin::new(),
            pb19: Pin::new(),
            pb20: Pin::new(),
            pb21: Pin::new(),
            pb22: Pin::new(),
            pb23: Pin::new(),
        }
    }
}
