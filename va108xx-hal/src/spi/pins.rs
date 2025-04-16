use vorago_shared_periphs::gpio::{PinId, PinIdProvider};
use vorago_shared_periphs::FunSel;

use crate::{
    pins::{
        Pa10, Pa11, Pa12, Pa13, Pa14, Pa15, Pa16, Pa17, Pa18, Pa19, Pa20, Pa21, Pa22, Pa23, Pa24,
        Pa25, Pa26, Pa27, Pa28, Pa29, Pa30, Pa31, Pb0, Pb1, Pb10, Pb11, Pb12, Pb13, Pb14, Pb15,
        Pb16, Pb17, Pb18, Pb19, Pb2, Pb22, Pb23, Pb3, Pb4, Pb5, Pb6, Pb7, Pb8, Pb9, Pin, PinMarker,
    },
    sealed::Sealed,
};

use super::{HwChipSelectId, SpiId};

pub trait PinSck: PinMarker {
    const SPI_ID: SpiId;
    const FUN_SEL: FunSel;
}

pub trait PinMosi: PinMarker {
    const SPI_ID: SpiId;
    const FUN_SEL: FunSel;
}

pub trait PinMiso: PinMarker {
    const SPI_ID: SpiId;
    const FUN_SEL: FunSel;
}

pub trait HwCsProvider: PinMarker {
    const SPI_ID: SpiId;
    const FUN_SEL: FunSel;
    const CS_ID: HwChipSelectId;
}

macro_rules! hw_cs_pins {
    ($SpiId:path, $(($Px:ident, $FunSel:path, $HwCsIdent:path),)+) => {
        $(
            impl HwCsProvider for Pin<$Px> {
                const SPI_ID: SpiId = $SpiId;
                const FUN_SEL: FunSel = $FunSel;
                const CS_ID: HwChipSelectId = $HwCsIdent;
            }
        )+
    };
}
macro_rules! hw_cs_multi_pin {
    (
        // name of the newtype wrapper struct
        $name:ident,
        // Pb0
        $pin_id:ident,
        // SpiId::B
        $spi_id:path,
        // FunSel::Sel1
        $fun_sel:path,
        // HwChipSelectId::Id2
        $cs_id:path
    ) => {
        #[doc = concat!(
                                            "Newtype wrapper to use [Pin] [`", stringify!($pin_ty),
                                            "`] as a HW CS pin for [`", stringify!($spi_id),
                                            "`] with [`", stringify!($cs_id), "`]."
                                        )]
        pub struct $name(Pin<$pin_id>);

        impl $name {
            pub fn new(pin: Pin<$pin_id>) -> Self {
                Self(pin)
            }
        }

        impl Sealed for $name {}

        impl PinMarker for $name {
            const ID: PinId = <$pin_id as PinIdProvider>::ID;
        }

        impl HwCsProvider for $name {
            const SPI_ID: SpiId = $spi_id;
            const FUN_SEL: FunSel = $fun_sel;
            const CS_ID: HwChipSelectId = $cs_id;
        }
    };
}

// SPIA

impl PinSck for Pin<Pa31> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMosi for Pin<Pa30> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMiso for Pin<Pa29> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel1;
}

pub type SpiAPortASck = Pin<Pa31>;
pub type SpiAPortAMosi = Pin<Pa30>;
pub type SpiAPortAMiso = Pin<Pa29>;

impl PinSck for Pin<Pb9> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel2;
}
impl PinMosi for Pin<Pb8> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel2;
}
impl PinMiso for Pin<Pb7> {
    const SPI_ID: SpiId = SpiId::A;
    const FUN_SEL: FunSel = FunSel::Sel2;
}

pub type SpiAPortBSck = Pin<Pb9>;
pub type SpiAPortBMosi = Pin<Pb8>;
pub type SpiAPortBMiso = Pin<Pb7>;

hw_cs_pins!(
    SpiId::A,
    (Pb0, FunSel::Sel2, HwChipSelectId::Id1),
    (Pb1, FunSel::Sel2, HwChipSelectId::Id2),
    (Pb2, FunSel::Sel2, HwChipSelectId::Id3),
    (Pb3, FunSel::Sel2, HwChipSelectId::Id4),
    (Pb4, FunSel::Sel2, HwChipSelectId::Id5),
    (Pb5, FunSel::Sel2, HwChipSelectId::Id6),
    (Pb6, FunSel::Sel2, HwChipSelectId::Id0),
    (Pa24, FunSel::Sel1, HwChipSelectId::Id4),
    (Pa25, FunSel::Sel1, HwChipSelectId::Id3),
    (Pa26, FunSel::Sel1, HwChipSelectId::Id2),
    (Pa27, FunSel::Sel1, HwChipSelectId::Id1),
    (Pa28, FunSel::Sel1, HwChipSelectId::Id0),
);

hw_cs_multi_pin!(
    PinPb0SpiaHwCsId1,
    Pb0,
    SpiId::A,
    FunSel::Sel2,
    HwChipSelectId::Id1
);
hw_cs_multi_pin!(
    PinPb1SpiaHwCsId2,
    Pb1,
    SpiId::A,
    FunSel::Sel2,
    HwChipSelectId::Id2
);
hw_cs_multi_pin!(
    PinPb2SpiaHwCsId3,
    Pb2,
    SpiId::A,
    FunSel::Sel2,
    HwChipSelectId::Id3
);

hw_cs_multi_pin!(
    PinPa21SpiaHwCsId7,
    Pa21,
    SpiId::A,
    FunSel::Sel1,
    HwChipSelectId::Id7
);
hw_cs_multi_pin!(
    PinPa22SpiaHwCsId6,
    Pa22,
    SpiId::A,
    FunSel::Sel1,
    HwChipSelectId::Id6
);
hw_cs_multi_pin!(
    PinPa23SpiaHwCsId5,
    Pa23,
    SpiId::A,
    FunSel::Sel1,
    HwChipSelectId::Id5
);

// SPIB

impl PinSck for Pin<Pa20> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel2;
}
impl PinMosi for Pin<Pa19> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel2;
}
impl PinMiso for Pin<Pa18> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel2;
}

pub type SpiBPortASck = Pin<Pa20>;
pub type SpiBPortAMosi = Pin<Pa19>;
pub type SpiBPortAMiso = Pin<Pa18>;

impl PinSck for Pin<Pb19> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMosi for Pin<Pb18> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMiso for Pin<Pb17> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}

impl PinSck for Pin<Pb5> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMosi for Pin<Pb4> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}
impl PinMiso for Pin<Pb3> {
    const SPI_ID: SpiId = SpiId::B;
    const FUN_SEL: FunSel = FunSel::Sel1;
}

// TODO: Need to deal with these duplications..
hw_cs_pins!(
    SpiId::B,
    (Pb16, FunSel::Sel1, HwChipSelectId::Id0),
    (Pb15, FunSel::Sel1, HwChipSelectId::Id1),
    (Pb14, FunSel::Sel1, HwChipSelectId::Id2),
    (Pb13, FunSel::Sel1, HwChipSelectId::Id3),
    (Pa17, FunSel::Sel2, HwChipSelectId::Id0),
    (Pa16, FunSel::Sel2, HwChipSelectId::Id1),
    (Pa15, FunSel::Sel2, HwChipSelectId::Id2),
    (Pa14, FunSel::Sel2, HwChipSelectId::Id3),
    (Pa13, FunSel::Sel2, HwChipSelectId::Id4),
    (Pa12, FunSel::Sel2, HwChipSelectId::Id5),
    (Pa11, FunSel::Sel2, HwChipSelectId::Id6),
    (Pa10, FunSel::Sel2, HwChipSelectId::Id7),
    (Pa23, FunSel::Sel2, HwChipSelectId::Id5),
    (Pa22, FunSel::Sel2, HwChipSelectId::Id6),
    (Pa21, FunSel::Sel2, HwChipSelectId::Id7),
);

hw_cs_multi_pin!(
    PinPb0SpibHwCsId2,
    Pb0,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id2
);
hw_cs_multi_pin!(
    PinPb1SpibHwCsId1,
    Pb1,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id1
);
hw_cs_multi_pin!(
    PinPb2SpibHwCsId0,
    Pb2,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id0
);

hw_cs_multi_pin!(
    PinPb10SpibHwCsId6,
    Pb10,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id6
);
hw_cs_multi_pin!(
    PinPb11SpibHwCsId5,
    Pb11,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id5
);
hw_cs_multi_pin!(
    PinPb12SpibHwCsId4,
    Pb12,
    SpiId::B,
    FunSel::Sel1,
    HwChipSelectId::Id4
);

hw_cs_multi_pin!(
    PinPb10SpibHwCsId2,
    Pb10,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id2
);
hw_cs_multi_pin!(
    PinPb11SpibHwCsId1,
    Pb11,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id1
);
hw_cs_multi_pin!(
    PinPb12SpibHwCsId0,
    Pb12,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id0
);

hw_cs_multi_pin!(
    PinPa21SpibHwCsId7,
    Pa21,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id7
);
hw_cs_multi_pin!(
    PinPa22SpibHwCsId6,
    Pa22,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id6
);
hw_cs_multi_pin!(
    PinPa23SpibHwCsId5,
    Pa23,
    SpiId::B,
    FunSel::Sel2,
    HwChipSelectId::Id5
);

// SPIC

/*
//  Dummy pin defintion for the ROM SCK.
pub struct RomSck;
//  Dummy pin defintion for the ROM MOSI.
pub struct RomMosi;
//  Dummy pin defintion for the ROM MISO.
pub struct RomMiso;
//  Dummy pin defintion for the ROM chip select.
pub struct RomCs;

impl Sealed for RomSck {}
impl PinSck for RomSck {
    const SPI_ID: SpiId = SpiId::C;
    /// Function select does not make sense here, just select default value.
    const FUN_SEL: FunSel = FunSel::Sel0;
}
impl Sealed for RomMosi {}
impl PinMosi for RomMosi {
    const SPI_ID: SpiId = SpiId::C;
    /// Function select does not make sense here, just select default value.
    const FUN_SEL: FunSel = FunSel::Sel0;
}
impl Sealed for RomMiso {}
impl PinMiso for RomMiso {
    const SPI_ID: SpiId = SpiId::C;
    /// Function select does not make sense here, just select default value.
    const FUN_SEL: FunSel = FunSel::Sel0;
}
impl Sealed for RomCs {}
*/

hw_cs_pins!(
    SpiId::C,
    (Pb9, FunSel::Sel3, HwChipSelectId::Id1),
    (Pb8, FunSel::Sel3, HwChipSelectId::Id2),
    (Pb7, FunSel::Sel3, HwChipSelectId::Id3),
    (Pb23, FunSel::Sel3, HwChipSelectId::Id2),
    (Pb22, FunSel::Sel3, HwChipSelectId::Id1),
    (Pa20, FunSel::Sel1, HwChipSelectId::Id1),
    (Pa19, FunSel::Sel1, HwChipSelectId::Id2),
    (Pb18, FunSel::Sel1, HwChipSelectId::Id3),
);

hw_cs_multi_pin!(
    PinPa21SpicHwCsId3,
    Pa21,
    SpiId::C,
    FunSel::Sel3,
    HwChipSelectId::Id3
);
hw_cs_multi_pin!(
    PinPa22SpicHwCsId2,
    Pa22,
    SpiId::C,
    FunSel::Sel3,
    HwChipSelectId::Id2
);
hw_cs_multi_pin!(
    PinPa23SpicHwCsId1,
    Pa23,
    SpiId::C,
    FunSel::Sel3,
    HwChipSelectId::Id1
);

hw_cs_multi_pin!(
    PinPa20SpicHwCsId1,
    Pa20,
    SpiId::C,
    FunSel::Sel1,
    HwChipSelectId::Id1
);
hw_cs_multi_pin!(
    PinPa20SpicHwCsId4,
    Pa20,
    SpiId::C,
    FunSel::Sel3,
    HwChipSelectId::Id4
);

/*
impl HwCsProvider for RomCs {
    const CS_ID: HwChipSelectId = HwChipSelectId::Id0;
    const SPI_ID: SpiId = SpiId::C;
    /// Function select does not make sense here, just select default value.
    const FUN_SEL: FunSel = FunSel::Sel0;
}
*/
