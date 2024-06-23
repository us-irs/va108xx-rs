//! API for the SPI peripheral
//!
//! ## Examples
//!
//! - [Blocking SPI example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/spi.rs)
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    gpio::pins::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA10, PA11, PA12, PA13, PA14, PA15, PA16, PA17, PA18,
        PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31, PB0, PB1,
        PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB2, PB22, PB23, PB3, PB4, PB5,
        PB6, PB7, PB8, PB9,
    },
    pac,
    time::Hertz,
    typelevel::Sealed,
};
use core::{convert::Infallible, fmt::Debug, marker::PhantomData};
use embedded_hal::spi::{Mode, SpiBus, MODE_0, MODE_1, MODE_2, MODE_3};

//==================================================================================================
// Defintions
//==================================================================================================

// FIFO has a depth of 16.
const FILL_DEPTH: usize = 12;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum HwChipSelectId {
    Id0 = 0,
    Id1 = 1,
    Id2 = 2,
    Id3 = 3,
    Id4 = 4,
    Id5 = 5,
    Id6 = 6,
    Id7 = 7,
    Invalid = 0xff,
}

#[derive(Debug)]
pub enum SpiPort {
    Porta = 0,
    Portb = 1,
    Portc = 2,
    Invalid = 3,
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum WordSize {
    OneBit = 0x00,
    FourBits = 0x03,
    EightBits = 0x07,
    SixteenBits = 0x0f,
}

//==================================================================================================
// Pin type definitions
//==================================================================================================

pub trait PinSck<SPI>: Sealed {}
pub trait PinMosi<SPI>: Sealed {}
pub trait PinMiso<SPI>: Sealed {}

pub trait HwCsProvider: Sealed {
    const CS_ID: HwChipSelectId;
    const SPI_PORT: SpiPort;
}

pub trait OptionalHwCs<Spi>: HwCsProvider + Sealed {}

macro_rules! hw_cs_pin {
    ($SPIx:path, $portId: path, $PXx:ident, $AFx:ident, $HwCsIdent:path, $typedef:ident) => {
        impl HwCsProvider for Pin<$PXx, $AFx> {
            const CS_ID: HwChipSelectId = $HwCsIdent;
            const SPI_PORT: SpiPort = $portId;
        }
        impl OptionalHwCs<$SPIx> for Pin<$PXx, $AFx> {}
        pub type $typedef = Pin<$PXx, $AFx>;
    };
}

impl HwCsProvider for NoneT {
    const CS_ID: HwChipSelectId = HwChipSelectId::Invalid;
    const SPI_PORT: SpiPort = SpiPort::Invalid;
}

impl OptionalHwCs<pac::Spia> for NoneT {}
impl OptionalHwCs<pac::Spib> for NoneT {}

// SPIA

impl PinSck<pac::Spia> for Pin<PA31, AltFunc1> {}
impl PinMosi<pac::Spia> for Pin<PA30, AltFunc1> {}
impl PinMiso<pac::Spia> for Pin<PA29, AltFunc1> {}

pub type SpiAPortASck = Pin<PA31, AltFunc1>;
pub type SpiAPortAMosi = Pin<PA30, AltFunc1>;
pub type SpiAPortAMiso = Pin<PA29, AltFunc1>;

impl PinSck<pac::Spia> for Pin<PB9, AltFunc2> {}
impl PinMosi<pac::Spia> for Pin<PB8, AltFunc2> {}
impl PinMiso<pac::Spia> for Pin<PB7, AltFunc2> {}

pub type SpiAPortBSck = Pin<PB9, AltFunc2>;
pub type SpiAPortBMosi = Pin<PB8, AltFunc2>;
pub type SpiAPortBMiso = Pin<PB7, AltFunc2>;

hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA28,
    AltFunc1,
    HwChipSelectId::Id0,
    HwCs0SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA27,
    AltFunc1,
    HwChipSelectId::Id1,
    HwCs1SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA26,
    AltFunc1,
    HwChipSelectId::Id2,
    HwCs2SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA25,
    AltFunc1,
    HwChipSelectId::Id3,
    HwCs3SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA24,
    AltFunc1,
    HwChipSelectId::Id4,
    HwCs4SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA23,
    AltFunc1,
    HwChipSelectId::Id5,
    HwCs5SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA22,
    AltFunc1,
    HwChipSelectId::Id6,
    HwCs6SpiAPortA
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PA21,
    AltFunc1,
    HwChipSelectId::Id7,
    HwCs7SpiAPortA
);

hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB6,
    AltFunc2,
    HwChipSelectId::Id0,
    HwCs0SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB5,
    AltFunc2,
    HwChipSelectId::Id6,
    HwCs6SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB4,
    AltFunc2,
    HwChipSelectId::Id5,
    HwCs5SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB3,
    AltFunc2,
    HwChipSelectId::Id4,
    HwCs4SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB2,
    AltFunc2,
    HwChipSelectId::Id3,
    HwCs3SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB1,
    AltFunc2,
    HwChipSelectId::Id2,
    HwCs2SpiAPortB
);
hw_cs_pin!(
    pac::Spia,
    SpiPort::Porta,
    PB0,
    AltFunc2,
    HwChipSelectId::Id1,
    HwCs1SpiAPortB
);

// SPIB

impl PinSck<pac::Spib> for Pin<PA20, AltFunc2> {}
impl PinMosi<pac::Spib> for Pin<PA19, AltFunc2> {}
impl PinMiso<pac::Spib> for Pin<PA18, AltFunc2> {}

pub type SpiBPortASck = Pin<PA20, AltFunc2>;
pub type SpiBPortAMosi = Pin<PA19, AltFunc2>;
pub type SpiBPortAMiso = Pin<PA18, AltFunc2>;

impl PinSck<pac::Spib> for Pin<PB19, AltFunc1> {}
impl PinMosi<pac::Spib> for Pin<PB18, AltFunc1> {}
impl PinMiso<pac::Spib> for Pin<PB17, AltFunc1> {}

impl PinSck<pac::Spib> for Pin<PB5, AltFunc1> {}
impl PinMosi<pac::Spib> for Pin<PB4, AltFunc1> {}
impl PinMiso<pac::Spib> for Pin<PB3, AltFunc1> {}

hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB16,
    AltFunc1,
    HwChipSelectId::Id0,
    HwCs0SpiBPortB0
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB15,
    AltFunc1,
    HwChipSelectId::Id1,
    HwCs1SpiBPortB0
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB14,
    AltFunc1,
    HwChipSelectId::Id2,
    HwCs2SpiBPortB0
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB13,
    AltFunc1,
    HwChipSelectId::Id3,
    HwCs3SpiBPortB
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB12,
    AltFunc1,
    HwChipSelectId::Id4,
    HwCs4SpiBPortB
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB11,
    AltFunc1,
    HwChipSelectId::Id5,
    HwCs5SpiBPortB
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB10,
    AltFunc1,
    HwChipSelectId::Id6,
    HwCs6SpiBPortB
);

hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB2,
    AltFunc1,
    HwChipSelectId::Id0,
    HwCs0SpiBPortB1
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB1,
    AltFunc1,
    HwChipSelectId::Id1,
    HwCs1SpiBPortB1
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB0,
    AltFunc1,
    HwChipSelectId::Id2,
    HwCs2SpiBPortB1
);

hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB12,
    AltFunc2,
    HwChipSelectId::Id0,
    HwCs0SpiBPortB2
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB11,
    AltFunc2,
    HwChipSelectId::Id1,
    HwCs1SpiBPortB2
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PB10,
    AltFunc2,
    HwChipSelectId::Id2,
    HwCs2SpiBPortB2
);

hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA17,
    AltFunc2,
    HwChipSelectId::Id0,
    HwCs0SpiBPortA
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA16,
    AltFunc2,
    HwChipSelectId::Id1,
    HwCs1SpiBPortA
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA15,
    AltFunc2,
    HwChipSelectId::Id2,
    HwCs2SpiBPortA
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA14,
    AltFunc2,
    HwChipSelectId::Id3,
    HwCs3SpiBPortA
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA13,
    AltFunc2,
    HwChipSelectId::Id4,
    HwCs4SpiBPortA
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA12,
    AltFunc2,
    HwChipSelectId::Id5,
    HwCs5SpiBPortA0
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA11,
    AltFunc2,
    HwChipSelectId::Id6,
    HwCs6SpiBPortA0
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA10,
    AltFunc2,
    HwChipSelectId::Id7,
    HwCs7SpiBPortA0
);

hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA23,
    AltFunc2,
    HwChipSelectId::Id5,
    HwCs5SpiBPortA1
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA22,
    AltFunc2,
    HwChipSelectId::Id6,
    HwCs6SpiBPortA1
);
hw_cs_pin!(
    pac::Spib,
    SpiPort::Portb,
    PA21,
    AltFunc2,
    HwChipSelectId::Id7,
    HwCs7SpiBPortA1
);

// SPIC

hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB9,
    AltFunc3,
    HwChipSelectId::Id1,
    HwCs1SpiCPortB0
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB8,
    AltFunc3,
    HwChipSelectId::Id2,
    HwCs2SpiCPortB0
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB7,
    AltFunc3,
    HwChipSelectId::Id3,
    HwCs3SpiCPortB
);

hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB22,
    AltFunc3,
    HwChipSelectId::Id1,
    HwCs1SpiCPortB1
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB23,
    AltFunc3,
    HwChipSelectId::Id2,
    HwCs2SpiCPortB1
);

hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA20,
    AltFunc1,
    HwChipSelectId::Id1,
    HwCs1SpiCPortA0
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA19,
    AltFunc1,
    HwChipSelectId::Id2,
    HwCs2SpiCPortA0
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PB18,
    AltFunc1,
    HwChipSelectId::Id3,
    HwCs3SpiCPortA0
);

hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA23,
    AltFunc3,
    HwChipSelectId::Id1,
    HwCs1SpiCPortA1
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA22,
    AltFunc3,
    HwChipSelectId::Id2,
    HwCs2SpiCPortA1
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA21,
    AltFunc3,
    HwChipSelectId::Id3,
    HwCs3SpiCPortA1
);
hw_cs_pin!(
    pac::Spic,
    SpiPort::Portc,
    PA20,
    AltFunc3,
    HwChipSelectId::Id4,
    HwCs4SpiCPortA
);

//==================================================================================================
// Config
//==================================================================================================

pub trait GenericTransferConfig {
    fn sod(&mut self, sod: bool);
    fn blockmode(&mut self, blockmode: bool);
    fn mode(&mut self, mode: Mode);
    fn frequency(&mut self, spi_clk: Hertz);
    fn hw_cs_id(&self) -> u8;
}

/// This struct contains all configuration parameter which are transfer specific
/// and might change for transfers to different SPI slaves
#[derive(Copy, Clone)]
pub struct TransferConfig<HwCs> {
    pub spi_clk: Hertz,
    pub mode: Mode,
    /// This only works if the Slave Output Disable (SOD) bit of the [`SpiConfig`] is set to
    /// false
    pub hw_cs: Option<HwCs>,
    pub sod: bool,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words
    pub blockmode: bool,
}

/// Type erased variant of the transfer configuration. This is required to avoid generics in
/// the SPI constructor.
pub struct ReducedTransferConfig {
    pub spi_clk: Hertz,
    pub mode: Mode,
    pub sod: bool,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words
    pub blockmode: bool,
    pub hw_cs: HwChipSelectId,
}

impl TransferConfig<NoneT> {
    pub fn new_no_hw_cs(spi_clk: impl Into<Hertz>, mode: Mode, blockmode: bool, sod: bool) -> Self {
        TransferConfig {
            spi_clk: spi_clk.into(),
            mode,
            hw_cs: None,
            sod,
            blockmode,
        }
    }
}

impl<HwCs: HwCsProvider> TransferConfig<HwCs> {
    pub fn new(
        spi_clk: impl Into<Hertz>,
        mode: Mode,
        hw_cs: Option<HwCs>,
        blockmode: bool,
        sod: bool,
    ) -> Self {
        TransferConfig {
            spi_clk: spi_clk.into(),
            mode,
            hw_cs,
            sod,
            blockmode,
        }
    }

    pub fn downgrade(self) -> ReducedTransferConfig {
        ReducedTransferConfig {
            spi_clk: self.spi_clk,
            mode: self.mode,
            sod: self.sod,
            blockmode: self.blockmode,
            hw_cs: HwCs::CS_ID,
        }
    }
}

impl<HwCs: HwCsProvider> GenericTransferConfig for TransferConfig<HwCs> {
    /// Slave Output Disable
    fn sod(&mut self, sod: bool) {
        self.sod = sod;
    }

    fn blockmode(&mut self, blockmode: bool) {
        self.blockmode = blockmode;
    }

    fn mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    fn frequency(&mut self, spi_clk: Hertz) {
        self.spi_clk = spi_clk;
    }

    fn hw_cs_id(&self) -> u8 {
        HwCs::CS_ID as u8
    }
}

#[derive(Default)]
/// Configuration options for the whole SPI bus. See Programmer Guide p.92 for more details
pub struct SpiConfig {
    /// Serial clock rate divider. Together with the CLKPRESCALE register, it determines
    /// the SPI clock rate in master mode. 0 by default. Specifying a higher value
    /// limits the maximum attainable SPI speed
    pub scrdv: u8,
    /// By default, configure SPI for master mode (ms == false)
    ms: bool,
    /// Slave output disable. Useful if separate GPIO pins or decoders are used for CS control
    sod: bool,
    /// Loopback mode. If you use this, don't connect MISO to MOSI, they will be tied internally
    lbm: bool,
    /// Enable Master Delayer Capture Mode. See Programmers Guide p.92 for more details
    pub mdlycap: bool,
}

impl SpiConfig {
    pub fn loopback(mut self, enable: bool) -> Self {
        self.lbm = enable;
        self
    }

    pub fn master_mode(mut self, master: bool) -> Self {
        self.ms = !master;
        self
    }

    pub fn slave_output_disable(mut self, sod: bool) -> Self {
        self.sod = sod;
        self
    }
}

//==================================================================================================
// Word Size
//==================================================================================================

/// Configuration trait for the Word Size
/// used by the SPI peripheral
pub trait Word: Copy + Default {
    fn word_reg() -> u8;
}

impl Word for u8 {
    fn word_reg() -> u8 {
        0x07
    }
}

impl Word for u16 {
    fn word_reg() -> u8 {
        0x0f
    }
}

//==================================================================================================
// Spi
//==================================================================================================

pub struct SpiBase<SpiInstance, Word = u8> {
    spi: SpiInstance,
    cfg: SpiConfig,
    sys_clk: Hertz,
    /// Fill word for read-only SPI transactions.
    pub fill_word: Word,
    blockmode: bool,
    _word: PhantomData<Word>,
}

pub struct Spi<SpiInstance, Pins, Word = u8> {
    inner: SpiBase<SpiInstance, Word>,
    pins: Pins,
}

// Not sure if implementing SpiDevice is a good idea here..
/*
pub struct SpiWithHwCs<SpiInstance, HwCs, Pins, Word = u8> {
    inner: SpiBase<SpiInstance, Word>,
    pins: Pins,
    hw_cs: HwCs,
}

pub struct SpiWithHwCsErased<SpiInstance, Word = u8> {
    inner: SpiBase<SpiInstance, Word>,
}
*/

// Re-export this so it can be used for the constructor
pub use crate::typelevel::NoneT;

macro_rules! spi {
    ($($SPIX:path: ($spix:ident, $clk_enb:path) => ($($WORD:ident),+),)+) => {
        $(
            impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>,
                WORD: Word> Spi<$SPIX, (Sck, Miso, Mosi), WORD>
            {
                /// Create a new SPI struct
                ///
                /// You can delete the pin type information by calling the
                /// [`downgrade`](Self::downgrade) function
                ///
                /// ## Arguments
                /// * `spi` - SPI bus to use
                /// * `pins` - Pins to be used for SPI transactions. These pins are consumed
                ///     to ensure the pins can not be used for other purposes anymore
                /// * `spi_cfg` - Configuration specific to the SPI bus
                /// * `transfer_cfg` - Optional initial transfer configuration which includes
                ///     configuration which can change across individual SPI transfers like SPI mode
                ///     or SPI clock. If only one device is connected, this configuration only needs
                ///     to be done once.
                /// * `syscfg` - Can be passed optionally to enable the peripheral clock
                pub fn $spix(
                    spi: $SPIX,
                    pins: (Sck, Miso, Mosi),
                    sys_clk: impl Into<Hertz> + Copy,
                    spi_cfg: SpiConfig,
                    syscfg: Option<&mut pac::Sysconfig>,
                    transfer_cfg: Option<&ReducedTransferConfig>,
                ) -> Self {
                    if let Some(syscfg) = syscfg {
                        enable_peripheral_clock(syscfg, $clk_enb);
                    }
                    let SpiConfig {
                        scrdv,
                        ms,
                        sod,
                        lbm,
                        mdlycap,
                    } = spi_cfg;
                    let mut mode = MODE_0;
                    let mut clk_prescale = 0x02;
                    let mut ss = 0;
                    let mut init_blockmode = false;
                    if let Some(transfer_cfg) = transfer_cfg {
                        mode = transfer_cfg.mode;
                        clk_prescale = sys_clk.into().raw() / (transfer_cfg.spi_clk.raw() * (scrdv as u32 + 1));
                        if  transfer_cfg.hw_cs != HwChipSelectId::Invalid {
                            ss = transfer_cfg.hw_cs as u8;
                        }
                        init_blockmode = transfer_cfg.blockmode;
                    }

                    let (cpo_bit, cph_bit) = match mode {
                        MODE_0 => (false, false),
                        MODE_1 => (false, true),
                        MODE_2 => (true, false),
                        MODE_3 => (true, true),
                    };
                    spi.ctrl0().write(|w| {
                        unsafe {
                            w.size().bits(WORD::word_reg());
                            w.scrdv().bits(scrdv);
                            // Clear clock phase and polarity. Will be set to correct value for each
                            // transfer
                            w.spo().bit(cpo_bit);
                            w.sph().bit(cph_bit)
                        }
                    });
                    spi.ctrl1().write(|w| {
                        w.lbm().bit(lbm);
                        w.sod().bit(sod);
                        w.ms().bit(ms);
                        w.mdlycap().bit(mdlycap);
                        w.blockmode().bit(init_blockmode);
                        unsafe { w.ss().bits(ss) }
                    });

                    spi.fifo_clr().write(|w| {
                        w.rxfifo().set_bit();
                        w.txfifo().set_bit()
                    });
                    spi.clkprescale().write(|w| unsafe { w.bits(clk_prescale) });
                    // Enable the peripheral as the last step as recommended in the
                    // programmers guide
                    spi.ctrl1().modify(|_, w| w.enable().set_bit());
                    Spi {
                        inner: SpiBase {
                            spi,
                            cfg: spi_cfg,
                            sys_clk: sys_clk.into(),
                            fill_word: Default::default(),
                            blockmode: init_blockmode,
                            _word: PhantomData,
                        },
                        pins,
                    }
                }

                #[inline]
                pub fn cfg_clock(&mut self, spi_clk: impl Into<Hertz>) {
                    self.inner.cfg_clock(spi_clk);
                }

                #[inline]
                pub fn cfg_mode(&mut self, mode: Mode) {
                    self.inner.cfg_mode(mode);
                }

                pub fn set_fill_word(&mut self, fill_word: WORD) {
                    self.inner.fill_word = fill_word;
                }

                pub fn fill_word(&self) -> WORD {
                    self.inner.fill_word
                }

                #[inline]
                pub fn perid(&self) -> u32 {
                    self.inner.perid()
                }

                pub fn cfg_transfer<HwCs: OptionalHwCs<$SPIX>>(&mut self, transfer_cfg: &TransferConfig<HwCs>) {
                    self.inner.cfg_transfer(transfer_cfg);
                }

                /// Releases the SPI peripheral and associated pins
                pub fn release(self) -> ($SPIX, (Sck, Miso, Mosi), SpiConfig) {
                    (self.inner.spi, self.pins, self.inner.cfg)
                }

                pub fn downgrade(self) -> SpiBase<$SPIX, WORD> {
                    self.inner
                }
            }

            impl<WORD: Word> SpiBase<$SPIX, WORD> {
                #[inline]
                pub fn cfg_clock(&mut self, spi_clk: impl Into<Hertz>) {
                    let clk_prescale = self.sys_clk.raw() / (spi_clk.into().raw() * (self.cfg.scrdv as u32 + 1));
                    self.spi
                        .clkprescale()
                        .write(|w| unsafe { w.bits(clk_prescale) });
                }

                #[inline]
                pub fn cfg_mode(&mut self, mode: Mode) {
                    let (cpo_bit, cph_bit) = match mode {
                        MODE_0 => (false, false),
                        MODE_1 => (false, true),
                        MODE_2 => (true, false),
                        MODE_3 => (true, true),
                    };
                    self.spi.ctrl0().modify(|_, w| {
                        w.spo().bit(cpo_bit);
                        w.sph().bit(cph_bit)
                    });
                }

                #[inline]
                pub fn clear_tx_fifo(&self) {
                    self.spi.fifo_clr().write(|w| w.txfifo().set_bit());
                }

                #[inline]
                pub fn clear_rx_fifo(&self) {
                    self.spi.fifo_clr().write(|w| w.rxfifo().set_bit());
                }

                #[inline]
                pub fn perid(&self) -> u32 {
                    self.spi.perid().read().bits()
                }

                #[inline]
                pub fn cfg_hw_cs(&mut self, hw_cs: HwChipSelectId) {
                    if hw_cs == HwChipSelectId::Invalid {
                        return;
                    }
                    self.spi.ctrl1().modify(|_, w| {
                        w.sod().clear_bit();
                        unsafe {
                            w.ss().bits(hw_cs as u8);
                        }
                        w
                    });
                }

                #[inline]
                pub fn cfg_hw_cs_with_pin<HwCs: OptionalHwCs<$SPIX>>(&mut self, _: &HwCs) {
                    self.cfg_hw_cs(HwCs::CS_ID);
                }

                pub fn cfg_hw_cs_disable(&mut self) {
                    self.spi.ctrl1().modify(|_, w| {
                        w.sod().set_bit();
                        w
                    });
                }

                pub fn cfg_transfer<HwCs: OptionalHwCs<$SPIX>>(&mut self, transfer_cfg: &TransferConfig<HwCs>) {
                    self.cfg_clock(transfer_cfg.spi_clk);
                    self.cfg_mode(transfer_cfg.mode);
                    self.blockmode = transfer_cfg.blockmode;
                    self.spi.ctrl1().modify(|_, w| {
                        if transfer_cfg.sod {
                            w.sod().set_bit();
                        } else if transfer_cfg.hw_cs.is_some() {
                            w.sod().clear_bit();
                            unsafe {
                                w.ss().bits(HwCs::CS_ID as u8);
                            }
                        } else {
                            w.sod().clear_bit();
                        }
                        if transfer_cfg.blockmode {
                            w.blockmode().set_bit();
                        } else {
                            w.blockmode().clear_bit();
                        }
                        w
                    });
                }
            }

            /// Changing the word size also requires a type conversion
            impl <Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                From<Spi<$SPIX, (Sck, Miso, Mosi), u8>> for Spi<$SPIX, (Sck, Miso, Mosi), u16>
            {
                fn from(
                    old_spi: Spi<$SPIX, (Sck, Miso, Mosi), u8>
                ) -> Self {
                    old_spi.inner.spi.ctrl0().modify(|_, w| {
                        unsafe {
                            w.size().bits(WordSize::SixteenBits as u8)
                        }
                    });
                    Spi {
                        inner: SpiBase {
                            spi: old_spi.inner.spi,
                            cfg: old_spi.inner.cfg,
                            blockmode: old_spi.inner.blockmode,
                            fill_word: Default::default(),
                            sys_clk: old_spi.inner.sys_clk,
                            _word: PhantomData,
                        },
                        pins: old_spi.pins,
                    }
                }
            }

            /// Changing the word size also requires a type conversion
            impl <Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                From<Spi<$SPIX, (Sck, Miso, Mosi), u16>> for
                Spi<$SPIX, (Sck, Miso, Mosi), u8>
            {
                fn from(
                    old_spi: Spi<$SPIX, (Sck, Miso, Mosi), u16>
                ) -> Self {
                    old_spi.inner.spi.ctrl0().modify(|_, w| {
                        unsafe {
                            w.size().bits(WordSize::EightBits as u8)
                        }
                    });
                    Spi {
                        inner: SpiBase {
                            spi: old_spi.inner.spi,
                            cfg: old_spi.inner.cfg,
                            blockmode: old_spi.inner.blockmode,
                            sys_clk: old_spi.inner.sys_clk,
                            fill_word: Default::default(),
                            _word: PhantomData,
                        },
                        pins: old_spi.pins,
                    }
                }
            }

            $(

                impl SpiBase<$SPIX, $WORD> {
                    /// Sends a word to the slave
                    #[inline(always)]
                    fn send_blocking(&self, word: $WORD) {
                        // TODO: Upper limit for wait cycles to avoid complete hangups?
                        while self.spi.status().read().tnf().bit_is_clear() {}
                        self.send(word)
                    }

                    #[inline(always)]
                    fn send(&self, word: $WORD) {
                        self.spi.data().write(|w| unsafe { w.bits(word as u32) });
                    }

                    /// Read a word from the slave. Must be preceeded by a [`send`](Self::send) call
                    #[inline(always)]
                    fn read_blocking(&self) -> $WORD {
                        // TODO: Upper limit for wait cycles to avoid complete hangups?
                        while self.spi.status().read().rne().bit_is_clear() {}
                        self.read_single_word()
                    }

                    #[inline(always)]
                    fn read_single_word(&self) -> $WORD {
                        (self.spi.data().read().bits() & 0xffff) as $WORD
                    }

                    fn transfer_preparation(&self, words: &[$WORD]) -> Result<(), Infallible> {
                        if words.len() == 0 {
                            return Ok(());
                        }
                        let mut status_reg = self.spi.status().read();
                        loop {
                            // Wait until all bytes have been transferred.
                            while status_reg.tfe().bit_is_clear() {
                                // Ignore all received read words.
                                if status_reg.rne().bit_is_set() {
                                    self.clear_rx_fifo();
                                }
                                status_reg = self.spi.status().read();
                                continue;
                            }
                            break;
                        }
                        // Ignore all received read words.
                        if status_reg.rne().bit_is_set() {
                            self.clear_rx_fifo();
                        }
                        Ok(())
                    }

                    fn initial_send_fifo_pumping(&self, words: Option<&[$WORD]>) -> usize {
                        if self.blockmode {
                            self.spi.ctrl1().modify(|_, w| w.mtxpause().set_bit())
                        }
                        // Fill the first half of the write FIFO
                        let mut current_write_idx = 0;
                        for _ in 0..core::cmp::min(FILL_DEPTH, words.map_or(0, |words| words.len())) {
                            self.send_blocking(words.map_or(self.fill_word, |words| words[current_write_idx]));
                            current_write_idx += 1;
                        }
                        if self.blockmode {
                            self.spi.ctrl1().modify(|_, w| w.mtxpause().clear_bit())
                        }
                        current_write_idx
                    }
                }

                impl embedded_hal::spi::ErrorType for SpiBase<$SPIX, $WORD> {
                    type Error = Infallible;
                }

                impl SpiBus<$WORD> for SpiBase<$SPIX, $WORD> {
                    fn read(&mut self, words: &mut [$WORD]) -> Result<(), Self::Error> {
                        self.transfer_preparation(words)?;
                        let mut current_read_idx = 0;
                        let mut current_write_idx = self.initial_send_fifo_pumping(None);
                        loop {
                            if current_write_idx < words.len() {
                                self.send_blocking(self.fill_word);
                                current_write_idx += 1;
                            }
                            if current_read_idx < words.len() {
                                words[current_read_idx] = self.read_blocking();
                                current_read_idx += 1;
                            }
                            if current_read_idx >= words.len() && current_write_idx >= words.len() {
                                break;
                            }
                        }
                        Ok(())
                    }

                    fn write(&mut self, words: &[$WORD]) -> Result<(), Self::Error> {
                        self.transfer_preparation(words)?;
                        let mut current_write_idx = self.initial_send_fifo_pumping(Some(words));
                        while current_write_idx < words.len() {
                            self.send_blocking(words[current_write_idx]);
                            current_write_idx += 1;
                            // Ignore received words.
                            if self.spi.status().read().rne().bit_is_set() {
                                self.clear_rx_fifo();
                            }
                        }
                        Ok(())
                    }

                    fn transfer(&mut self, read: &mut [$WORD], write: &[$WORD]) -> Result<(), Self::Error> {
                        self.transfer_preparation(write)?;
                        let mut current_read_idx = 0;
                        let mut current_write_idx = self.initial_send_fifo_pumping(Some(write));
                        while current_read_idx < read.len() || current_write_idx < write.len() {
                            if current_write_idx < write.len() {
                                self.send_blocking(write[current_write_idx]);
                                current_write_idx += 1;
                            }
                            if current_read_idx < read.len() {
                                read[current_read_idx] = self.read_blocking();
                                current_read_idx += 1;
                            }
                        }

                        Ok(())
                    }

                    fn transfer_in_place(&mut self, words: &mut [$WORD]) -> Result<(), Self::Error> {
                        self.transfer_preparation(words)?;
                        let mut current_read_idx = 0;
                        let mut current_write_idx = self.initial_send_fifo_pumping(Some(words));

                        while current_read_idx < words.len() || current_write_idx < words.len() {
                            if current_write_idx < words.len() {
                                self.send_blocking(words[current_write_idx]);
                                current_write_idx += 1;
                            }
                            if current_read_idx < words.len() && current_read_idx < current_write_idx {
                                words[current_read_idx] = self.read_blocking();
                                current_read_idx += 1;
                            }
                        }
                        Ok(())
                    }

                    fn flush(&mut self) -> Result<(), Self::Error> {
                        let status_reg = self.spi.status().read();
                        while status_reg.tfe().bit_is_clear() || status_reg.rne().bit_is_set() {
                            if status_reg.rne().bit_is_set() {
                                self.read_single_word();
                            }
                        }
                        Ok(())
                    }
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>>
                    embedded_hal::spi::ErrorType for Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                {
                    type Error = Infallible;
                }

                impl<Sck: PinSck<$SPIX>, Miso: PinMiso<$SPIX>, Mosi: PinMosi<$SPIX>> SpiBus<$WORD>
                    for Spi<$SPIX, (Sck, Miso, Mosi), $WORD>
                {
                    fn read(&mut self, words: &mut [$WORD]) -> Result<(), Self::Error> {
                        self.inner.read(words)
                    }

                    fn write(&mut self, words: &[$WORD]) -> Result<(), Self::Error> {
                        self.inner.write(words)
                    }

                    fn transfer(&mut self, read: &mut [$WORD], write: &[$WORD]) -> Result<(), Self::Error> {
                        self.inner.transfer(read, write)
                    }

                    fn transfer_in_place(&mut self, words: &mut [$WORD]) -> Result<(), Self::Error> {
                        self.inner.transfer_in_place(words)
                    }

                    fn flush(&mut self) -> Result<(), Self::Error> {
                        self.inner.flush()
                    }
                }
            )+
        )+
    }
}

spi!(
    pac::Spia: (spia, PeripheralClocks::Spi0) => (u8, u16),
    pac::Spib: (spib, PeripheralClocks::Spi1) => (u8, u16),
    pac::Spic: (spic, PeripheralClocks::Spi2) => (u8, u16),
);
