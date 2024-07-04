//! API for the SPI peripheral
//!
//! ## Examples
//!
//! - [Blocking SPI example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/spi.rs)
use crate::{
    clock::enable_peripheral_clock,
    gpio::pin::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA10, PA11, PA12, PA13, PA14, PA15, PA16, PA17, PA18,
        PA19, PA20, PA21, PA22, PA23, PA24, PA25, PA26, PA27, PA28, PA29, PA30, PA31, PB0, PB1,
        PB10, PB11, PB12, PB13, PB14, PB15, PB16, PB17, PB18, PB19, PB2, PB22, PB23, PB3, PB4, PB5,
        PB6, PB7, PB8, PB9,
    },
    pac,
    time::Hertz,
    typelevel::Sealed,
    PeripheralSelect,
};
use core::{convert::Infallible, fmt::Debug, marker::PhantomData, ops::Deref};
use embedded_hal::spi::{Mode, MODE_0, MODE_1, MODE_2, MODE_3};

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

pub type SpiRegBlock = pac::spia::RegisterBlock;

/// Common trait implemented by all PAC peripheral access structures. The register block
/// format is the same for all SPI blocks.
pub trait Instance: Deref<Target = SpiRegBlock> {
    const IDX: u8;
    const PERIPH_SEL: PeripheralSelect;

    fn ptr() -> *const SpiRegBlock;
}

impl Instance for pac::Spia {
    const IDX: u8 = 0;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi0;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
}

impl Instance for pac::Spib {
    const IDX: u8 = 1;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi1;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
}

impl Instance for pac::Spic {
    const IDX: u8 = 2;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi2;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
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

macro_rules! hw_cs_pins {
    ($SPIx:path, $portId: path:
        $(
            ($PXx:ident, $AFx:ident, $HwCsIdent:path, $typedef:ident),
        )+
    ) => {
        $(
            impl HwCsProvider for Pin<$PXx, $AFx> {
                const CS_ID: HwChipSelectId = $HwCsIdent;
                const SPI_PORT: SpiPort = $portId;
            }
            impl OptionalHwCs<$SPIx> for Pin<$PXx, $AFx> {}
            pub type $typedef = Pin<$PXx, $AFx>;
        )+
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

hw_cs_pins!(
    pac::Spia, SpiPort::Porta:
    (PA28, AltFunc1, HwChipSelectId::Id0, HwCs0SpiAPortA),
    (PA27, AltFunc1, HwChipSelectId::Id1, HwCs1SpiAPortA),
    (PA26, AltFunc1, HwChipSelectId::Id2, HwCs2SpiAPortA),
    (PA25, AltFunc1, HwChipSelectId::Id3, HwCs3SpiAPortA),
    (PA24, AltFunc1, HwChipSelectId::Id4, HwCs4SpiAPortA),
    (PA23, AltFunc1, HwChipSelectId::Id5, HwCs5SpiAPortA),
    (PA22, AltFunc1, HwChipSelectId::Id6, HwCs6SpiAPortA),
    (PA21, AltFunc1, HwChipSelectId::Id7, HwCs7SpiAPortA),
    (PB6, AltFunc2, HwChipSelectId::Id0, HwCs0SpiAPortB),
    (PB5, AltFunc2, HwChipSelectId::Id6, HwCs6SpiAPortB),
    (PB4, AltFunc2, HwChipSelectId::Id5, HwCs5SpiAPortB),
    (PB3, AltFunc2, HwChipSelectId::Id4, HwCs4SpiAPortB),
    (PB2, AltFunc2, HwChipSelectId::Id3, HwCs3SpiAPortB),
    (PB1, AltFunc2, HwChipSelectId::Id2, HwCs2SpiAPortB),
    (PB0, AltFunc2, HwChipSelectId::Id1, HwCs1SpiAPortB),
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

hw_cs_pins!(
    pac::Spib, SpiPort::Portb:
    (PB16, AltFunc1, HwChipSelectId::Id0, HwCs0SpiBPortB0),
    (PB15, AltFunc1, HwChipSelectId::Id1, HwCs1SpiBPortB0),
    (PB14, AltFunc1, HwChipSelectId::Id2, HwCs2SpiBPortB0),
    (PB13, AltFunc1, HwChipSelectId::Id3, HwCs3SpiBPortB),
    (PB12, AltFunc1, HwChipSelectId::Id4, HwCs4SpiBPortB),
    (PB11, AltFunc1, HwChipSelectId::Id5, HwCs5SpiBPortB),
    (PB12, AltFunc2, HwChipSelectId::Id0, HwCs0SpiBPortB2),
    (PB11, AltFunc2, HwChipSelectId::Id1, HwCs1SpiBPortB2),
    (PB10, AltFunc1, HwChipSelectId::Id6, HwCs6SpiBPortB),
    (PB10, AltFunc2, HwChipSelectId::Id2, HwCs2SpiBPortB2),
    (PB2, AltFunc1, HwChipSelectId::Id0, HwCs0SpiBPortB1),
    (PB1, AltFunc1, HwChipSelectId::Id1, HwCs1SpiBPortB1),
    (PB0, AltFunc1, HwChipSelectId::Id2, HwCs2SpiBPortB1),
    (PA17, AltFunc2, HwChipSelectId::Id0, HwCs0SpiBPortA),
    (PA16, AltFunc2, HwChipSelectId::Id1, HwCs1SpiBPortA),
    (PA15, AltFunc2, HwChipSelectId::Id2, HwCs2SpiBPortA),
    (PA14, AltFunc2, HwChipSelectId::Id3, HwCs3SpiBPortA),
    (PA13, AltFunc2, HwChipSelectId::Id4, HwCs4SpiBPortA),
    (PA12, AltFunc2, HwChipSelectId::Id5, HwCs5SpiBPortA0),
    (PA11, AltFunc2, HwChipSelectId::Id6, HwCs6SpiBPortA0),
    (PA10, AltFunc2, HwChipSelectId::Id7, HwCs7SpiBPortA0),
    (PA23, AltFunc2, HwChipSelectId::Id5, HwCs5SpiBPortA1),
    (PA22, AltFunc2, HwChipSelectId::Id6, HwCs6SpiBPortA1),
    (PA21, AltFunc2, HwChipSelectId::Id7, HwCs7SpiBPortA1),
);

// SPIC

hw_cs_pins!(
    pac::Spic, SpiPort::Portc:
    (PB9, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortB0),
    (PB8, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortB0),
    (PB7, AltFunc3, HwChipSelectId::Id3, HwCs3SpiCPortB),
    (PB23, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortB1),
    (PB22, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortB1),
    (PA20, AltFunc1, HwChipSelectId::Id1, HwCs1SpiCPortA0),
    (PA19, AltFunc1, HwChipSelectId::Id2, HwCs2SpiCPortA0),
    (PB18, AltFunc1, HwChipSelectId::Id3, HwCs3SpiCPortA0),
    (PA23, AltFunc3, HwChipSelectId::Id1, HwCs1SpiCPortA1),
    (PA22, AltFunc3, HwChipSelectId::Id2, HwCs2SpiCPortA1),
    (PA21, AltFunc3, HwChipSelectId::Id3, HwCs3SpiCPortA1),
    (PA20, AltFunc3, HwChipSelectId::Id4, HwCs4SpiCPortA),
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
pub trait WordProvider: Copy + Default + Into<u32> + TryFrom<u32> + 'static {
    const MASK: u32;
    fn word_reg() -> u8;
}

impl WordProvider for u8 {
    const MASK: u32 = 0xff;
    fn word_reg() -> u8 {
        0x07
    }
}

impl WordProvider for u16 {
    const MASK: u32 = 0xffff;
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
    word: PhantomData<Word>,
}

pub struct Spi<SpiInstance, Pins, Word = u8> {
    inner: SpiBase<SpiInstance, Word>,
    pins: Pins,
}

// Re-export this so it can be used for the constructor
pub use crate::typelevel::NoneT;

impl<
        SpiI: Instance,
        Sck: PinSck<SpiI>,
        Miso: PinMiso<SpiI>,
        Mosi: PinMosi<SpiI>,
        Word: WordProvider,
    > Spi<SpiI, (Sck, Miso, Mosi), Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
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
    pub fn new(
        syscfg: &mut pac::Sysconfig,
        sys_clk: impl Into<Hertz> + Copy,
        spi: SpiI,
        pins: (Sck, Miso, Mosi),
        spi_cfg: SpiConfig,
        transfer_cfg: Option<&ReducedTransferConfig>,
    ) -> Self {
        enable_peripheral_clock(syscfg, SpiI::PERIPH_SEL);
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
            if transfer_cfg.hw_cs != HwChipSelectId::Invalid {
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
                w.size().bits(Word::word_reg());
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
                word: PhantomData,
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

    pub fn set_fill_word(&mut self, fill_word: Word) {
        self.inner.fill_word = fill_word;
    }

    pub fn fill_word(&self) -> Word {
        self.inner.fill_word
    }

    #[inline]
    pub fn perid(&self) -> u32 {
        self.inner.perid()
    }

    pub fn cfg_transfer<HwCs: OptionalHwCs<SpiI>>(&mut self, transfer_cfg: &TransferConfig<HwCs>) {
        self.inner.cfg_transfer(transfer_cfg);
    }

    /// Releases the SPI peripheral and associated pins
    pub fn release(self) -> (SpiI, (Sck, Miso, Mosi), SpiConfig) {
        (self.inner.spi, self.pins, self.inner.cfg)
    }

    pub fn downgrade(self) -> SpiBase<SpiI, Word> {
        self.inner
    }
}

impl<SpiInstance: Instance, Word: WordProvider> SpiBase<SpiInstance, Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    #[inline]
    pub fn cfg_clock(&mut self, spi_clk: impl Into<Hertz>) {
        let clk_prescale =
            self.sys_clk.raw() / (spi_clk.into().raw() * (self.cfg.scrdv as u32 + 1));
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
    pub fn cfg_hw_cs_with_pin<HwCs: OptionalHwCs<SpiInstance>>(&mut self, _: &HwCs) {
        self.cfg_hw_cs(HwCs::CS_ID);
    }

    pub fn cfg_hw_cs_disable(&mut self) {
        self.spi.ctrl1().modify(|_, w| {
            w.sod().set_bit();
            w
        });
    }

    pub fn cfg_transfer<HwCs: OptionalHwCs<SpiInstance>>(
        &mut self,
        transfer_cfg: &TransferConfig<HwCs>,
    ) {
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

    /// Sends a word to the slave
    #[inline(always)]
    fn send_blocking(&self, word: Word) {
        // TODO: Upper limit for wait cycles to avoid complete hangups?
        while self.spi.status().read().tnf().bit_is_clear() {}
        self.send(word)
    }

    #[inline(always)]
    fn send(&self, word: Word) {
        self.spi.data().write(|w| unsafe { w.bits(word.into()) });
    }

    /// Read a word from the slave. Must be preceeded by a [`send`](Self::send) call
    #[inline(always)]
    fn read_blocking(&self) -> Word {
        // TODO: Upper limit for wait cycles to avoid complete hangups?
        while self.spi.status().read().rne().bit_is_clear() {}
        self.read_single_word()
    }

    #[inline(always)]
    fn read_single_word(&self) -> Word {
        (self.spi.data().read().bits() & Word::MASK)
            .try_into()
            .unwrap()
    }

    fn transfer_preparation(&self, words: &[Word]) -> Result<(), Infallible> {
        if words.is_empty() {
            return Ok(());
        }
        let mut status_reg = self.spi.status().read();
        // Wait until all bytes have been transferred.
        while status_reg.tfe().bit_is_clear() {
            // Ignore all received read words.
            if status_reg.rne().bit_is_set() {
                self.clear_rx_fifo();
            }
            status_reg = self.spi.status().read();
        }
        // Ignore all received read words.
        if status_reg.rne().bit_is_set() {
            self.clear_rx_fifo();
        }
        Ok(())
    }

    fn initial_send_fifo_pumping(&self, words: Option<&[Word]>) -> usize {
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

/// Changing the word size also requires a type conversion
impl<SpiI: Instance, Sck: PinSck<SpiI>, Miso: PinMiso<SpiI>, Mosi: PinMosi<SpiI>>
    From<Spi<SpiI, (Sck, Miso, Mosi), u8>> for Spi<SpiI, (Sck, Miso, Mosi), u16>
{
    fn from(old_spi: Spi<SpiI, (Sck, Miso, Mosi), u8>) -> Self {
        old_spi
            .inner
            .spi
            .ctrl0()
            .modify(|_, w| unsafe { w.size().bits(WordSize::SixteenBits as u8) });
        Spi {
            inner: SpiBase {
                spi: old_spi.inner.spi,
                cfg: old_spi.inner.cfg,
                blockmode: old_spi.inner.blockmode,
                fill_word: Default::default(),
                sys_clk: old_spi.inner.sys_clk,
                word: PhantomData,
            },
            pins: old_spi.pins,
        }
    }
}

/// Changing the word size also requires a type conversion
impl<SpiI: Instance, Sck: PinSck<SpiI>, Miso: PinMiso<SpiI>, Mosi: PinMosi<SpiI>>
    From<Spi<SpiI, (Sck, Miso, Mosi), u16>> for Spi<SpiI, (Sck, Miso, Mosi), u8>
{
    fn from(old_spi: Spi<SpiI, (Sck, Miso, Mosi), u16>) -> Self {
        old_spi
            .inner
            .spi
            .ctrl0()
            .modify(|_, w| unsafe { w.size().bits(WordSize::EightBits as u8) });
        Spi {
            inner: SpiBase {
                spi: old_spi.inner.spi,
                cfg: old_spi.inner.cfg,
                blockmode: old_spi.inner.blockmode,
                sys_clk: old_spi.inner.sys_clk,
                fill_word: Default::default(),
                word: PhantomData,
            },
            pins: old_spi.pins,
        }
    }
}

impl<SpiI: Instance, Word: WordProvider> embedded_hal::spi::ErrorType for SpiBase<SpiI, Word> {
    type Error = Infallible;
}

impl<SpiI: Instance, Word: WordProvider> embedded_hal::spi::SpiBus<Word> for SpiBase<SpiI, Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
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

    fn write(&mut self, words: &[Word]) -> Result<(), Self::Error> {
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

    fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error> {
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

    fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
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

impl<
        SpiI: Instance,
        Word: WordProvider,
        Sck: PinSck<SpiI>,
        Miso: PinMiso<SpiI>,
        Mosi: PinMosi<SpiI>,
    > embedded_hal::spi::ErrorType for Spi<SpiI, (Sck, Miso, Mosi), Word>
{
    type Error = Infallible;
}

impl<
        SpiI: Instance,
        Word: WordProvider,
        Sck: PinSck<SpiI>,
        Miso: PinMiso<SpiI>,
        Mosi: PinMosi<SpiI>,
    > embedded_hal::spi::SpiBus<Word> for Spi<SpiI, (Sck, Miso, Mosi), Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        self.inner.read(words)
    }

    fn write(&mut self, words: &[Word]) -> Result<(), Self::Error> {
        self.inner.write(words)
    }

    fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error> {
        self.inner.transfer(read, write)
    }

    fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        self.inner.transfer_in_place(words)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.inner.flush()
    }
}
