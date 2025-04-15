//! API for the SPI peripheral.
//!
//! The main abstraction provided by this module are the [Spi] and the [SpiBase] structure.
//! These provide the [embedded_hal::spi] traits, but also offer a low level interface
//! via the [SpiLowLevel] trait.
//!
//! ## Examples
//!
//! - [Blocking SPI example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/spi.rs)
//! - [REB1 ADC example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/max11519-adc.rs)
//! - [REB1 EEPROM library](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/src/m95m01.rs)
use crate::{clock::enable_peripheral_clock, pac, time::Hertz, PeripheralSelect};
use core::{convert::Infallible, fmt::Debug, marker::PhantomData, ops::Deref};
use embedded_hal::spi::{Mode, MODE_0};
use pins::{HwCsProvider, PinMiso, PinMosi, PinSck};

pub mod pins;

//==================================================================================================
// Defintions
//==================================================================================================

// FIFO has a depth of 16.
const FILL_DEPTH: usize = 12;

pub const BMSTART_BMSTOP_MASK: u32 = 1 << 31;
pub const BMSKIPDATA_MASK: u32 = 1 << 30;

pub const DEFAULT_CLK_DIV: u16 = 2;

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HwChipSelectId {
    Id0 = 0,
    Id1 = 1,
    Id2 = 2,
    Id3 = 3,
    Id4 = 4,
    Id5 = 5,
    Id6 = 6,
    Id7 = 7,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiId {
    A,
    B,
    C,
}

impl SpiId {
    /// Unsafely steal a peripheral MMIO block for the given UART.
    ///
    /// # Safety
    ///
    /// Circumvents ownership and safety guarantees by the HAL which can lead to data races
    /// on cuncurrent usage.
    pub unsafe fn reg_block(&self) -> &'static SpiRegBlock {
        unsafe {
            match self {
                SpiId::A => va108xx::Spia::steal().reg_block(),
                SpiId::B => va108xx::Spib::steal().reg_block(),
                SpiId::C => va108xx::Spic::steal().reg_block(),
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WordSize {
    OneBit = 0x00,
    FourBits = 0x03,
    EightBits = 0x07,
    SixteenBits = 0x0f,
}

pub type SpiRegBlock = pac::spia::RegisterBlock;

/// Common trait implemented by all PAC peripheral access structures. The register block
/// format is the same for all SPI blocks.
pub trait SpiPeripheralMarker: Deref<Target = SpiRegBlock> {
    const ID: SpiId;
    const PERIPH_SEL: PeripheralSelect;

    fn ptr() -> *const SpiRegBlock;

    #[inline(always)]
    fn reg_block(&self) -> &'static mut SpiRegBlock {
        unsafe { &mut *(Self::ptr() as *mut _) }
    }
}

impl SpiPeripheralMarker for pac::Spia {
    const ID: SpiId = SpiId::A;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi0;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
}

impl SpiPeripheralMarker for pac::Spib {
    const ID: SpiId = SpiId::B;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi1;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
}

impl SpiPeripheralMarker for pac::Spic {
    const ID: SpiId = SpiId::C;
    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Spi2;

    #[inline(always)]
    fn ptr() -> *const SpiRegBlock {
        Self::ptr()
    }
}

//==================================================================================================
// Config
//==================================================================================================

pub trait TransferConfigProvider {
    fn sod(&mut self, sod: bool);
    fn blockmode(&mut self, blockmode: bool);
    fn mode(&mut self, mode: Mode);
    fn clk_cfg(&mut self, clk_cfg: SpiClkConfig);
    fn hw_cs_id(&self) -> u8;
}

/*
/// This struct contains all configuration parameter which are transfer specific
/// and might change for transfers to different SPI slaves
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferConfigWithHwcs<HwCs> {
    pub hw_cs: Option<HwCs>,
    pub cfg: TransferConfig,
}
*/

/// Type erased variant of the transfer configuration. This is required to avoid generics in
/// the SPI constructor.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferConfig {
    pub clk_cfg: Option<SpiClkConfig>,
    pub mode: Option<Mode>,
    pub sod: bool,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words
    pub blockmode: bool,
    /// Only used when blockmode is used. The SCK will be stalled until an explicit stop bit
    /// is set on a written word.
    pub bmstall: bool,
    pub hw_cs: Option<HwChipSelectId>,
}

/*
impl TransferConfigWithHwcs<NoneT> {
    pub fn new_no_hw_cs(
        clk_cfg: Option<SpiClkConfig>,
        mode: Option<Mode>,
        blockmode: bool,
        bmstall: bool,
        sod: bool,
    ) -> Self {
        TransferConfigWithHwcs {
            hw_cs: None,
            cfg: TransferConfig {
                clk_cfg,
                mode,
                sod,
                blockmode,
                bmstall,
                hw_cs: HwChipSelectId::Invalid,
            },
        }
    }
}

impl<HwCs: HwCsProvider> TransferConfigWithHwcs<HwCs> {
    pub fn new(
        clk_cfg: Option<SpiClkConfig>,
        mode: Option<Mode>,
        hw_cs: Option<HwCs>,
        blockmode: bool,
        bmstall: bool,
        sod: bool,
    ) -> Self {
        TransferConfigWithHwcs {
            hw_cs,
            cfg: TransferConfig {
                clk_cfg,
                mode,
                sod,
                blockmode,
                bmstall,
                hw_cs: HwCs::CS_ID,
            },
        }
    }

    pub fn downgrade(self) -> TransferConfig {
        self.cfg
    }
}

impl<HwCs: HwCsProvider> TransferConfigProvider for TransferConfigWithHwcs<HwCs> {
    /// Slave Output Disable
    fn sod(&mut self, sod: bool) {
        self.cfg.sod = sod;
    }

    fn blockmode(&mut self, blockmode: bool) {
        self.cfg.blockmode = blockmode;
    }

    fn mode(&mut self, mode: Mode) {
        self.cfg.mode = Some(mode);
    }

    fn clk_cfg(&mut self, clk_cfg: SpiClkConfig) {
        self.cfg.clk_cfg = Some(clk_cfg);
    }

    fn hw_cs_id(&self) -> u8 {
        HwCs::CS_ID as u8
    }
}
*/

/// Configuration options for the whole SPI bus. See Programmer Guide p.92 for more details
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiConfig {
    clk: SpiClkConfig,
    // SPI mode configuration
    pub init_mode: Mode,
    /// If this is enabled, all data in the FIFO is transmitted in a single frame unless
    /// the BMSTOP bit is set on a dataword. A frame is defined as CSn being active for the
    /// duration of multiple data words. Defaults to true.
    pub blockmode: bool,
    /// This enables the stalling of the SPI SCK if in blockmode and the FIFO is empty.
    /// Currently enabled by default.
    pub bmstall: bool,
    /// By default, configure SPI for master mode (ms == false)
    ms: bool,
    /// Slave output disable. Useful if separate GPIO pins or decoders are used for CS control
    pub slave_output_disable: bool,
    /// Loopback mode. If you use this, don't connect MISO to MOSI, they will be tied internally
    pub loopback_mode: bool,
    /// Enable Master Delayer Capture Mode. See Programmers Guide p.92 for more details
    pub master_delayer_capture: bool,
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            init_mode: MODE_0,
            blockmode: true,
            bmstall: true,
            // Default value is definitely valid.
            clk: SpiClkConfig::from_div(DEFAULT_CLK_DIV).unwrap(),
            ms: Default::default(),
            slave_output_disable: Default::default(),
            loopback_mode: Default::default(),
            master_delayer_capture: Default::default(),
        }
    }
}

impl SpiConfig {
    pub fn loopback(mut self, enable: bool) -> Self {
        self.loopback_mode = enable;
        self
    }

    pub fn blockmode(mut self, enable: bool) -> Self {
        self.blockmode = enable;
        self
    }

    pub fn bmstall(mut self, enable: bool) -> Self {
        self.bmstall = enable;
        self
    }

    pub fn mode(mut self, mode: Mode) -> Self {
        self.init_mode = mode;
        self
    }

    pub fn clk_cfg(mut self, clk_cfg: SpiClkConfig) -> Self {
        self.clk = clk_cfg;
        self
    }

    pub fn master_mode(mut self, master: bool) -> Self {
        self.ms = !master;
        self
    }

    pub fn slave_output_disable(mut self, sod: bool) -> Self {
        self.slave_output_disable = sod;
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

/// Low level access trait for the SPI peripheral.
pub trait SpiLowLevel {
    /// Low level function to write a word to the SPI FIFO but also checks whether
    /// there is actually data in the FIFO.
    ///
    /// Uses the [nb] API to allow usage in blocking and non-blocking contexts.
    fn write_fifo(&mut self, data: u32) -> nb::Result<(), Infallible>;

    /// Low level function to write a word to the SPI FIFO without checking whether
    /// there FIFO is full.
    ///
    /// This does not necesarily mean there is a space in the FIFO available.
    /// Use [Self::write_fifo] function to write a word into the FIFO reliably.
    fn write_fifo_unchecked(&mut self, data: u32);

    /// Low level function to read a word from the SPI FIFO. Must be preceeded by a
    /// [Self::write_fifo] call.
    ///
    /// Uses the [nb] API to allow usage in blocking and non-blocking contexts.
    fn read_fifo(&mut self) -> nb::Result<u32, Infallible>;

    /// Low level function to read a word from from the SPI FIFO.
    ///
    /// This does not necesarily mean there is a word in the FIFO available.
    /// Use the [Self::read_fifo] function to read a word from the FIFO reliably using the [nb]
    /// API.
    /// You might also need to mask the value to ignore the BMSTART/BMSTOP bit.
    fn read_fifo_unchecked(&mut self) -> u32;
}

pub struct Spi<Word = u8> {
    id: SpiId,
    reg_block: *mut SpiRegBlock,
    cfg: SpiConfig,
    sys_clk: Hertz,
    /// Fill word for read-only SPI transactions.
    pub fill_word: Word,
    blockmode: bool,
    bmstall: bool,
    word: PhantomData<Word>,
}

/*
pub struct Spi<, Pins, Word = u8> {
    inner: SpiBase<SpiInstance, Word>,
    pins: Pins,
}
*/

#[inline(always)]
pub fn mode_to_cpo_cph_bit(mode: embedded_hal::spi::Mode) -> (bool, bool) {
    match mode {
        embedded_hal::spi::MODE_0 => (false, false),
        embedded_hal::spi::MODE_1 => (false, true),
        embedded_hal::spi::MODE_2 => (true, false),
        embedded_hal::spi::MODE_3 => (true, true),
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SpiClkConfig {
    prescale_val: u16,
    scrdv: u8,
}

impl SpiClkConfig {
    pub fn prescale_val(&self) -> u16 {
        self.prescale_val
    }
    pub fn scrdv(&self) -> u8 {
        self.scrdv
    }
}

impl SpiClkConfig {
    pub fn new(prescale_val: u16, scrdv: u8) -> Self {
        Self {
            prescale_val,
            scrdv,
        }
    }

    pub fn from_div(div: u16) -> Result<Self, SpiClkConfigError> {
        spi_clk_config_from_div(div)
    }

    pub fn from_clk(sys_clk: impl Into<Hertz>, spi_clk: impl Into<Hertz>) -> Option<Self> {
        clk_div_for_target_clock(sys_clk, spi_clk).map(|div| spi_clk_config_from_div(div).unwrap())
    }
}

#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SpiClkConfigError {
    #[error("division by zero")]
    DivIsZero,
    #[error("divide value is not even")]
    DivideValueNotEven,
    #[error("scrdv value is too large")]
    ScrdvValueTooLarge,
}

#[inline]
pub fn spi_clk_config_from_div(mut div: u16) -> Result<SpiClkConfig, SpiClkConfigError> {
    if div == 0 {
        return Err(SpiClkConfigError::DivIsZero);
    }
    if div % 2 != 0 {
        return Err(SpiClkConfigError::DivideValueNotEven);
    }
    let mut prescale_val = 0;

    // find largest (even) prescale value that divides into div
    for i in (2..=0xfe).rev().step_by(2) {
        if div % i == 0 {
            prescale_val = i;
            break;
        }
    }

    if prescale_val == 0 {
        return Err(SpiClkConfigError::DivideValueNotEven);
    }

    div /= prescale_val;
    if div > u8::MAX as u16 + 1 {
        return Err(SpiClkConfigError::ScrdvValueTooLarge);
    }
    Ok(SpiClkConfig {
        prescale_val,
        scrdv: (div - 1) as u8,
    })
}

#[inline]
pub fn clk_div_for_target_clock(
    sys_clk: impl Into<Hertz>,
    spi_clk: impl Into<Hertz>,
) -> Option<u16> {
    let spi_clk = spi_clk.into();
    let sys_clk = sys_clk.into();
    if spi_clk > sys_clk {
        return None;
    }

    // Step 1: Calculate raw divider.
    let raw_div = sys_clk.raw() / spi_clk.raw();
    let remainder = sys_clk.raw() % spi_clk.raw();

    // Step 2: Round up if necessary.
    let mut rounded_div = if remainder * 2 >= spi_clk.raw() {
        raw_div + 1
    } else {
        raw_div
    };

    if rounded_div % 2 != 0 {
        // Take slower clock conservatively.
        rounded_div += 1;
    }
    if rounded_div > u16::MAX as u32 {
        return None;
    }
    Some(rounded_div as u16)
}

impl<Word: WordProvider> Spi<Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    /// Create a new SPI struct
    ///
    /// You can delete the pin type information by calling the
    /// [`downgrade`](Self::downgrade) function
    ///
    /// ## Arguments
    /// * `syscfg` - Can be passed optionally to enable the peripheral clock
    /// * `sys_clk` - System clock
    /// * `spi` - SPI bus to use
    /// * `pins` - Pins to be used for SPI transactions. These pins are consumed
    ///   to ensure the pins can not be used for other purposes anymore
    /// * `spi_cfg` - Configuration specific to the SPI bus
    pub fn new<SpiI: SpiPeripheralMarker, Sck: PinSck, Miso: PinMiso, Mosi: PinMosi>(
        sys_clk: impl Into<Hertz>,
        spi: SpiI,
        _pins: (Sck, Miso, Mosi),
        spi_cfg: SpiConfig,
    ) -> Self {
        enable_peripheral_clock(SpiI::PERIPH_SEL);
        let (cpo_bit, cph_bit) = mode_to_cpo_cph_bit(spi_cfg.init_mode);
        spi.ctrl0().write(|w| {
            unsafe {
                w.size().bits(Word::word_reg());
                w.scrdv().bits(spi_cfg.clk.scrdv);
                // Clear clock phase and polarity. Will be set to correct value for each
                // transfer
                w.spo().bit(cpo_bit);
                w.sph().bit(cph_bit)
            }
        });

        spi.ctrl1().write(|w| {
            w.lbm().bit(spi_cfg.loopback_mode);
            w.sod().bit(spi_cfg.slave_output_disable);
            w.ms().bit(spi_cfg.ms);
            w.mdlycap().bit(spi_cfg.master_delayer_capture);
            w.blockmode().bit(spi_cfg.blockmode);
            w.bmstall().bit(spi_cfg.bmstall);
            unsafe { w.ss().bits(0) }
        });
        spi.clkprescale()
            .write(|w| unsafe { w.bits(spi_cfg.clk.prescale_val as u32) });

        spi.fifo_clr().write(|w| {
            w.rxfifo().set_bit();
            w.txfifo().set_bit()
        });
        // Enable the peripheral as the last step as recommended in the
        // programmers guide
        spi.ctrl1().modify(|_, w| w.enable().set_bit());
        Spi {
            id: SpiI::ID,
            reg_block: spi.reg_block(),
            cfg: spi_cfg,
            sys_clk: sys_clk.into(),
            fill_word: Default::default(),
            bmstall: spi_cfg.bmstall,
            blockmode: spi_cfg.blockmode,
            word: PhantomData,
        }
    }

    #[inline(always)]
    pub fn reg_block_mut(&mut self) -> &'static mut SpiRegBlock {
        unsafe { &mut *(self.reg_block) }
    }

    #[inline(always)]
    pub fn reg_block(&self) -> &'static SpiRegBlock {
        unsafe { &*(self.reg_block) }
    }

    #[inline]
    pub fn cfg_clock(&mut self, cfg: SpiClkConfig) {
        self.reg_block()
            .ctrl0()
            .modify(|_, w| unsafe { w.scrdv().bits(cfg.scrdv) });
        self.reg_block()
            .clkprescale()
            .write(|w| unsafe { w.bits(cfg.prescale_val as u32) });
    }

    #[inline]
    pub fn cfg_clock_from_div(&mut self, div: u16) -> Result<(), SpiClkConfigError> {
        let val = spi_clk_config_from_div(div)?;
        self.cfg_clock(val);
        Ok(())
    }

    #[inline]
    pub fn cfg_mode(&mut self, mode: Mode) {
        let (cpo_bit, cph_bit) = mode_to_cpo_cph_bit(mode);
        self.reg_block().ctrl0().modify(|_, w| {
            w.spo().bit(cpo_bit);
            w.sph().bit(cph_bit)
        });
    }

    #[inline]
    pub fn fill_word(&self) -> Word {
        self.fill_word
    }

    #[inline]
    pub fn clear_tx_fifo(&mut self) {
        self.reg_block().fifo_clr().write(|w| w.txfifo().set_bit());
    }

    #[inline]
    pub fn clear_rx_fifo(&mut self) {
        self.reg_block().fifo_clr().write(|w| w.rxfifo().set_bit());
    }

    #[inline]
    pub fn perid(&self) -> u32 {
        self.reg_block().perid().read().bits()
    }

    /// Configure the hardware chip select given a hardware chip select ID.
    #[inline]
    pub fn cfg_hw_cs(&mut self, hw_cs: HwChipSelectId) {
        self.reg_block_mut().ctrl1().modify(|_, w| {
            w.sod().clear_bit();
            unsafe {
                w.ss().bits(hw_cs as u8);
            }
            w
        });
    }

    /// Configure the hardware chip select given a physical hardware CS pin.
    #[inline]
    pub fn cfg_hw_cs_with_pin<HwCs: HwCsProvider>(&mut self, _: &HwCs) {
        // TODO: Error handling.
        self.cfg_hw_cs(HwCs::CS_ID);
    }

    /// Disables the hardware chip select functionality. This can be used when performing
    /// external chip select handling, for example with GPIO pins.
    #[inline]
    pub fn cfg_hw_cs_disable(&mut self) {
        self.reg_block().ctrl1().modify(|_, w| {
            w.sod().set_bit();
            w
        });
    }

    /// Utility function to configure all relevant transfer parameters in one go.
    /// This is useful if multiple devices with different clock and mode configurations
    /// are connected to one bus.
    pub fn cfg_transfer(&mut self, transfer_cfg: &TransferConfig) {
        if let Some(trans_clk_div) = transfer_cfg.clk_cfg {
            self.cfg_clock(trans_clk_div);
        }
        if let Some(mode) = transfer_cfg.mode {
            self.cfg_mode(mode);
        }
        self.blockmode = transfer_cfg.blockmode;
        self.reg_block().ctrl1().modify(|_, w| {
            if transfer_cfg.sod {
                w.sod().set_bit();
            } else if transfer_cfg.hw_cs.is_some() {
                w.sod().clear_bit();
                unsafe {
                    w.ss().bits(transfer_cfg.hw_cs.unwrap() as u8);
                }
            } else {
                w.sod().clear_bit();
            }
            w.blockmode().bit(transfer_cfg.blockmode);
            w.bmstall().bit(transfer_cfg.bmstall)
        });
    }

    fn flush_internal(&mut self) {
        let reg_block_mut = self.reg_block_mut();
        let mut status_reg = reg_block_mut.status().read();
        while status_reg.tfe().bit_is_clear()
            || status_reg.rne().bit_is_set()
            || status_reg.busy().bit_is_set()
        {
            if status_reg.rne().bit_is_set() {
                self.read_fifo_unchecked();
            }
            status_reg = reg_block_mut.status().read();
        }
    }

    fn transfer_preparation(&mut self, words: &[Word]) -> Result<(), Infallible> {
        if words.is_empty() {
            return Ok(());
        }
        self.flush_internal();
        Ok(())
    }

    // The FIFO can hold a guaranteed amount of data, so we can pump it on transfer
    // initialization. Returns the amount of written bytes.
    fn initial_send_fifo_pumping_with_words(&mut self, words: &[Word]) -> usize {
        let reg_block_mut = self.reg_block_mut();
        if self.blockmode {
            reg_block_mut.ctrl1().modify(|_, w| w.mtxpause().set_bit());
        }
        // Fill the first half of the write FIFO
        let mut current_write_idx = 0;
        let smaller_idx = core::cmp::min(FILL_DEPTH, words.len());
        for _ in 0..smaller_idx {
            if current_write_idx == smaller_idx.saturating_sub(1) && self.bmstall {
                self.write_fifo_unchecked(words[current_write_idx].into() | BMSTART_BMSTOP_MASK);
            } else {
                self.write_fifo_unchecked(words[current_write_idx].into());
            }
            current_write_idx += 1;
        }
        if self.blockmode {
            reg_block_mut
                .ctrl1()
                .modify(|_, w| w.mtxpause().clear_bit());
        }
        current_write_idx
    }

    // The FIFO can hold a guaranteed amount of data, so we can pump it on transfer
    // initialization.
    fn initial_send_fifo_pumping_with_fill_words(&mut self, send_len: usize) -> usize {
        let reg_block_mut = self.reg_block_mut();
        if self.blockmode {
            reg_block_mut.ctrl1().modify(|_, w| w.mtxpause().set_bit());
        }
        // Fill the first half of the write FIFO
        let mut current_write_idx = 0;
        let smaller_idx = core::cmp::min(FILL_DEPTH, send_len);
        for _ in 0..smaller_idx {
            if current_write_idx == smaller_idx.saturating_sub(1) && self.bmstall {
                self.write_fifo_unchecked(self.fill_word.into() | BMSTART_BMSTOP_MASK);
            } else {
                self.write_fifo_unchecked(self.fill_word.into());
            }
            current_write_idx += 1;
        }
        if self.blockmode {
            reg_block_mut
                .ctrl1()
                .modify(|_, w| w.mtxpause().clear_bit());
        }
        current_write_idx
    }
}

impl<Word: WordProvider> SpiLowLevel for Spi<Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    #[inline(always)]
    fn write_fifo(&mut self, data: u32) -> nb::Result<(), Infallible> {
        if self.reg_block_mut().status().read().tnf().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        self.write_fifo_unchecked(data);
        Ok(())
    }

    #[inline(always)]
    fn write_fifo_unchecked(&mut self, data: u32) {
        self.reg_block_mut()
            .data()
            .write(|w| unsafe { w.bits(data) });
    }

    #[inline(always)]
    fn read_fifo(&mut self) -> nb::Result<u32, Infallible> {
        if self.reg_block_mut().status().read().rne().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(self.read_fifo_unchecked())
    }

    #[inline(always)]
    fn read_fifo_unchecked(&mut self) -> u32 {
        self.reg_block_mut().data().read().bits()
    }
}

impl<Word: WordProvider> embedded_hal::spi::ErrorType for Spi<Word> {
    type Error = Infallible;
}

impl<Word: WordProvider> embedded_hal::spi::SpiBus<Word> for Spi<Word>
where
    <Word as TryFrom<u32>>::Error: core::fmt::Debug,
{
    fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        self.transfer_preparation(words)?;
        let mut current_read_idx = 0;
        let mut current_write_idx = self.initial_send_fifo_pumping_with_fill_words(words.len());
        loop {
            if current_read_idx < words.len() {
                words[current_read_idx] = (nb::block!(self.read_fifo())? & Word::MASK)
                    .try_into()
                    .unwrap();
                current_read_idx += 1;
            }
            if current_write_idx < words.len() {
                if current_write_idx == words.len() - 1 && self.bmstall {
                    nb::block!(self.write_fifo(self.fill_word.into() | BMSTART_BMSTOP_MASK))?;
                } else {
                    nb::block!(self.write_fifo(self.fill_word.into()))?;
                }
                current_write_idx += 1;
            }
            if current_read_idx >= words.len() && current_write_idx >= words.len() {
                break;
            }
        }
        Ok(())
    }

    fn write(&mut self, words: &[Word]) -> Result<(), Self::Error> {
        self.transfer_preparation(words)?;
        let mut current_write_idx = self.initial_send_fifo_pumping_with_words(words);
        while current_write_idx < words.len() {
            if current_write_idx == words.len() - 1 && self.bmstall {
                nb::block!(self.write_fifo(words[current_write_idx].into() | BMSTART_BMSTOP_MASK))?;
            } else {
                nb::block!(self.write_fifo(words[current_write_idx].into()))?;
            }
            current_write_idx += 1;
            // Ignore received words.
            if self.reg_block().status().read().rne().bit_is_set() {
                self.clear_rx_fifo();
            }
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error> {
        self.transfer_preparation(write)?;
        let mut current_read_idx = 0;
        let mut current_write_idx = self.initial_send_fifo_pumping_with_words(write);
        while current_read_idx < read.len() || current_write_idx < write.len() {
            if current_write_idx < write.len() {
                if current_write_idx == write.len() - 1 && self.bmstall {
                    nb::block!(
                        self.write_fifo(write[current_write_idx].into() | BMSTART_BMSTOP_MASK)
                    )?;
                } else {
                    nb::block!(self.write_fifo(write[current_write_idx].into()))?;
                }
                current_write_idx += 1;
            }
            if current_read_idx < read.len() {
                read[current_read_idx] = (nb::block!(self.read_fifo())? & Word::MASK)
                    .try_into()
                    .unwrap();
                current_read_idx += 1;
            }
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        self.transfer_preparation(words)?;
        let mut current_read_idx = 0;
        let mut current_write_idx = self.initial_send_fifo_pumping_with_words(words);

        while current_read_idx < words.len() || current_write_idx < words.len() {
            if current_write_idx < words.len() {
                if current_write_idx == words.len() - 1 && self.bmstall {
                    nb::block!(
                        self.write_fifo(words[current_write_idx].into() | BMSTART_BMSTOP_MASK)
                    )?;
                } else {
                    nb::block!(self.write_fifo(words[current_write_idx].into()))?;
                }
                current_write_idx += 1;
            }
            if current_read_idx < words.len() && current_read_idx < current_write_idx {
                words[current_read_idx] = (nb::block!(self.read_fifo())? & Word::MASK)
                    .try_into()
                    .unwrap();
                current_read_idx += 1;
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_internal();
        Ok(())
    }
}

/// Changing the word size also requires a type conversion
impl From<Spi<u8>> for Spi<u16> {
    fn from(old_spi: Spi<u8>) -> Self {
        old_spi
            .reg_block()
            .ctrl0()
            .modify(|_, w| unsafe { w.size().bits(WordSize::SixteenBits as u8) });
        Spi {
            id: old_spi.id,
            reg_block: old_spi.reg_block,
            cfg: old_spi.cfg,
            blockmode: old_spi.blockmode,
            fill_word: Default::default(),
            bmstall: old_spi.bmstall,
            sys_clk: old_spi.sys_clk,
            word: PhantomData,
        }
    }
}

impl From<Spi<u16>> for Spi<u8> {
    fn from(old_spi: Spi<u16>) -> Self {
        old_spi
            .reg_block()
            .ctrl0()
            .modify(|_, w| unsafe { w.size().bits(WordSize::EightBits as u8) });
        Spi {
            id: old_spi.id,
            reg_block: old_spi.reg_block,
            cfg: old_spi.cfg,
            blockmode: old_spi.blockmode,
            fill_word: Default::default(),
            bmstall: old_spi.bmstall,
            sys_clk: old_spi.sys_clk,
            word: PhantomData,
        }
    }
}
