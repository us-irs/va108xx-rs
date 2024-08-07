//! # API for the UART peripheral
//!
//! ## Examples
//!
//! - [UART simple example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/uart.rs)
//! - [UART with IRQ and RTIC](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/uart-irq-rtic.rs)
use core::{marker::PhantomData, ops::Deref};
use embedded_hal_nb::serial::Read;
use fugit::RateExtU32;

pub use crate::IrqCfg;
use crate::{
    clock::{enable_peripheral_clock, PeripheralClocks},
    gpio::pin::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA16, PA17, PA18, PA19, PA2, PA26, PA27, PA3, PA30,
        PA31, PA8, PA9, PB18, PB19, PB20, PB21, PB22, PB23, PB6, PB7, PB8, PB9,
    },
    pac::{self, uarta as uart_base},
    time::Hertz,
    utility::unmask_irq,
    PeripheralSelect,
};

//==================================================================================================
// Type-Level support
//==================================================================================================

pub trait Pins<UART> {}

impl Pins<pac::Uarta> for (Pin<PA9, AltFunc2>, Pin<PA8, AltFunc2>) {}
impl Pins<pac::Uarta> for (Pin<PA17, AltFunc3>, Pin<PA16, AltFunc3>) {}
impl Pins<pac::Uarta> for (Pin<PA31, AltFunc3>, Pin<PA30, AltFunc3>) {}

impl Pins<pac::Uarta> for (Pin<PB9, AltFunc1>, Pin<PB8, AltFunc1>) {}
impl Pins<pac::Uarta> for (Pin<PB23, AltFunc1>, Pin<PB22, AltFunc1>) {}

impl Pins<pac::Uartb> for (Pin<PA3, AltFunc2>, Pin<PA2, AltFunc2>) {}
impl Pins<pac::Uartb> for (Pin<PA19, AltFunc3>, Pin<PA18, AltFunc3>) {}
impl Pins<pac::Uartb> for (Pin<PA27, AltFunc3>, Pin<PA26, AltFunc3>) {}

impl Pins<pac::Uartb> for (Pin<PB7, AltFunc1>, Pin<PB6, AltFunc1>) {}
impl Pins<pac::Uartb> for (Pin<PB19, AltFunc2>, Pin<PB18, AltFunc2>) {}
impl Pins<pac::Uartb> for (Pin<PB21, AltFunc1>, Pin<PB20, AltFunc1>) {}

//==================================================================================================
// Regular Definitions
//==================================================================================================

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Overrun,
    FramingError,
    ParityError,
    BreakCondition,
    TransferPending,
    BufferTooShort,
    /// Can be a combination of overrun, framing or parity error
    IrqError,
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        embedded_hal_nb::serial::ErrorKind::Other
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum Event {
    // Receiver FIFO interrupt enable. Generates interrupt
    // when FIFO is at least half full. Half full is defined as FIFO
    // count >= RXFIFOIRQTRG
    RxFifoHalfFull,
    // Framing error, Overrun error, Parity Error and Break error
    RxError,
    // Event for timeout condition: Data in the FIFO and no receiver
    // FIFO activity for 4 character times
    RxTimeout,

    // Transmitter FIFO interrupt enable. Generates interrupt
    // when FIFO is at least half full. Half full is defined as FIFO
    // count >= TXFIFOIRQTRG
    TxFifoHalfFull,
    // FIFO overflow error
    TxError,
    // Generate interrupt when transmit FIFO is empty and TXBUSY is 0
    TxEmpty,
    // Interrupt when CTSn changes value
    TxCts,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Parity {
    None,
    Odd,
    Even,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum StopBits {
    One = 0,
    Two = 1,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum WordSize {
    Five = 0,
    Six = 1,
    Seven = 2,
    Eight = 3,
}

pub struct Config {
    pub baudrate: Hertz,
    pub parity: Parity,
    pub stopbits: StopBits,
    // When false, use standard 16x baud clock, other 8x baud clock
    pub baud8: bool,
    pub wordsize: WordSize,
    pub enable_tx: bool,
    pub enable_rx: bool,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Hertz) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::None;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::Even;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::Odd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }

    pub fn wordsize(mut self, wordsize: WordSize) -> Self {
        self.wordsize = wordsize;
        self
    }

    pub fn baud8(mut self, baud: bool) -> Self {
        self.baud8 = baud;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        let baudrate = 115_200_u32.Hz();
        Config {
            baudrate,
            parity: Parity::None,
            stopbits: StopBits::One,
            baud8: false,
            wordsize: WordSize::Eight,
            enable_tx: true,
            enable_rx: true,
        }
    }
}

impl From<Hertz> for Config {
    fn from(baud: Hertz) -> Self {
        Config::default().baudrate(baud)
    }
}

//==================================================================================================
// IRQ Definitions
//==================================================================================================

struct IrqInfo {
    rx_len: usize,
    rx_idx: usize,
    irq_cfg: IrqCfg,
    mode: IrqReceptionMode,
}

pub enum IrqResultMask {
    Complete = 0,
    Overflow = 1,
    FramingError = 2,
    ParityError = 3,
    Break = 4,
    Timeout = 5,
    Addr9 = 6,
    /// Should not happen
    Unknown = 7,
}

/// This struct is used to return the default IRQ handler result to the user
#[derive(Debug, Default)]
pub struct IrqResult {
    raw_res: u32,
    pub bytes_read: usize,
}

impl IrqResult {
    pub const fn new() -> Self {
        IrqResult {
            raw_res: 0,
            bytes_read: 0,
        }
    }
}

impl IrqResult {
    #[inline]
    pub fn raw_result(&self) -> u32 {
        self.raw_res
    }

    #[inline]
    pub(crate) fn clear_result(&mut self) {
        self.raw_res = 0;
    }
    #[inline]
    pub(crate) fn set_result(&mut self, flag: IrqResultMask) {
        self.raw_res |= 1 << flag as u32;
    }

    #[inline]
    pub fn complete(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Complete as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn error(&self) -> bool {
        if self.overflow_error() || self.framing_error() || self.parity_error() {
            return true;
        }
        false
    }

    #[inline]
    pub fn overflow_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Overflow as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn framing_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::FramingError as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn parity_error(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::ParityError as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }

    #[inline]
    pub fn timeout(&self) -> bool {
        if ((self.raw_res >> IrqResultMask::Timeout as u32) & 0x01) == 0x01 {
            return true;
        }
        false
    }
}

#[derive(Debug, PartialEq)]
enum IrqReceptionMode {
    Idle,
    Pending,
}

//==================================================================================================
// UART implementation
//==================================================================================================

/// Type erased variant of a UART. Can be created with the [`Uart::downgrade`] function.
pub struct UartBase<Uart> {
    uart: Uart,
    tx: Tx<Uart>,
    rx: Rx<Uart>,
}
/// Serial abstraction. Entry point to create a new UART
pub struct Uart<Uart, Pins> {
    inner: UartBase<Uart>,
    pins: Pins,
}

/// UART using the IRQ capabilities of the peripheral. Can be created with the
/// [`Uart::into_uart_with_irq`] function.
pub struct UartWithIrq<Uart, Pins> {
    irq_base: UartWithIrqBase<Uart>,
    pins: Pins,
}

/// Type-erased UART using the IRQ capabilities of the peripheral. Can be created with the
/// [`UartWithIrq::downgrade`] function.
pub struct UartWithIrqBase<Uart> {
    pub uart: UartBase<Uart>,
    irq_info: IrqInfo,
}

/// Serial receiver
pub struct Rx<Uart> {
    _usart: PhantomData<Uart>,
}

/// Serial transmitter
pub struct Tx<Uart> {
    _usart: PhantomData<Uart>,
}

impl<UART> Rx<UART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

impl<UART> Tx<UART> {
    fn new() -> Self {
        Self {
            _usart: PhantomData,
        }
    }
}

pub trait Instance: Deref<Target = uart_base::RegisterBlock> {
    fn ptr() -> *const uart_base::RegisterBlock;
    const IDX: u8;
    const PERIPH_SEL: PeripheralSelect;
}

impl<UART: Instance> UartBase<UART> {
    /// This function assumes that the peripheral clock was alredy enabled
    /// in the SYSCONFIG register
    fn init(self, config: Config, sys_clk: Hertz) -> Self {
        let baud_multiplier = match config.baud8 {
            false => 16,
            true => 8,
        };

        // This is the calculation: (64.0 * (x - integer_part as f32) + 0.5) as u32 without floating
        // point calculations.
        let frac = ((sys_clk.raw() % (config.baudrate.raw() * 16)) * 64
            + (config.baudrate.raw() * 8))
            / (config.baudrate.raw() * 16);
        // Calculations here are derived from chapter 4.8.5 (p.79) of the datasheet.
        let x = sys_clk.raw() as f32 / (config.baudrate.raw() * baud_multiplier) as f32;
        let integer_part = x as u32;
        self.uart.clkscale().write(|w| unsafe {
            w.frac().bits(frac as u8);
            w.int().bits(integer_part)
        });

        let (paren, pareven) = match config.parity {
            Parity::None => (false, false),
            Parity::Odd => (true, false),
            Parity::Even => (true, true),
        };
        let stopbits = match config.stopbits {
            StopBits::One => false,
            StopBits::Two => true,
        };
        let wordsize = config.wordsize as u8;
        let baud8 = config.baud8;
        self.uart.ctrl().write(|w| {
            w.paren().bit(paren);
            w.pareven().bit(pareven);
            w.stopbits().bit(stopbits);
            w.baud8().bit(baud8);
            unsafe { w.wordsize().bits(wordsize) }
        });
        let (txenb, rxenb) = (config.enable_tx, config.enable_rx);
        // Clear the FIFO
        self.uart.fifo_clr().write(|w| {
            w.rxfifo().set_bit();
            w.txfifo().set_bit()
        });
        self.uart.enable().write(|w| {
            w.rxenable().bit(rxenb);
            w.txenable().bit(txenb)
        });
        self
    }

    #[inline]
    pub fn enable_rx(&mut self) {
        self.uart.enable().modify(|_, w| w.rxenable().set_bit());
    }

    #[inline]
    pub fn disable_rx(&mut self) {
        self.uart.enable().modify(|_, w| w.rxenable().clear_bit());
    }

    #[inline]
    pub fn enable_tx(&mut self) {
        self.uart.enable().modify(|_, w| w.txenable().set_bit());
    }

    #[inline]
    pub fn disable_tx(&mut self) {
        self.uart.enable().modify(|_, w| w.txenable().clear_bit());
    }

    #[inline]
    pub fn clear_rx_fifo(&mut self) {
        self.uart.fifo_clr().write(|w| w.rxfifo().set_bit());
    }

    #[inline]
    pub fn clear_tx_fifo(&mut self) {
        self.uart.fifo_clr().write(|w| w.txfifo().set_bit());
    }

    #[inline]
    pub fn clear_rx_status(&mut self) {
        self.uart.fifo_clr().write(|w| w.rxsts().set_bit());
    }

    #[inline]
    pub fn clear_tx_status(&mut self) {
        self.uart.fifo_clr().write(|w| w.txsts().set_bit());
    }

    pub fn listen(&self, event: Event) {
        self.uart.irq_enb().modify(|_, w| match event {
            Event::RxError => w.irq_rx_status().set_bit(),
            Event::RxFifoHalfFull => w.irq_rx().set_bit(),
            Event::RxTimeout => w.irq_rx_to().set_bit(),
            Event::TxEmpty => w.irq_tx_empty().set_bit(),
            Event::TxError => w.irq_tx_status().set_bit(),
            Event::TxFifoHalfFull => w.irq_tx().set_bit(),
            Event::TxCts => w.irq_tx_cts().set_bit(),
        });
    }

    pub fn unlisten(&self, event: Event) {
        self.uart.irq_enb().modify(|_, w| match event {
            Event::RxError => w.irq_rx_status().clear_bit(),
            Event::RxFifoHalfFull => w.irq_rx().clear_bit(),
            Event::RxTimeout => w.irq_rx_to().clear_bit(),
            Event::TxEmpty => w.irq_tx_empty().clear_bit(),
            Event::TxError => w.irq_tx_status().clear_bit(),
            Event::TxFifoHalfFull => w.irq_tx().clear_bit(),
            Event::TxCts => w.irq_tx_cts().clear_bit(),
        });
    }

    pub fn release(self) -> UART {
        // Clear the FIFO
        self.uart.fifo_clr().write(|w| {
            w.rxfifo().set_bit();
            w.txfifo().set_bit()
        });
        self.uart.enable().write(|w| {
            w.rxenable().clear_bit();
            w.txenable().clear_bit()
        });
        self.uart
    }

    pub fn split(self) -> (Tx<UART>, Rx<UART>) {
        (self.tx, self.rx)
    }
}

impl<UartInstance, PinsInstance> Uart<UartInstance, PinsInstance>
where
    UartInstance: Instance,
    PinsInstance: Pins<UartInstance>,
{
    pub fn new(
        syscfg: &mut va108xx::Sysconfig,
        sys_clk: impl Into<Hertz>,
        uart: UartInstance,
        pins: PinsInstance,
        config: impl Into<Config>,
    ) -> Self {
        crate::clock::enable_peripheral_clock(syscfg, UartInstance::PERIPH_SEL);
        Uart {
            inner: UartBase {
                uart,
                tx: Tx::new(),
                rx: Rx::new(),
            },
            pins,
        }
        .init(config.into(), sys_clk.into())
    }

    /// This function assumes that the peripheral clock was alredy enabled
    /// in the SYSCONFIG register
    fn init(mut self, config: Config, sys_clk: Hertz) -> Self {
        self.inner = self.inner.init(config, sys_clk);
        self
    }

    /// If the IRQ capabilities of the peripheral are used, the UART needs to be converted
    /// with this function
    pub fn into_uart_with_irq(
        self,
        irq_cfg: IrqCfg,
        sys_cfg: Option<&mut pac::Sysconfig>,
        irq_sel: Option<&mut pac::Irqsel>,
    ) -> UartWithIrq<UartInstance, PinsInstance> {
        let (uart, pins) = self.downgrade_internal();
        UartWithIrq {
            pins,
            irq_base: UartWithIrqBase {
                uart,
                irq_info: IrqInfo {
                    rx_len: 0,
                    rx_idx: 0,
                    irq_cfg,
                    mode: IrqReceptionMode::Idle,
                },
            }
            .init(sys_cfg, irq_sel),
        }
    }

    #[inline]
    pub fn enable_rx(&mut self) {
        self.inner.enable_rx();
    }

    #[inline]
    pub fn disable_rx(&mut self) {
        self.inner.enable_rx();
    }

    #[inline]
    pub fn enable_tx(&mut self) {
        self.inner.enable_tx();
    }

    #[inline]
    pub fn disable_tx(&mut self) {
        self.inner.disable_tx();
    }

    #[inline]
    pub fn clear_rx_fifo(&mut self) {
        self.inner.clear_rx_fifo();
    }

    #[inline]
    pub fn clear_tx_fifo(&mut self) {
        self.inner.clear_tx_fifo();
    }

    #[inline]
    pub fn clear_rx_status(&mut self) {
        self.inner.clear_rx_status();
    }

    #[inline]
    pub fn clear_tx_status(&mut self) {
        self.inner.clear_tx_status();
    }

    pub fn listen(&self, event: Event) {
        self.inner.listen(event);
    }

    pub fn unlisten(&self, event: Event) {
        self.inner.unlisten(event);
    }

    pub fn release(self) -> (UartInstance, PinsInstance) {
        (self.inner.release(), self.pins)
    }

    fn downgrade_internal(self) -> (UartBase<UartInstance>, PinsInstance) {
        let base = UartBase {
            uart: self.inner.uart,
            tx: self.inner.tx,
            rx: self.inner.rx,
        };
        (base, self.pins)
    }

    pub fn downgrade(self) -> UartBase<UartInstance> {
        UartBase {
            uart: self.inner.uart,
            tx: self.inner.tx,
            rx: self.inner.rx,
        }
    }

    pub fn split(self) -> (Tx<UartInstance>, Rx<UartInstance>) {
        self.inner.split()
    }
}

impl Instance for pac::Uarta {
    fn ptr() -> *const uart_base::RegisterBlock {
        pac::Uarta::ptr() as *const _
    }
    const IDX: u8 = 0;

    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Uart0;
}

impl Instance for pac::Uartb {
    fn ptr() -> *const uart_base::RegisterBlock {
        pac::Uartb::ptr() as *const _
    }
    const IDX: u8 = 1;

    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Uart1;
}

impl<UART: Instance> UartWithIrqBase<UART> {
    fn init(self, sys_cfg: Option<&mut pac::Sysconfig>, irq_sel: Option<&mut pac::Irqsel>) -> Self {
        if let Some(sys_cfg) = sys_cfg {
            enable_peripheral_clock(sys_cfg, PeripheralClocks::Irqsel)
        }
        if let Some(irq_sel) = irq_sel {
            if self.irq_info.irq_cfg.route {
                irq_sel
                    .uart0(UART::IDX as usize)
                    .write(|w| unsafe { w.bits(self.irq_info.irq_cfg.irq as u32) });
            }
        }
        self
    }

    /// This initializes a non-blocking read transfer using the IRQ capabilities of the UART
    /// peripheral.
    ///
    /// The only required information is the maximum length for variable sized reception
    /// or the expected length for fixed length reception. If variable sized packets are expected,
    /// the timeout functionality of the IRQ should be enabled as well. After calling this function,
    /// the [`irq_handler`](Self::irq_handler) function should be called in the user interrupt
    /// handler to read the received packets and reinitiate another transfer if desired.
    pub fn read_fixed_len_using_irq(
        &mut self,
        max_len: usize,
        enb_timeout_irq: bool,
    ) -> Result<(), Error> {
        if self.irq_info.mode != IrqReceptionMode::Idle {
            return Err(Error::TransferPending);
        }
        self.irq_info.mode = IrqReceptionMode::Pending;
        self.irq_info.rx_idx = 0;
        self.irq_info.rx_len = max_len;
        self.uart.enable_rx();
        self.uart.enable_tx();
        self.enable_rx_irq_sources(enb_timeout_irq);
        if self.irq_info.irq_cfg.enable {
            unmask_irq(self.irq_info.irq_cfg.irq);
        }
        Ok(())
    }

    #[inline]
    fn enable_rx_irq_sources(&mut self, timeout: bool) {
        self.uart.uart.irq_enb().modify(|_, w| {
            if timeout {
                w.irq_rx_to().set_bit();
            }
            w.irq_rx_status().set_bit();
            w.irq_rx().set_bit()
        });
    }

    #[inline]
    fn disable_rx_irq_sources(&mut self) {
        self.uart.uart.irq_enb().modify(|_, w| {
            w.irq_rx_to().clear_bit();
            w.irq_rx_status().clear_bit();
            w.irq_rx().clear_bit()
        });
    }

    #[inline]
    pub fn enable_tx(&mut self) {
        self.uart.enable_tx()
    }

    #[inline]
    pub fn disable_tx(&mut self) {
        self.uart.disable_tx()
    }

    pub fn cancel_transfer(&mut self) {
        // Disable IRQ
        cortex_m::peripheral::NVIC::mask(self.irq_info.irq_cfg.irq);
        self.disable_rx_irq_sources();
        self.uart.clear_tx_fifo();
        self.irq_info.rx_idx = 0;
        self.irq_info.rx_len = 0;
    }

    /// Default IRQ handler which can be used to read the packets arriving on the UART peripheral.
    ///
    /// If passed buffer is equal to or larger than the specified maximum length, an
    /// [`Error::BufferTooShort`] will be returned
    pub fn irq_handler(&mut self, res: &mut IrqResult, buf: &mut [u8]) -> Result<(), Error> {
        if buf.len() < self.irq_info.rx_len {
            return Err(Error::BufferTooShort);
        }

        let irq_end = self.uart.uart.irq_end().read();
        let enb_status = self.uart.uart.enable().read();
        let rx_enabled = enb_status.rxenable().bit_is_set();
        let _tx_enabled = enb_status.txenable().bit_is_set();
        let read_handler =
            |res: &mut IrqResult, read_res: nb::Result<u8, Error>| -> Result<Option<u8>, Error> {
                match read_res {
                    Ok(byte) => Ok(Some(byte)),
                    Err(nb::Error::WouldBlock) => Ok(None),
                    Err(nb::Error::Other(e)) => match e {
                        Error::Overrun => {
                            res.set_result(IrqResultMask::Overflow);
                            Err(Error::IrqError)
                        }
                        Error::FramingError => {
                            res.set_result(IrqResultMask::FramingError);
                            Err(Error::IrqError)
                        }
                        Error::ParityError => {
                            res.set_result(IrqResultMask::ParityError);
                            Err(Error::IrqError)
                        }
                        _ => {
                            res.set_result(IrqResultMask::Unknown);
                            Err(Error::IrqError)
                        }
                    },
                }
            };
        if irq_end.irq_rx().bit_is_set() {
            // If this interrupt bit is set, the trigger level is available at the very least.
            // Read everything as fast as possible
            for _ in 0..core::cmp::min(
                self.uart.uart.rxfifoirqtrg().read().bits() as usize,
                self.irq_info.rx_len,
            ) {
                buf[self.irq_info.rx_idx] = (self.uart.uart.data().read().bits() & 0xff) as u8;
                self.irq_info.rx_idx += 1;
            }

            // While there is data in the FIFO, write it into the reception buffer
            loop {
                if self.irq_info.rx_idx == self.irq_info.rx_len {
                    self.irq_completion_handler(res);
                    return Ok(());
                }
                if let Some(byte) = read_handler(res, self.uart.read())? {
                    buf[self.irq_info.rx_idx] = byte;
                    self.irq_info.rx_idx += 1;
                } else {
                    break;
                }
            }
        }

        // RX transfer not complete, check for RX errors
        if (self.irq_info.rx_idx < self.irq_info.rx_len) && rx_enabled {
            // Read status register again, might have changed since reading received data
            let rx_status = self.uart.uart.rxstatus().read();
            res.clear_result();
            if rx_status.rxovr().bit_is_set() {
                res.set_result(IrqResultMask::Overflow);
            }
            if rx_status.rxfrm().bit_is_set() {
                res.set_result(IrqResultMask::FramingError);
            }
            if rx_status.rxpar().bit_is_set() {
                res.set_result(IrqResultMask::ParityError);
            }
            if rx_status.rxbrk().bit_is_set() {
                res.set_result(IrqResultMask::Break);
            }
            if rx_status.rxto().bit_is_set() {
                // A timeout has occured but there might be some leftover data in the FIFO,
                // so read that data as well
                while let Some(byte) = read_handler(res, self.uart.read())? {
                    buf[self.irq_info.rx_idx] = byte;
                    self.irq_info.rx_idx += 1;
                }
                self.irq_completion_handler(res);
                res.set_result(IrqResultMask::Timeout);
                return Ok(());
            }

            // If it is not a timeout, it's an error
            if res.raw_res != 0 {
                self.disable_rx_irq_sources();
                return Err(Error::IrqError);
            }
        }

        // Clear the interrupt status bits
        self.uart
            .uart
            .irq_clr()
            .write(|w| unsafe { w.bits(irq_end.bits()) });
        Ok(())
    }

    fn irq_completion_handler(&mut self, res: &mut IrqResult) {
        self.disable_rx_irq_sources();
        self.uart.disable_rx();
        res.bytes_read = self.irq_info.rx_idx;
        res.clear_result();
        res.set_result(IrqResultMask::Complete);
        self.irq_info.mode = IrqReceptionMode::Idle;
        self.irq_info.rx_idx = 0;
        self.irq_info.rx_len = 0;
    }

    pub fn release(self) -> UART {
        self.uart.release()
    }
}

impl<UART: Instance, PINS> UartWithIrq<UART, PINS> {
    /// See [`UartWithIrqBase::read_fixed_len_using_irq`] doc
    pub fn read_fixed_len_using_irq(
        &mut self,
        max_len: usize,
        enb_timeout_irq: bool,
    ) -> Result<(), Error> {
        self.irq_base
            .read_fixed_len_using_irq(max_len, enb_timeout_irq)
    }

    pub fn cancel_transfer(&mut self) {
        self.irq_base.cancel_transfer()
    }

    /// See [`UartWithIrqBase::irq_handler`] doc
    pub fn irq_handler(&mut self, res: &mut IrqResult, buf: &mut [u8]) -> Result<(), Error> {
        self.irq_base.irq_handler(res, buf)
    }

    pub fn release(self) -> (UART, PINS) {
        (self.irq_base.release(), self.pins)
    }

    pub fn downgrade(self) -> (UartWithIrqBase<UART>, PINS) {
        (self.irq_base, self.pins)
    }
}

/*
uart_impl! {
    pac::Uarta: (uarta, clock::PeripheralClocks::Uart0),
    pac::Uartb: (uartb, clock::PeripheralClocks::Uart1),
}
*/

impl<Uart> Tx<Uart> where Uart: Instance {}

impl<UartInstance> embedded_io::ErrorType for UartBase<UartInstance> {
    type Error = Error;
}

impl<UartInstance> embedded_hal_nb::serial::ErrorType for UartBase<UartInstance> {
    type Error = Error;
}

impl<Uart> embedded_hal_nb::serial::ErrorType for Rx<Uart> {
    type Error = Error;
}

impl<Uart> embedded_io::ErrorType for Rx<Uart> {
    type Error = Error;
}

impl<Uart> embedded_io::ErrorType for Tx<Uart> {
    type Error = Error;
}

impl<Uart> embedded_hal_nb::serial::ErrorType for Tx<Uart> {
    type Error = Error;
}

impl<Uart: Instance> embedded_hal_nb::serial::Write<u8> for Tx<Uart> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        let reader = unsafe { &(*Uart::ptr()) }.txstatus().read();
        if reader.wrrdy().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        } else {
            // DPARITY bit not supported yet
            unsafe {
                // NOTE(unsafe) atomic write to data register
                (*Uart::ptr()).data().write(|w| w.bits(word as u32));
            }
        }
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // SAFETY: Only TX related registers are used.
        let reader = unsafe { &(*Uart::ptr()) }.txstatus().read();
        if reader.wrbusy().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }
}

impl<Uart: Instance> embedded_io::Write for Tx<Uart> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        for byte in buf.iter() {
            nb::block!(<Self as embedded_hal_nb::serial::Write<u8>>::write(
                self, *byte
            ))?;
        }

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(<Self as embedded_hal_nb::serial::Write<u8>>::flush(self))
    }
}

impl<Uart: Instance> embedded_io::Write for UartBase<Uart> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<Uart: Instance> embedded_hal_nb::serial::Read<u8> for UartBase<Uart> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read()
    }
}

impl<Uart: Instance> embedded_hal_nb::serial::Write<u8> for UartBase<Uart> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }
}

impl<Uart: Instance> embedded_hal_nb::serial::Read<u8> for Rx<Uart> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let uart = unsafe { &(*Uart::ptr()) };
        let status_reader = uart.rxstatus().read();
        let err = if status_reader.rxovr().bit_is_set() {
            Some(Error::Overrun)
        } else if status_reader.rxfrm().bit_is_set() {
            Some(Error::FramingError)
        } else if status_reader.rxpar().bit_is_set() {
            Some(Error::ParityError)
        } else {
            None
        };
        if let Some(err) = err {
            // The status code is always related to the next bit for the framing
            // and parity status bits. We have to read the DATA register
            // so that the next status reflects the next DATA word
            // For overrun error, we read as well to clear the peripheral
            uart.data().read().bits();
            Err(err.into())
        } else if status_reader.rdavl().bit_is_set() {
            let data = uart.data().read().bits();
            Ok((data & 0xff) as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<Uart: Instance> embedded_io::Read for Rx<Uart> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        for byte in buf.iter_mut() {
            let w = nb::block!(<Self as embedded_hal_nb::serial::Read<u8>>::read(self))?;
            *byte = w;
        }

        Ok(buf.len())
    }
}
