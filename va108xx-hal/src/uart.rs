//! # API for the UART peripheral
//!
//! ## Examples
//!
//! - [UART simple example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/examples/simple/examples/uart.rs)
//! - [UART with IRQ and RTIC](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/va108xx-update-package/examples/rtic/src/bin/uart-rtic.rs)
use core::{convert::Infallible, ops::Deref};
use fugit::RateExtU32;
use va108xx::Uarta;

pub use crate::IrqCfg;
use crate::{
    clock::enable_peripheral_clock,
    enable_interrupt,
    gpio::pin::{
        AltFunc1, AltFunc2, AltFunc3, Pin, PA16, PA17, PA18, PA19, PA2, PA26, PA27, PA3, PA30,
        PA31, PA8, PA9, PB18, PB19, PB20, PB21, PB22, PB23, PB6, PB7, PB8, PB9,
    },
    pac::{self, uarta as uart_base},
    time::Hertz,
    PeripheralSelect,
};
use embedded_hal_nb::serial::Read;

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

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TransferPendingError;

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxError {
    Overrun,
    Framing,
    Parity,
}
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Rx(RxError),
    BreakCondition,
}

impl From<RxError> for Error {
    fn from(value: RxError) -> Self {
        Self::Rx(value)
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl embedded_io::Error for RxError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}
impl embedded_hal_nb::serial::Error for RxError {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            RxError::Overrun => embedded_hal_nb::serial::ErrorKind::Overrun,
            RxError::Framing => embedded_hal_nb::serial::ErrorKind::FrameFormat,
            RxError::Parity => embedded_hal_nb::serial::ErrorKind::Parity,
        }
    }
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        match self {
            Error::Rx(rx_error) => embedded_hal_nb::serial::Error::kind(rx_error),
            Error::BreakCondition => embedded_hal_nb::serial::ErrorKind::Other,
        }
    }
}

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    None,
    Odd,
    Even,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    One = 0,
    Two = 1,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WordSize {
    Five = 0,
    Six = 1,
    Seven = 2,
    Eight = 3,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[derive(Debug, Copy, Clone)]
pub struct IrqContextTimeoutOrMaxSize {
    rx_idx: usize,
    mode: IrqReceptionMode,
    pub max_len: usize,
}

impl IrqContextTimeoutOrMaxSize {
    pub fn new(max_len: usize) -> Self {
        IrqContextTimeoutOrMaxSize {
            rx_idx: 0,
            max_len,
            mode: IrqReceptionMode::Idle,
        }
    }
}

impl IrqContextTimeoutOrMaxSize {
    pub fn reset(&mut self) {
        self.rx_idx = 0;
        self.mode = IrqReceptionMode::Idle;
    }
}

/// This struct is used to return the default IRQ handler result to the user
#[derive(Debug, Default)]
pub struct IrqResult {
    pub bytes_read: usize,
    pub errors: Option<IrqUartError>,
}

/// This struct is used to return the default IRQ handler result to the user
#[derive(Debug, Default)]
pub struct IrqResultMaxSizeOrTimeout {
    complete: bool,
    timeout: bool,
    pub errors: Option<IrqUartError>,
    pub bytes_read: usize,
}

impl IrqResultMaxSizeOrTimeout {
    pub fn new() -> Self {
        IrqResultMaxSizeOrTimeout {
            complete: false,
            timeout: false,
            errors: None,
            bytes_read: 0,
        }
    }
}
impl IrqResultMaxSizeOrTimeout {
    #[inline]
    pub fn has_errors(&self) -> bool {
        self.errors.is_some()
    }

    #[inline]
    pub fn overflow_error(&self) -> bool {
        self.errors.map_or(false, |e| e.overflow)
    }

    #[inline]
    pub fn framing_error(&self) -> bool {
        self.errors.map_or(false, |e| e.framing)
    }

    #[inline]
    pub fn parity_error(&self) -> bool {
        self.errors.map_or(false, |e| e.parity)
    }

    #[inline]
    pub fn timeout(&self) -> bool {
        self.timeout
    }

    #[inline]
    pub fn complete(&self) -> bool {
        self.complete
    }
}

#[derive(Debug, PartialEq, Copy, Clone)]
enum IrqReceptionMode {
    Idle,
    Pending,
}

#[derive(Default, Debug, Copy, Clone)]
pub struct IrqUartError {
    overflow: bool,
    framing: bool,
    parity: bool,
    other: bool,
}

impl IrqUartError {
    #[inline(always)]
    pub fn overflow(&self) -> bool {
        self.overflow
    }

    #[inline(always)]
    pub fn framing(&self) -> bool {
        self.framing
    }

    #[inline(always)]
    pub fn parity(&self) -> bool {
        self.parity
    }

    #[inline(always)]
    pub fn other(&self) -> bool {
        self.other
    }
}

impl IrqUartError {
    #[inline(always)]
    pub fn error(&self) -> bool {
        self.overflow || self.framing || self.parity
    }
}

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BufferTooShortError {
    found: usize,
    expected: usize,
}

//==================================================================================================
// UART peripheral wrapper
//==================================================================================================

pub trait Instance: Deref<Target = uart_base::RegisterBlock> {
    const IDX: u8;
    const PERIPH_SEL: PeripheralSelect;

    /// Retrieve the peripheral structure.
    ///
    /// # Safety
    ///
    /// This circumvents the safety guarantees of the HAL.
    unsafe fn steal() -> Self;
    fn ptr() -> *const uart_base::RegisterBlock;
}

impl Instance for pac::Uarta {
    const IDX: u8 = 0;

    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Uart0;

    unsafe fn steal() -> Self {
        pac::Peripherals::steal().uarta
    }
    fn ptr() -> *const uart_base::RegisterBlock {
        Uarta::ptr() as *const _
    }
}

impl Instance for pac::Uartb {
    const IDX: u8 = 1;

    const PERIPH_SEL: PeripheralSelect = PeripheralSelect::Uart1;

    unsafe fn steal() -> Self {
        pac::Peripherals::steal().uartb
    }
    fn ptr() -> *const uart_base::RegisterBlock {
        Uarta::ptr() as *const _
    }
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

impl<Uart: Instance> UartBase<Uart> {
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

    pub fn release(self) -> Uart {
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

    pub fn split(self) -> (Tx<Uart>, Rx<Uart>) {
        (self.tx, self.rx)
    }
}

impl<UartInstance> embedded_io::ErrorType for UartBase<UartInstance> {
    type Error = Error;
}

impl<UartInstance> embedded_hal_nb::serial::ErrorType for UartBase<UartInstance> {
    type Error = Error;
}

impl<Uart: Instance> embedded_hal_nb::serial::Read<u8> for UartBase<Uart> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx.read().map_err(|e| e.map(Error::Rx))
    }
}

impl<Uart: Instance> embedded_hal_nb::serial::Write<u8> for UartBase<Uart> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.tx.write(word).map_err(|e| {
            if let nb::Error::Other(_) = e {
                unreachable!()
            }
            nb::Error::WouldBlock
        })
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush().map_err(|e| {
            if let nb::Error::Other(_) = e {
                unreachable!()
            }
            nb::Error::WouldBlock
        })
    }
}

/// Serial abstraction. Entry point to create a new UART
pub struct Uart<Uart, Pins> {
    inner: UartBase<Uart>,
    pins: Pins,
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
                tx: Tx::new(unsafe { UartInstance::steal() }),
                rx: Rx::new(unsafe { UartInstance::steal() }),
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

/// Serial receiver.
///
/// Can be created by using the [Uart::split] or [UartBase::split] API.
pub struct Rx<Uart>(Uart);

impl<Uart: Instance> Rx<Uart> {
    fn new(uart: Uart) -> Self {
        Self(uart)
    }

    /// Direct access to the peripheral structure.
    ///
    /// # Safety
    ///
    /// You must ensure that only registers related to the operation of the RX side are used.
    pub unsafe fn uart(&self) -> &Uart {
        &self.0
    }

    #[inline]
    pub fn clear_fifo(&self) {
        self.0.fifo_clr().write(|w| w.rxfifo().set_bit());
    }

    #[inline]
    pub fn enable(&mut self) {
        self.0.enable().modify(|_, w| w.rxenable().set_bit());
    }

    #[inline]
    pub fn disable(&mut self) {
        self.0.enable().modify(|_, w| w.rxenable().clear_bit());
    }

    /// Low level function to read a word from the UART FIFO.
    ///
    /// Uses the [nb] API to allow usage in blocking and non-blocking contexts.
    ///
    /// Please note that you might have to mask the returned value with 0xff to retrieve the actual
    /// value if you use the manual parity mode. See chapter 4.6.2 for more information.
    #[inline(always)]
    pub fn read_fifo(&self) -> nb::Result<u32, Infallible> {
        if self.0.rxstatus().read().rdavl().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(self.read_fifo_unchecked())
    }

    /// Low level function to read a word from from the UART FIFO.
    ///
    /// This does not necesarily mean there is a word in the FIFO available.
    /// Use the [Self::read_fifo] function to read a word from the FIFO reliably using the [nb]
    /// API.
    ///
    /// Please note that you might have to mask the returned value with 0xff to retrieve the actual
    /// value if you use the manual parity mode. See chapter 4.6.2 for more information.
    #[inline(always)]
    pub fn read_fifo_unchecked(&self) -> u32 {
        self.0.data().read().bits()
    }

    pub fn into_rx_with_irq(
        self,
        sysconfig: &mut pac::Sysconfig,
        irqsel: &mut pac::Irqsel,
        interrupt: pac::Interrupt,
    ) -> RxWithIrq<Uart> {
        RxWithIrq::new(self, sysconfig, irqsel, interrupt)
    }

    pub fn release(self) -> Uart {
        self.0
    }
}

impl<Uart> embedded_io::ErrorType for Rx<Uart> {
    type Error = RxError;
}

impl<Uart> embedded_hal_nb::serial::ErrorType for Rx<Uart> {
    type Error = RxError;
}

impl<Uart: Instance> embedded_hal_nb::serial::Read<u8> for Rx<Uart> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let uart = unsafe { &(*Uart::ptr()) };
        let status_reader = uart.rxstatus().read();
        let err = if status_reader.rxovr().bit_is_set() {
            Some(RxError::Overrun)
        } else if status_reader.rxfrm().bit_is_set() {
            Some(RxError::Framing)
        } else if status_reader.rxpar().bit_is_set() {
            Some(RxError::Parity)
        } else {
            None
        };
        if let Some(err) = err {
            // The status code is always related to the next bit for the framing
            // and parity status bits. We have to read the DATA register
            // so that the next status reflects the next DATA word
            // For overrun error, we read as well to clear the peripheral
            self.read_fifo_unchecked();
            return Err(err.into());
        }
        self.read_fifo().map(|val| (val & 0xff) as u8).map_err(|e| {
            if let nb::Error::Other(_) = e {
                unreachable!()
            }
            nb::Error::WouldBlock
        })
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

/// Serial transmitter
///
/// Can be created by using the [Uart::split] or [UartBase::split] API.
pub struct Tx<Uart>(Uart);

impl<Uart: Instance> Tx<Uart> {
    fn new(uart: Uart) -> Self {
        Self(uart)
    }

    /// Direct access to the peripheral structure.
    ///
    /// # Safety
    ///
    /// You must ensure that only registers related to the operation of the TX side are used.
    pub unsafe fn uart(&self) -> &Uart {
        &self.0
    }

    #[inline]
    pub fn clear_fifo(&self) {
        self.0.fifo_clr().write(|w| w.txfifo().set_bit());
    }

    #[inline]
    pub fn enable(&mut self) {
        self.0.enable().modify(|_, w| w.txenable().set_bit());
    }

    #[inline]
    pub fn disable(&mut self) {
        self.0.enable().modify(|_, w| w.txenable().clear_bit());
    }

    /// Low level function to write a word to the UART FIFO.
    ///
    /// Uses the [nb] API to allow usage in blocking and non-blocking contexts.
    ///
    /// Please note that you might have to mask the returned value with 0xff to retrieve the actual
    /// value if you use the manual parity mode. See chapter 11.4.1 for more information.
    #[inline(always)]
    pub fn write_fifo(&self, data: u32) -> nb::Result<(), Infallible> {
        if self.0.txstatus().read().wrrdy().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }
        self.write_fifo_unchecked(data);
        Ok(())
    }

    /// Low level function to write a word to the UART FIFO.
    ///
    /// This does not necesarily mean that the FIFO can process another word because it might be
    /// full.
    /// Use the [Self::read_fifo] function to write a word to the FIFO reliably using the [nb]
    /// API.
    #[inline(always)]
    pub fn write_fifo_unchecked(&self, data: u32) {
        self.0.data().write(|w| unsafe { w.bits(data) });
    }
}

impl<Uart> embedded_io::ErrorType for Tx<Uart> {
    type Error = Infallible;
}

impl<Uart> embedded_hal_nb::serial::ErrorType for Tx<Uart> {
    type Error = Infallible;
}

impl<Uart: Instance> embedded_hal_nb::serial::Write<u8> for Tx<Uart> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_fifo(word as u32)
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

/// Serial receiver, using interrupts to offload reading to the hardware.
///
/// You can use [Rx::to_rx_with_irq] to convert a normal [Rx] structure into this structure.
/// This structure provides two distinct ways to read the UART RX using interrupts. It should
/// be noted that the interrupt service routine (ISR) still has to be provided by the user. However,
/// this structure provides API calls which can be used inside the ISRs to simplify the reading
/// of the UART.
///
///  1. The first way simply empties the FIFO on an interrupt into a user provided buffer. You
///     can simply use [Self::start] to prepare the peripheral and then call the
///     [Self::irq_handler] in the interrupt service routine.
///  2. The second way reads packets bounded by a maximum size or a baudtick based timeout. You
///     can use [Self::read_fixed_len_or_timeout_based_using_irq] to prepare the peripheral and
///     then call the [Self::irq_handler_max_size_or_timeout_based] in the interrupt service
///     routine. You have to call [Self::read_fixed_len_or_timeout_based_using_irq] in the ISR to
///     start reading the next packet.
pub struct RxWithIrq<Uart> {
    pub rx: Rx<Uart>,
    pub interrupt: pac::Interrupt,
}

impl<Uart: Instance> RxWithIrq<Uart> {
    pub fn new(
        rx: Rx<Uart>,
        syscfg: &mut pac::Sysconfig,
        irqsel: &mut pac::Irqsel,
        interrupt: pac::Interrupt,
    ) -> Self {
        enable_peripheral_clock(syscfg, PeripheralSelect::Irqsel);
        irqsel
            .uart0(Uart::IDX as usize)
            .write(|w| unsafe { w.bits(interrupt as u32) });
        Self { rx, interrupt }
    }

    /// This function should be called once at initialization time if the regular
    /// [Self::irq_handler] is used to read the UART receiver to enable and start the receiver.
    pub fn start(&mut self) {
        self.rx.enable();
        self.enable_rx_irq_sources(true);
        unsafe { enable_interrupt(self.interrupt) };
    }

    #[inline(always)]
    pub fn uart(&self) -> &Uart {
        &self.rx.0
    }

    /// This function is used together with the [Self::irq_handler_max_size_or_timeout_based]
    /// function to read packets with a maximum size or variable sized packets by using the
    /// receive timeout of the hardware.
    ///
    /// This function should be called once at initialization to initiate the context state
    /// and to [Self::start] the receiver. After that, it should be called after each
    /// completed [Self::irq_handler_max_size_or_timeout_based] call to restart the reception
    /// of a packet.
    pub fn read_fixed_len_or_timeout_based_using_irq(
        &mut self,
        context: &mut IrqContextTimeoutOrMaxSize,
    ) -> Result<(), TransferPendingError> {
        if context.mode != IrqReceptionMode::Idle {
            return Err(TransferPendingError);
        }
        context.mode = IrqReceptionMode::Pending;
        context.rx_idx = 0;
        self.start();
        Ok(())
    }

    #[inline]
    fn enable_rx_irq_sources(&mut self, timeout: bool) {
        self.uart().irq_enb().modify(|_, w| {
            if timeout {
                w.irq_rx_to().set_bit();
            }
            w.irq_rx_status().set_bit();
            w.irq_rx().set_bit()
        });
    }

    #[inline]
    fn disable_rx_irq_sources(&mut self) {
        self.uart().irq_enb().modify(|_, w| {
            w.irq_rx_to().clear_bit();
            w.irq_rx_status().clear_bit();
            w.irq_rx().clear_bit()
        });
    }

    pub fn cancel_transfer(&mut self) {
        self.disable_rx_irq_sources();
        self.rx.clear_fifo();
    }

    /// This function should be called in the user provided UART interrupt handler.
    ///
    /// It simply empties any bytes in the FIFO into the user provided buffer and returns the
    /// result of the operation.
    ///
    /// This function will not disable the RX interrupts, so you don't need to call any other
    /// API after calling this function to continue emptying the FIFO. RX errors are handled
    /// as partial errors and are returned as part of the [IrqResult].
    pub fn irq_handler(&mut self, buf: &mut [u8; 16]) -> IrqResult {
        let mut result = IrqResult::default();

        let irq_end = self.uart().irq_end().read();
        let enb_status = self.uart().enable().read();
        let rx_enabled = enb_status.rxenable().bit_is_set();

        // Half-Full interrupt. We have a guaranteed amount of data we can read.
        if irq_end.irq_rx().bit_is_set() {
            let available_bytes = self.uart().rxfifoirqtrg().read().bits() as usize;

            // If this interrupt bit is set, the trigger level is available at the very least.
            // Read everything as fast as possible
            for _ in 0..available_bytes {
                buf[result.bytes_read] = (self.uart().data().read().bits() & 0xff) as u8;
                result.bytes_read += 1;
            }
        }

        // Timeout, empty the FIFO completely.
        if irq_end.irq_rx_to().bit_is_set() {
            loop {
                // While there is data in the FIFO, write it into the reception buffer
                let read_result = self.rx.read();
                if let Some(byte) = self.read_handler(&mut result.errors, &read_result) {
                    buf[result.bytes_read] = byte;
                    result.bytes_read += 1;
                } else {
                    break;
                }
            }
        }

        // RX transfer not complete, check for RX errors
        if rx_enabled {
            self.check_for_errors(&mut result.errors);
        }

        // Clear the interrupt status bits
        self.uart()
            .irq_clr()
            .write(|w| unsafe { w.bits(irq_end.bits()) });
        result
    }

    /// This function should be called in the user provided UART interrupt handler.
    ///
    /// This function is used to read packets which either have a maximum size or variable sized
    /// packet which are bounded by sufficient delays between them, triggering a hardware timeout.
    ///
    /// If either the maximum number of packets have been read or a timeout occured, the transfer
    /// will be deemed completed. The state information of the transfer is tracked in the
    /// [IrqContextTimeoutOrMaxSize] structure.
    ///
    /// If passed buffer is equal to or larger than the specified maximum length, an
    /// [BufferTooShortError] will be returned. Other RX errors are treated as partial errors
    /// and returned inside the [IrqResultMaxSizeOrTimeout] structure.
    pub fn irq_handler_max_size_or_timeout_based(
        &mut self,
        context: &mut IrqContextTimeoutOrMaxSize,
        buf: &mut [u8],
    ) -> Result<IrqResultMaxSizeOrTimeout, BufferTooShortError> {
        if buf.len() < context.max_len {
            return Err(BufferTooShortError {
                found: buf.len(),
                expected: context.max_len,
            });
        }
        let mut result = IrqResultMaxSizeOrTimeout::default();

        let irq_end = self.uart().irq_end().read();
        let enb_status = self.uart().enable().read();
        let rx_enabled = enb_status.rxenable().bit_is_set();

        // Half-Full interrupt. We have a guaranteed amount of data we can read.
        if irq_end.irq_rx().bit_is_set() {
            // Determine the number of bytes to read, ensuring we leave 1 byte in the FIFO.
            // We use this trick/hack because the timeout feature of the peripheral relies on data
            // being in the RX FIFO. If data continues arriving, another half-full IRQ will fire.
            // If not, the last byte(s) is/are emptied by the timeout interrupt.
            let available_bytes = self.uart().rxfifoirqtrg().read().bits() as usize;

            let bytes_to_read = core::cmp::min(
                available_bytes.saturating_sub(1),
                context.max_len - context.rx_idx,
            );

            // If this interrupt bit is set, the trigger level is available at the very least.
            // Read everything as fast as possible
            for _ in 0..bytes_to_read {
                buf[context.rx_idx] = (self.uart().data().read().bits() & 0xff) as u8;
                context.rx_idx += 1;
            }

            // On high-baudrates, data might be available immediately, and we possible have to
            // read continuosly? Then again, the CPU should always be faster than that. I'd rather
            // rely on the hardware firing another IRQ. I have not tried baudrates higher than
            // 115200 so far.
        }
        // Timeout, empty the FIFO completely.
        if irq_end.irq_rx_to().bit_is_set() {
            // While there is data in the FIFO, write it into the reception buffer
            loop {
                if context.rx_idx == context.max_len {
                    break;
                }
                let read_result = self.rx.read();
                if let Some(byte) = self.read_handler(&mut result.errors, &read_result) {
                    buf[context.rx_idx] = byte;
                    context.rx_idx += 1;
                } else {
                    break;
                }
            }
            self.irq_completion_handler_max_size_timeout(&mut result, context);
            return Ok(result);
        }

        // RX transfer not complete, check for RX errors
        if (context.rx_idx < context.max_len) && rx_enabled {
            self.check_for_errors(&mut result.errors);
        }

        // Clear the interrupt status bits
        self.uart()
            .irq_clr()
            .write(|w| unsafe { w.bits(irq_end.bits()) });
        Ok(result)
    }

    fn read_handler(
        &self,
        errors: &mut Option<IrqUartError>,
        read_res: &nb::Result<u8, RxError>,
    ) -> Option<u8> {
        match read_res {
            Ok(byte) => Some(*byte),
            Err(nb::Error::WouldBlock) => None,
            Err(nb::Error::Other(e)) => {
                // Ensure `errors` is Some(IrqUartError), initializing if it's None
                let err = errors.get_or_insert(IrqUartError::default());

                // Now we can safely modify fields inside `err`
                match e {
                    RxError::Overrun => err.overflow = true,
                    RxError::Framing => err.framing = true,
                    RxError::Parity => err.parity = true,
                }
                None
            }
        }
    }

    fn check_for_errors(&self, errors: &mut Option<IrqUartError>) {
        let rx_status = self.uart().rxstatus().read();

        if rx_status.rxovr().bit_is_set()
            || rx_status.rxfrm().bit_is_set()
            || rx_status.rxpar().bit_is_set()
        {
            let err = errors.get_or_insert(IrqUartError::default());

            if rx_status.rxovr().bit_is_set() {
                err.overflow = true;
            }
            if rx_status.rxfrm().bit_is_set() {
                err.framing = true;
            }
            if rx_status.rxpar().bit_is_set() {
                err.parity = true;
            }
        }
    }

    fn irq_completion_handler_max_size_timeout(
        &mut self,
        res: &mut IrqResultMaxSizeOrTimeout,
        context: &mut IrqContextTimeoutOrMaxSize,
    ) {
        self.disable_rx_irq_sources();
        self.rx.disable();
        res.bytes_read = context.rx_idx;
        res.complete = true;
        context.mode = IrqReceptionMode::Idle;
        context.rx_idx = 0;
    }

    /// # Safety
    ///
    /// This API allows creating multiple UART instances when releasing the TX structure as well.
    /// The user must ensure that these instances are not used to create multiple overlapping
    /// UART drivers.
    pub unsafe fn release(self) -> Uart {
        self.rx.release()
    }
}

/*

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

impl<Uart: Instance> UartWithIrqBase<Uart> {
    fn init(self, sys_cfg: Option<&mut pac::Sysconfig>, irq_sel: Option<&mut pac::Irqsel>) -> Self {
        if let Some(sys_cfg) = sys_cfg {
            enable_peripheral_clock(sys_cfg, PeripheralClocks::Irqsel)
        }
        if let Some(irq_sel) = irq_sel {
            if self.irq_info.irq_cfg.route {
                irq_sel
                    .uart0(Uart::IDX as usize)
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
            unsafe {
                enable_interrupt(self.irq_info.irq_cfg.irq);
            }
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

    pub fn release(self) -> Uart {
        self.uart.release()
    }
}
*/
