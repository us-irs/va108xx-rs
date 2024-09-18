//! SPI example application
#![no_main]
#![no_std]

use core::cell::RefCell;

use cortex_m_rt::entry;
use embedded_hal::{
    delay::DelayNs,
    spi::{Mode, SpiBus, MODE_0},
};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::{PinsA, PinsB},
    pac::{self, interrupt},
    prelude::*,
    pwm::{default_ms_irq_handler, set_up_ms_tick},
    spi::{self, Spi, SpiBase, SpiClkConfig, TransferConfigWithHwcs},
    IrqCfg,
};

#[derive(PartialEq, Debug)]
pub enum ExampleSelect {
    // Enter loopback mode. It is not necessary to tie MOSI/MISO together for this
    Loopback,
    MosiMisoTiedTogetherManually,
}

#[derive(PartialEq, Debug)]
pub enum SpiBusSelect {
    SpiAPortA,
    SpiAPortB,
    SpiBPortB,
}

const EXAMPLE_SEL: ExampleSelect = ExampleSelect::Loopback;
const SPI_BUS_SEL: SpiBusSelect = SpiBusSelect::SpiBPortB;
const SPI_SPEED_KHZ: u32 = 1000;
const SPI_MODE: Mode = MODE_0;
const BLOCKMODE: bool = true;
const FILL_WORD: u8 = 0x0f;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108xx SPI example application--");
    let mut dp = pac::Peripherals::take().unwrap();
    let mut delay = set_up_ms_tick(
        IrqCfg::new(interrupt::OC0, true, true),
        &mut dp.sysconfig,
        Some(&mut dp.irqsel),
        50.MHz(),
        dp.tim0,
    );

    let spi_clk_cfg = SpiClkConfig::from_clk(50.MHz(), SPI_SPEED_KHZ.kHz())
        .expect("creating SPI clock config failed");
    let spia_ref: RefCell<Option<SpiBase<pac::Spia, u8>>> = RefCell::new(None);
    let spib_ref: RefCell<Option<SpiBase<pac::Spib, u8>>> = RefCell::new(None);
    let pinsa = PinsA::new(&mut dp.sysconfig, None, dp.porta);
    let pinsb = PinsB::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.portb);

    let mut spi_cfg = spi::SpiConfig::default();
    if EXAMPLE_SEL == ExampleSelect::Loopback {
        spi_cfg = spi_cfg.loopback(true)
    }

    // Set up the SPI peripheral
    match SPI_BUS_SEL {
        SpiBusSelect::SpiAPortA => {
            let (sck, mosi, miso) = (
                pinsa.pa31.into_funsel_1(),
                pinsa.pa30.into_funsel_1(),
                pinsa.pa29.into_funsel_1(),
            );
            let mut spia = Spi::new(
                &mut dp.sysconfig,
                50.MHz(),
                dp.spia,
                (sck, miso, mosi),
                spi_cfg,
            );
            spia.set_fill_word(FILL_WORD);
            spia_ref.borrow_mut().replace(spia.downgrade());
        }
        SpiBusSelect::SpiAPortB => {
            let (sck, mosi, miso) = (
                pinsb.pb9.into_funsel_2(),
                pinsb.pb8.into_funsel_2(),
                pinsb.pb7.into_funsel_2(),
            );
            let mut spia = Spi::new(
                &mut dp.sysconfig,
                50.MHz(),
                dp.spia,
                (sck, miso, mosi),
                spi_cfg,
            );
            spia.set_fill_word(FILL_WORD);
            spia_ref.borrow_mut().replace(spia.downgrade());
        }
        SpiBusSelect::SpiBPortB => {
            let (sck, mosi, miso) = (
                pinsb.pb5.into_funsel_1(),
                pinsb.pb4.into_funsel_1(),
                pinsb.pb3.into_funsel_1(),
            );
            let mut spib = Spi::new(
                &mut dp.sysconfig,
                50.MHz(),
                dp.spib,
                (sck, miso, mosi),
                spi_cfg,
            );
            spib.set_fill_word(FILL_WORD);
            spib_ref.borrow_mut().replace(spib.downgrade());
        }
    }
    // Configure transfer specific properties here
    match SPI_BUS_SEL {
        SpiBusSelect::SpiAPortA | SpiBusSelect::SpiAPortB => {
            if let Some(ref mut spi) = *spia_ref.borrow_mut() {
                let transfer_cfg = TransferConfigWithHwcs::new_no_hw_cs(
                    Some(spi_clk_cfg),
                    Some(SPI_MODE),
                    BLOCKMODE,
                    false,
                );
                spi.cfg_transfer(&transfer_cfg);
            }
        }
        SpiBusSelect::SpiBPortB => {
            if let Some(ref mut spi) = *spib_ref.borrow_mut() {
                let hw_cs_pin = pinsb.pb2.into_funsel_1();
                let transfer_cfg = TransferConfigWithHwcs::new(
                    Some(spi_clk_cfg),
                    Some(SPI_MODE),
                    Some(hw_cs_pin),
                    BLOCKMODE,
                    false,
                );
                spi.cfg_transfer(&transfer_cfg);
            }
        }
    }

    // Application logic
    loop {
        let mut reply_buf: [u8; 8] = [0; 8];
        match SPI_BUS_SEL {
            SpiBusSelect::SpiAPortA | SpiBusSelect::SpiAPortB => {
                if let Some(ref mut spi) = *spia_ref.borrow_mut() {
                    // Can't really verify correct reply here.
                    spi.write(&[0x42]).expect("write failed");
                    // Because of the loopback mode, we should get back the fill word here.
                    spi.read(&mut reply_buf[0..1]).unwrap();
                    assert_eq!(reply_buf[0], FILL_WORD);
                    delay.delay_ms(500_u32);

                    let tx_buf: [u8; 3] = [0x01, 0x02, 0x03];
                    spi.transfer(&mut reply_buf[0..3], &tx_buf).unwrap();
                    assert_eq!(tx_buf, reply_buf[0..3]);
                    rprintln!(
                        "Received reply: {}, {}, {}",
                        reply_buf[0],
                        reply_buf[1],
                        reply_buf[2]
                    );
                    delay.delay_ms(500_u32);

                    let mut tx_rx_buf: [u8; 3] = [0x03, 0x02, 0x01];
                    spi.transfer_in_place(&mut tx_rx_buf).unwrap();
                    rprintln!(
                        "Received reply: {}, {}, {}",
                        tx_rx_buf[0],
                        tx_rx_buf[1],
                        tx_rx_buf[2]
                    );
                    assert_eq!(&tx_rx_buf[0..3], &[0x03, 0x02, 0x01]);
                }
            }
            SpiBusSelect::SpiBPortB => {
                if let Some(ref mut spi) = *spib_ref.borrow_mut() {
                    // Can't really verify correct reply here.
                    spi.write(&[0x42]).expect("write failed");
                    // Because of the loopback mode, we should get back the fill word here.
                    spi.read(&mut reply_buf[0..1]).unwrap();
                    assert_eq!(reply_buf[0], FILL_WORD);
                    delay.delay_ms(500_u32);

                    let tx_buf: [u8; 3] = [0x01, 0x02, 0x03];
                    spi.transfer(&mut reply_buf[0..3], &tx_buf).unwrap();
                    assert_eq!(tx_buf, reply_buf[0..3]);
                    rprintln!(
                        "Received reply: {}, {}, {}",
                        reply_buf[0],
                        reply_buf[1],
                        reply_buf[2]
                    );
                    delay.delay_ms(500_u32);

                    let mut tx_rx_buf: [u8; 3] = [0x03, 0x02, 0x01];
                    spi.transfer_in_place(&mut tx_rx_buf).unwrap();
                    rprintln!(
                        "Received reply: {}, {}, {}",
                        tx_rx_buf[0],
                        tx_rx_buf[1],
                        tx_rx_buf[2]
                    );
                    assert_eq!(&tx_rx_buf[0..3], &[0x03, 0x02, 0x01]);
                }
            }
        }
    }
}

#[interrupt]
#[allow(non_snake_case)]
fn OC0() {
    default_ms_irq_handler()
}
