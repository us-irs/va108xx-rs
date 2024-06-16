//! ADXL343 accelerometer example
//!
//! Please note that the default REB1 board is not populated with the ADXL343BCCZ-RL7.
//! To use this example, this chip needs to be soldered onto the board.
#![no_main]
#![no_std]
use cortex_m_rt::entry;
use embedded_hal::spi::SpiBus;
use embedded_hal::{delay::DelayNs, digital::OutputPin};
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{
    gpio::PinsA,
    pac,
    prelude::*,
    spi::{Spi, SpiConfig, TransferConfig},
    timer::set_up_ms_delay_provider,
};

const READ_MASK: u8 = 1 << 7;
const _MULTI_BYTE_MASK: u8 = 1 << 6;
const DEVID_REG: u8 = 0x00;

const POWER_CTL_REG: u8 = 0x2D;
const PWR_MEASUREMENT_MODE_MASK: u8 = 1 << 3;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- Vorago Accelerometer Example --");
    let mut dp = pac::Peripherals::take().unwrap();
    let mut delay = set_up_ms_delay_provider(&mut dp.sysconfig, 50.MHz(), dp.tim0);
    let pinsa = PinsA::new(&mut dp.sysconfig, None, dp.porta);
    let spi_cfg = SpiConfig::default();
    let (sck, mosi, miso) = (
        pinsa.pa20.into_funsel_2(),
        pinsa.pa19.into_funsel_2(),
        pinsa.pa18.into_funsel_2(),
    );
    let cs_pin = pinsa.pa16.into_funsel_2();

    // Need to set the ADC chip select low
    let mut adc_cs = pinsa.pa17.into_push_pull_output();
    adc_cs
        .set_high()
        .expect("Setting ADC chip select high failed");

    let transfer_cfg = TransferConfig::new(
        1.MHz(),
        embedded_hal::spi::MODE_3,
        Some(cs_pin),
        false,
        true,
    );
    let mut spi = Spi::spib(
        dp.spib,
        (sck, miso, mosi),
        50.MHz(),
        spi_cfg,
        Some(&mut dp.sysconfig),
        Some(&transfer_cfg.downgrade()),
    );

    let mut tx_rx_buf: [u8; 3] = [0; 3];
    tx_rx_buf[0] = READ_MASK | DEVID_REG;
    spi.transfer_in_place(&mut tx_rx_buf[0..2])
        .expect("Reading DEVID register failed");
    rprintln!("DEVID register: {}", tx_rx_buf[1]);

    tx_rx_buf[0] = POWER_CTL_REG;
    tx_rx_buf[1] = PWR_MEASUREMENT_MODE_MASK;
    spi.write(&tx_rx_buf[0..2])
        .expect("Enabling measurement mode failed");

    loop {
        delay.delay_ms(500);
    }
}
