//! Example application which interfaces with the boot EEPROM.
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_hal::{pac, pwm::CountDownTimer, time::Hertz};
use vorago_reb1::m95m01::M95M01;

const CLOCK_FREQ: Hertz = Hertz::from_raw(50_000_000);

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("-- VA108XX REB1 NVM example --");

    let mut dp = pac::Peripherals::take().unwrap();

    let mut timer = CountDownTimer::new(&mut dp.sysconfig, CLOCK_FREQ, dp.tim0);
    let mut nvm = M95M01::new(&mut dp.sysconfig, CLOCK_FREQ, dp.spic);
    let status_reg = nvm.read_status_reg().expect("reading status reg failed");
    if status_reg.zero_segment() == 0b111 {
        panic!("status register unexpected values");
    }

    let mut orig_content: [u8; 16] = [0; 16];
    let mut read_buf: [u8; 16] = [0; 16];
    let write_buf: [u8; 16] = [0; 16];
    for (idx, val) in read_buf.iter_mut().enumerate() {
        *val = idx as u8;
    }
    nvm.read(0x4000, &mut orig_content).unwrap();

    // One byte write and read.
    nvm.write(0x4000, &write_buf[0..1]).unwrap();
    nvm.read(0x4000, &mut read_buf[0..1]).unwrap();
    assert_eq!(write_buf[0], read_buf[0]);
    read_buf.fill(0);

    // Four bytes write and read.
    nvm.write(0x4000, &write_buf[0..4]).unwrap();
    nvm.read(0x4000, &mut read_buf[0..4]).unwrap();
    assert_eq!(&read_buf[0..4], &write_buf[0..4]);
    read_buf.fill(0);

    // Full sixteen bytes
    nvm.write(0x4000, &write_buf).unwrap();
    nvm.read(0x4000, &mut read_buf).unwrap();
    assert_eq!(&read_buf, &write_buf);
    read_buf.fill(0);

    // 3 bytes
    nvm.write(0x4000, &write_buf[0..3]).unwrap();
    nvm.read(0x4000, &mut read_buf[0..3]).unwrap();
    assert_eq!(&read_buf[0..3], &write_buf[0..3]);

    // Write back original content.
    nvm.write(0x4000, &orig_content).unwrap();
    loop {
        timer.delay_ms(500);
    }
}
