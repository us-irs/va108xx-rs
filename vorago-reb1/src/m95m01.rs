//! Basic driver for the ST M95M01 EEPROM memory.
//!
//! This driver is used by the provided bootloader application for the REB1
//! board. It provides a convenient wrapper around the HAL SPI to interface
//! with the EEPROM memory of the REB1 board.
//!
//! # Example
//!
//! - [REB1 EEPROM example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/nvm.rs)
use core::convert::Infallible;
use embedded_hal::spi::SpiBus;

bitfield::bitfield! {
    pub struct StatusReg(u8);
    impl Debug;
    u8;
    pub status_register_write_protect, _: 7;
    pub zero_segment, _: 6, 4;
    pub block_protection_bits, set_block_protection_bits: 3, 2;
    pub write_enable_latch, _: 1;
    pub write_in_progress, _: 0;
}

// Registers.
pub mod regs {
    /// Write status register command.
    pub const WRSR: u8 = 0x01;
    // Write command.
    pub const WRITE: u8 = 0x02;
    // Read command.
    pub const READ: u8 = 0x03;
    /// Write disable command.
    pub const WRDI: u8 = 0x04;
    /// Read status register command.
    pub const RDSR: u8 = 0x05;
    /// Write enable command.
    pub const WREN: u8 = 0x06;
}

use regs::*;
use va108xx_hal::{
    pac,
    prelude::*,
    spi::{RomMiso, RomMosi, RomSck, Spi, SpiConfig, BMSTART_BMSTOP_MASK},
};

pub type RomSpi = Spi<pac::Spic, (RomSck, RomMiso, RomMosi), u8>;

/// Driver for the ST device M95M01 EEPROM memory.
///
/// Specialized for the requirements of the VA108XX MCUs.
pub struct M95M01 {
    pub spi: RomSpi,
}

impl M95M01 {
    pub fn new(syscfg: &mut pac::Sysconfig, sys_clk: impl Into<Hertz>, spi: pac::Spic) -> Self {
        let spi = RomSpi::new(
            syscfg,
            sys_clk,
            spi,
            (RomSck, RomMiso, RomMosi),
            SpiConfig::default(),
        );
        let mut spi_dev = Self { spi };
        spi_dev.clear_block_protection().unwrap();
        spi_dev
    }

    pub fn release(mut self) -> pac::Spic {
        self.set_block_protection().unwrap();
        self.spi.release().0
    }

    // Wait until the write-in-progress state is cleared. This exposes a [nb] API, so this function
    // will return [nb::Error::WouldBlock] if the EEPROM is still busy.
    pub fn writes_are_done(&mut self) -> nb::Result<(), Infallible> {
        let rdsr = self.read_status_reg()?;
        if rdsr.write_in_progress() {
            return Err(nb::Error::WouldBlock);
        }
        Ok(())
    }

    pub fn read_status_reg(&mut self) -> Result<StatusReg, Infallible> {
        let mut write_read: [u8; 2] = [regs::RDSR, 0x00];
        self.spi.transfer_in_place(&mut write_read)?;
        Ok(StatusReg(write_read[1]))
    }

    pub fn write_enable(&mut self) -> Result<(), Infallible> {
        self.spi.write(&[regs::WREN])
    }

    pub fn clear_block_protection(&mut self) -> Result<(), Infallible> {
        // Has to be written separately.
        self.write_enable()?;
        self.spi.write(&[WRSR, 0x00])
    }

    pub fn set_block_protection(&mut self) -> Result<(), Infallible> {
        let mut reg = StatusReg(0);
        reg.set_block_protection_bits(0b11);
        self.write_enable()?;
        self.spi.write(&[WRSR, reg.0])
    }

    fn common_init_write_and_read(&mut self, address: u32, reg: u8) -> Result<(), Infallible> {
        nb::block!(self.writes_are_done())?;
        self.spi.flush()?;
        if reg == WRITE {
            self.write_enable()?;
            self.spi.write_fifo_unchecked(WRITE as u32);
        } else {
            self.spi.write_fifo_unchecked(READ as u32);
        }
        self.spi.write_fifo_unchecked((address >> 16) & 0xff);
        self.spi.write_fifo_unchecked((address >> 8) & 0xff);
        self.spi.write_fifo_unchecked(address & 0xff);
        Ok(())
    }

    fn common_read(&mut self, address: u32) -> Result<(), Infallible> {
        self.common_init_write_and_read(address, READ)?;
        for _ in 0..4 {
            // Pump the FIFO.
            self.spi.write_fifo_unchecked(0);
            // Ignore the first 4 bytes.
            self.spi.read_fifo_unchecked();
        }
        Ok(())
    }

    pub fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Infallible> {
        self.common_init_write_and_read(address, WRITE)?;
        for val in data.iter().take(data.len() - 1) {
            nb::block!(self.spi.write_fifo(*val as u32))?;
            self.spi.read_fifo_unchecked();
        }
        nb::block!(self
            .spi
            .write_fifo(*data.last().unwrap() as u32 | BMSTART_BMSTOP_MASK))?;
        self.spi.flush()?;
        nb::block!(self.writes_are_done())?;
        Ok(())
    }

    pub fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Infallible> {
        self.common_read(address)?;
        for val in buf.iter_mut() {
            nb::block!(self.spi.write_fifo(0))?;
            *val = (nb::block!(self.spi.read_fifo()).unwrap() & 0xff) as u8;
        }
        nb::block!(self.spi.write_fifo(BMSTART_BMSTOP_MASK))?;
        self.spi.flush()?;
        Ok(())
    }

    pub fn verify(&mut self, address: u32, data: &[u8]) -> Result<bool, Infallible> {
        self.common_read(address)?;
        for val in data.iter() {
            nb::block!(self.spi.write_fifo(0))?;
            let read_val = (nb::block!(self.spi.read_fifo()).unwrap() & 0xff) as u8;
            if read_val != *val {
                return Ok(false);
            }
        }
        nb::block!(self.spi.write_fifo(BMSTART_BMSTOP_MASK))?;
        self.spi.flush()?;
        Ok(true)
    }
}
