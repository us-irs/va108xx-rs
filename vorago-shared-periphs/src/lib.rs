#![no_std]
pub mod gpio;
pub mod ioconfig;

#[cfg(not(feature = "_family-selected"))]
compile_error!("no Vorago CPU family was select. Choices: vor1x or vor4x");

pub use ioconfig::regs::FunSel;

cfg_if::cfg_if! {
    if #[cfg(feature = "vor1x")] {
        /// Number of GPIO ports and IOCONFIG registers for PORT A
        pub const NUM_PORT_A: usize = 32;
        /// Number of GPIO ports and IOCONFIG registers for PORT B
        pub const NUM_PORT_B: usize = 24;
    } else if #[cfg(feature = "vor4x")] {
        /// Number of GPIO ports and IOCONFIG registers for PORT C to Port F
        pub const NUM_PORT_DEFAULT: usize = 16;
        /// Number of GPIO ports and IOCONFIG registers for PORT A
        pub const NUM_PORT_A: usize = NUM_PORT_DEFAULT;
        /// Number of GPIO ports and IOCONFIG registers for PORT B
        pub const NUM_PORT_B: usize = NUM_PORT_DEFAULT;
        /// Number of GPIO ports and IOCONFIG registers for PORT G
        pub const NUM_PORT_G: usize = 8;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Port {
    A = 0,
    B = 1,
    #[cfg(feature = "vor4x")]
    C = 2,
    #[cfg(feature = "vor4x")]
    D = 3,
    #[cfg(feature = "vor4x")]
    E = 4,
    #[cfg(feature = "vor4x")]
    F = 5,
    #[cfg(feature = "vor4x")]
    G = 6,
}

impl Port {
    pub fn max_offset(&self) -> usize {
        match self {
            Port::A => NUM_PORT_A,
            Port::B => NUM_PORT_B,
            #[cfg(feature = "vor4x")]
            Port::C | Port::D | Port::E | Port::F => NUM_PORT_DEFAULT,
            #[cfg(feature = "vor4x")]
            Port::G => NUM_PORT_G,
        }
    }
}

#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[error("invalid GPIO offset {offset} for port {port:?}")]
pub struct InvalidOffsetError {
    offset: usize,
    port: Port,
}

#[allow(dead_code)]
pub(crate) mod sealed {
    pub trait Sealed {}
}
