//! # API for using the REB1 LEDs
//!
//! ## Examples
//!
//! - [LED example](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-leds.rs)
//! - [Button Blinky using IRQs](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-irq.rs)
//! - [Button Blinky using IRQs and RTIC](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1/examples/blinky-button-rtic.rs)
use embedded_hal::digital::OutputPin;
use va108xx_hal::{
    gpio::dynpin::DynPin,
    gpio::pin::{Pin, PushPullOutput, PA10, PA6, PA7},
};

pub type LD2 = Pin<PA10, PushPullOutput>;
pub type LD3 = Pin<PA7, PushPullOutput>;
pub type LD4 = Pin<PA6, PushPullOutput>;

#[derive(Debug)]
pub struct Leds(pub [Led; 3]);

impl Leds {
    pub fn new(led_pin1: LD2, led_pin2: LD3, led_pin3: LD4) -> Leds {
        Leds([led_pin1.into(), led_pin2.into(), led_pin3.into()])
    }
}

impl core::ops::Deref for Leds {
    type Target = [Led];

    fn deref(&self) -> &[Led] {
        &self.0
    }
}

impl core::ops::DerefMut for Leds {
    fn deref_mut(&mut self) -> &mut [Led] {
        &mut self.0
    }
}

impl core::ops::Index<usize> for Leds {
    type Output = Led;

    fn index(&self, i: usize) -> &Led {
        &self.0[i]
    }
}

impl core::ops::IndexMut<usize> for Leds {
    fn index_mut(&mut self, i: usize) -> &mut Led {
        &mut self.0[i]
    }
}

#[derive(Debug)]
pub struct Led(pub DynPin);

macro_rules! ctor {
	($($ldx:ident),+) => {
		$(
			impl From<$ldx> for Led {
				fn from(led: $ldx) -> Self {
					Led(led.into())
				}
			}
		)+
	}
}

ctor!(LD2, LD3, LD4);

impl Led {
    /// Turns the LED off. Setting the pin high actually turns the LED off
    #[inline]
    pub fn off(&mut self) {
        self.0.set_high().ok();
    }

    /// Turns the LED on. Setting the pin low actually turns the LED on
    #[inline]
    pub fn on(&mut self) {
        self.0.set_low().ok();
    }

    /// Toggles the LED
    #[inline]
    pub fn toggle(&mut self) {
        self.0.toggle_with_toggle_reg().ok();
    }
}
