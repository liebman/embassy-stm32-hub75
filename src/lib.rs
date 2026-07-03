//! # embassy-stm32-hub75
//!
//! A `no-std` Rust driver for HUB75-style LED matrix panels on STM32
//! microcontrollers. HUB75 is a standard interface for driving large, bright,
//! and colorful RGB LED displays, commonly used in digital signage and art
//! installations.
//!
//! This library uses an ISR-driven DMA refresh loop to continuously output
//! framebuffer data to a GPIO port. A timer generates the pixel clock via PWM
//! and triggers DMA transfers on each update event. Once the driver is
//! initialised via `hub75_define!`'s `init()` function, rendering happens
//! entirely in the background via DMA transfer-complete interrupts — no CPU
//! involvement per pixel.
//!
//! ## Double Buffering
//!
//! The driver supports double-buffered operation via `Hub75::swap()`. The
//! application writes to one framebuffer while the ISR renders from another,
//! swapping atomically at frame boundaries.
//!
//! ## 8-bit Mode
//!
//! In 8-bit mode, an external 74HC574-style latch handles the row address
//! lines. This requires only 8 data pins: R1, G1, B1, R2, G2, B2, LATCH,
//! and BLANK. The 8 pins must occupy either the lower byte (pins 0-7) or
//! upper byte (pins 8-15) of a single GPIO port. Byte-width DMA writes to
//! the corresponding ODR byte update only those 8 pins without disturbing
//! the other half of the port.
//!
//! ## 16-bit Mode
//!
//! In 16-bit mode, all 16 pins of a GPIO port are used. Half-word-width DMA
//! writes the full ODR register on each clock cycle. Pin layout and bit
//! assignments depend on the framebuffer implementation used.
//!
//! ## Framebuffers
//!
//! The `hub75-framebuffer` crate provides bitplane framebuffers that are
//! strongly recommended for their memory efficiency. The bitplane latched
//! variant (`framebuffer::bitplane::latched::DmaFrameBuffer`) stores one bit
//! per pixel per plane, and the driver outputs the data via DMA without any
//! format conversion.
//!
//! ## Defining an Instance
//!
//! Use the `hub75_define!` macro to create a driver module bound to specific
//! timer and DMA channel peripherals:
//!
//! ```ignore
//! use embassy_stm32::{bind_interrupts, dma, peripherals};
//! use embassy_stm32_hub75::hub75_define;
//!
//! hub75_define!(hub75, embassy_stm32::peripherals::TIM2, embassy_stm32::peripherals::DMA1_CH1);
//!
//! bind_interrupts!(struct Irqs {
//!     DMA1_CHANNEL1 =>
//!         dma::InterruptHandler<peripherals::DMA1_CH1>,
//!         hub75::Hub75DmaHandler;
//! });
//!
//! let hub75 = hub75::init(
//!     p.TIM2, p.PA0, p.DMA1_CH1, Irqs, pins,
//!     Config::new().frequency(Hertz(6_000_000)),
//!     fb,
//! );
//! ```
//!
//! ## Crate Features
//!
//! - `stm32wl55`: Enable support for the STM32WL55
//! - `defmt`: Enable logging with `defmt`

#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

use embassy_stm32::dma::word::WordSize;
use embassy_stm32::Peri;
pub use hub75_framebuffer as framebuffer;

/// The color type used by the HUB75 driver.
pub use hub75_framebuffer::Color;

#[doc(hidden)]
pub mod bcm;
pub mod dma;

/// Re-exports used by the [`hub75_define!`] macro. Not part of the public API.
#[doc(hidden)]
pub mod __macro_support {
    pub use critical_section;
    pub use embassy_stm32;
}

use embassy_stm32::gpio::{AnyPin, Flex, Level, Pin};

// re-export items from embassy-stm32 that are used in the public API
pub use embassy_stm32::gpio::Speed;
pub use embassy_stm32::time::Hertz;

/// Driver configuration for pixel clock frequency (defaults to 10 MHz) and GPIO output speed (defaults to Medium).
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Pixel clock frequency for the HUB75 panel.
    pub frequency: Hertz,
    /// GPIO output speed for the data and control pins.
    pub gpio_speed: Speed,
}

impl Config {
    /// Sensible starting defaults: 10 MHz pixel clock, medium GPIO speed.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            frequency: Hertz(10_000_000),
            gpio_speed: Speed::Medium,
        }
    }

    /// Set the pixel clock frequency.
    #[must_use]
    pub const fn frequency(mut self, frequency: Hertz) -> Self {
        self.frequency = frequency;
        self
    }

    /// Set the GPIO output speed for the data and control pins.
    #[must_use]
    pub const fn gpio_speed(mut self, gpio_speed: Speed) -> Self {
        self.gpio_speed = gpio_speed;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}

/// Trait implemented by HUB75 pin groups (8-bit or 16-bit).
pub trait Hub75Pins {
    /// The word type for the GPIO port (`u8` for 8-bit ports, `u16` for 16-bit ports).
    type Word;
    /// DMA word size matching [`Self::Word`].
    const DMA_WORD_SIZE: WordSize;
    /// Configure the pins and get the ODR pointer.
    fn configure_and_get_odr(self, speed: Speed) -> *mut u8;
}

/// Pin configuration for a HUB75 panel with an external address latch.
///
/// The 8 data pins must all be on the same GPIO port, occupying either the
/// lower byte (pins 0-7) or upper byte (pins 8-15). The pins must be wired
/// in order so that R1 maps to bit 0 of the byte, G1 to bit 1, and so on.
///
/// The data pins map directly to the `hub75-framebuffer` latched byte layout:
/// - bit 0: R1
/// - bit 1: G1
/// - bit 2: B1
/// - bit 3: R2
/// - bit 4: G2
/// - bit 5: B2
/// - bit 6: LATCH
/// - bit 7: BLANK
///
/// For upper-byte wiring, pin N+8 corresponds to bit N.
/// For lower-byte wiring, pin N corresponds to bit N.
///
/// The clock pin is passed separately to the `init()` constructor as a raw
/// GPIO pin. It must be a valid timer channel 1 output for the chosen timer
/// (enforced at compile time). The driver configures it as a PWM output
/// internally.
///
/// Use [`Hub75Pins8::new()`] to construct; it validates pin layout at
/// creation time.
pub struct Hub75Pins8 {
    /// The 8 GPIO pins in order.
    pub pins: [AnyPin; 8],
    /// First pin number in the group (0 or 8).
    pub base_pin: u8,
}

impl Hub75Pins8 {
    /// Create a validated pin configuration.
    ///
    /// All 8 pins must be on the same GPIO port and must occupy 8
    /// consecutive pins in either the lower byte (pins 0-7) or upper byte
    /// (pins 8-15), with R1 on the lowest pin of the group.
    ///
    /// # Errors
    /// Returns [`Hub75Error::PinNotOnSamePort`] if any pin is on a
    /// different GPIO port than the first, or
    /// [`Hub75Error::PinsNotConsecutive`] if the pins are not 8
    /// consecutive pins starting at pin 0 or pin 8.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        red1: AnyPin,
        grn1: AnyPin,
        blu1: AnyPin,
        red2: AnyPin,
        grn2: AnyPin,
        blu2: AnyPin,
        latch: AnyPin,
        blank: AnyPin,
    ) -> Result<Self, Hub75Error> {
        let pins_ref: [&AnyPin; 8] = [&red1, &grn1, &blu1, &red2, &grn2, &blu2, &latch, &blank];

        let port = pins_ref[0].port();
        let first = pins_ref[0].pin();

        if first != 0 && first != 8 {
            return Err(Hub75Error::PinsNotConsecutive {
                index: 0,
                expected: 0,
                actual: first,
            });
        }

        for (i, pin) in pins_ref.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let i = i as u8;
            if pin.port() != port {
                return Err(Hub75Error::PinNotOnSamePort { index: i });
            }
            let expected = first + i;
            if pin.pin() != expected {
                return Err(Hub75Error::PinsNotConsecutive {
                    index: i,
                    expected,
                    actual: pin.pin(),
                });
            }
        }

        Ok(Self {
            pins: [red1, grn1, blu1, red2, grn2, blu2, latch, blank],
            base_pin: first,
        })
    }
}

impl Hub75Pins for Hub75Pins8 {
    type Word = u8;
    const DMA_WORD_SIZE: WordSize = WordSize::OneByte;
    fn configure_and_get_odr(self, speed: Speed) -> *mut u8 {
        let gpio = self.pins[0].block();
        let byte_offset = usize::from(self.base_pin != 0);
        let odr_byte_addr = unsafe { (gpio.odr().as_ptr().cast::<u8>()).add(byte_offset) };

        for (i, pin) in self.pins.into_iter().enumerate() {
            // SAFETY: we own the AnyPin and will leak the Flex to keep it alive.
            let peri = unsafe { Peri::new_unchecked(pin) };
            let mut flex = Flex::new(peri);
            if i == 7 {
                flex.set_level(Level::High);
            }
            flex.set_as_output(speed);
            core::mem::forget(flex);
        }
        odr_byte_addr
    }
}

/// Pin configuration for a HUB75 panel using 16 data pins (full GPIO port width).
///
/// All 16 pins must be on the same GPIO port, occupying pins 0-15 in order.
/// The DMA writes a full `u16` to the ODR register on each clock cycle.
///
/// The data pins map directly to the `hub75-framebuffer` 16-bit layout.
/// Specific bit assignments depend on the framebuffer implementation used.
///
/// The clock pin is passed separately to the `init()` constructor as a raw
/// GPIO pin. It must be a valid timer channel 1 output for the chosen timer
/// (enforced at compile time). The driver configures it as a PWM output
/// internally.
///
/// Use [`Hub75Pins16::new()`] to construct; it validates pin layout at
/// creation time.
pub struct Hub75Pins16 {
    /// The 16 GPIO pins in order (pin 0 through pin 15).
    pub pins: [AnyPin; 16],
}

impl Hub75Pins16 {
    /// Create a validated 16-pin configuration.
    ///
    /// All 16 pins must be on the same GPIO port and must occupy pins 0-15
    /// consecutively.
    ///
    /// # Errors
    /// Returns [`Hub75Error::PinNotOnSamePort`] if any pin is on a
    /// different GPIO port than the first, or
    /// [`Hub75Error::PinsNotConsecutive`] if the pins are not
    /// consecutive starting at pin 0.
    #[allow(clippy::too_many_arguments)]
    pub fn new(pins: [AnyPin; 16]) -> Result<Self, Hub75Error> {
        let port = pins[0].port();
        let first = pins[0].pin();

        if first != 0 {
            return Err(Hub75Error::PinsNotConsecutive {
                index: 0,
                expected: 0,
                actual: first,
            });
        }

        for (i, pin) in pins.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            let i = i as u8;
            if pin.port() != port {
                return Err(Hub75Error::PinNotOnSamePort { index: i });
            }
            let expected = i;
            if pin.pin() != expected {
                return Err(Hub75Error::PinsNotConsecutive {
                    index: i,
                    expected,
                    actual: pin.pin(),
                });
            }
        }

        Ok(Self { pins })
    }
}

impl Hub75Pins for Hub75Pins16 {
    type Word = u16;
    const DMA_WORD_SIZE: WordSize = WordSize::TwoBytes;
    fn configure_and_get_odr(self, speed: Speed) -> *mut u8 {
        let gpio = self.pins[0].block();
        let odr_addr = gpio.odr().as_ptr().cast::<u8>();

        for pin in self.pins {
            let peri = unsafe { Peri::new_unchecked(pin) };
            let mut flex = Flex::new(peri);
            flex.set_as_output(speed);
            core::mem::forget(flex);
        }
        odr_addr
    }
}

/// Represents errors that can occur during HUB75 driver operations.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hub75Error {
    /// A pin is on a different GPIO port than the others.
    /// `index` is the position in the pin list.
    PinNotOnSamePort {
        /// Index of the offending pin.
        index: u8,
    },
    /// The pins are not consecutive starting at the expected position.
    PinsNotConsecutive {
        /// Index of the offending pin.
        index: u8,
        /// Expected pin number.
        expected: u8,
        /// Actual pin number.
        actual: u8,
    },
    /// Error occurred during DMA operations.
    Dma,
    /// Error occurred during timer configuration.
    Timer,
    /// The driver has not been initialised yet.
    NotInitialised,
}
