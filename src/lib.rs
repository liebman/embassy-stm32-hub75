//! # embassy-stm32-hub75
//!
//! A `no-std` Rust driver for HUB75-style LED matrix panels on STM32
//! microcontrollers. HUB75 is a standard interface for driving large, bright,
//! and colorful RGB LED displays, commonly used in digital signage and art
//! installations.
//!
//! This library uses an ISR-driven DMA refresh loop to continuously output
//! framebuffer data to a GPIO port. A timer generates the pixel clock via PWM
//! and triggers DMA byte-transfers on each update event. Once
//! [`Hub75::start()`] is called, rendering happens entirely in the background
//! via DMA transfer-complete interrupts — no CPU involvement per pixel.
//!
//! ## Double Buffering
//!
//! The driver supports double-buffered operation via [`Hub75::swap()`]. The
//! application writes to one framebuffer while the ISR renders from another,
//! swapping atomically at frame boundaries.
//!
//! ## Latched (8-bit) Mode
//!
//! This driver supports the **latched** HUB75 configuration, where an
//! external 74HC574-style latch handles the row address lines. This requires
//! only 8 data pins: R1, G1, B1, R2, G2, B2, LATCH, and BLANK. The 8 pins
//! must occupy either the lower byte (pins 0-7) or upper byte (pins 8-15) of
//! a single GPIO port, and the pins must be in the correct order (R1 on bit 0,
//! G1 on bit 1, etc.). Byte-width DMA writes to the corresponding ODR byte
//! update only those 8 pins without disturbing the other half of the port.
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
//! Use the [`hub75_define!`] macro to create a driver module bound to specific
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
//! ```
//!
//! ## Crate Features
//!
//! - `stm32wl55`: Enable support for the STM32WL55
//! - `defmt`: Enable logging with `defmt`

#![no_std]
#![warn(missing_docs)]

pub use hub75_framebuffer as framebuffer;

/// The color type used by the HUB75 driver.
pub use hub75_framebuffer::Color;

#[doc(hidden)]
pub mod bcm;
mod latched;

/// Re-exports used by the [`hub75_define!`] macro. Not part of the public API.
#[doc(hidden)]
pub mod __macro_support {
    pub use critical_section;
    pub use embassy_stm32;
}

/// Typestate marker for an idle [`Hub75`] that has not yet been started.
///
/// Implements [`FrameBuffer`](framebuffer::FrameBuffer) trivially so it can
/// serve as the default type parameter; never instantiated at runtime.
pub struct Idle;

impl framebuffer::FrameBuffer for Idle {
    fn get_word_size(&self) -> framebuffer::WordSize {
        framebuffer::WordSize::Eight
    }
    fn plane_count(&self) -> usize {
        0
    }
    fn plane_ptr_len(&self, _: usize) -> (*const u8, usize) {
        (core::ptr::null(), 0)
    }
}

use embassy_stm32::gpio::{AnyPin, Pin};

pub use embassy_stm32::time::Hertz;

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
/// The clock pin is passed separately to the [`Hub75`](hub75_define) constructor
/// as a raw GPIO pin. It must be a valid timer channel 1 output for the chosen
/// timer (enforced at compile time). The driver configures it as a PWM
/// output internally.
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
        let pins_ref: [&AnyPin; 8] =
            [&red1, &grn1, &blu1, &red2, &grn2, &blu2, &latch, &blank];

        let port = pins_ref[0].port();
        let first = pins_ref[0].pin();

        // R1 must start at pin 0 or pin 8
        if first != 0 && first != 8 {
            return Err(Hub75Error::PinsNotConsecutive {
                index: 0,
                expected: 0,
                actual: first,
            });
        }

        for (i, pin) in pins_ref.iter().enumerate() {
            if pin.port() != port {
                return Err(Hub75Error::PinNotOnSamePort { index: i as u8 });
            }
            let expected = first + i as u8;
            if pin.pin() != expected {
                return Err(Hub75Error::PinsNotConsecutive {
                    index: i as u8,
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

    /// The BLANK pin (index 7 in the group).
    pub fn blank_pin_num(&self) -> usize {
        (self.base_pin + 7) as usize
    }
}

/// Represents errors that can occur during HUB75 driver operations.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Hub75Error {
    /// A pin is on a different GPIO port than the others.
    /// `index` is the position in the pin list (0=R1 .. 7=BLANK).
    PinNotOnSamePort {
        /// Index of the offending pin (0=R1, 1=G1, ..., 7=BLANK).
        index: u8,
    },
    /// The pins are not 8 consecutive pins starting at 0 or 8.
    PinsNotConsecutive {
        /// Index of the offending pin (0=R1, 1=G1, ..., 7=BLANK).
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
}
