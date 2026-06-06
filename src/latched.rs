//! Timer-triggered DMA driver for HUB75 latched panels.
//!
//! Uses an embassy-stm32 [`Timer`] for the pixel clock (PWM output on CH1)
//! and DMA trigger, with a [`ChannelAndRequest`] performing byte-width
//! transfers from the framebuffer to the target byte of the GPIO port's
//! ODR register (either pins 0-7 or pins 8-15).
//!
//! The timer update event triggers one DMA byte transfer per tick. The
//! PWM output provides the panel CLK signal. BCM weighting is achieved
//! by scaling the timer period for each bit plane.

use embassy_stm32::dma::{self, Channel, TransferOptions};
use embassy_stm32::gpio::{AnyPin, OutputType, Pin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, RoundTo, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

use crate::framebuffer::FrameBuffer;
use crate::Hub75Pins8;

/// HUB75 LED matrix driver using timer-triggered DMA to GPIO.
///
/// Generic over `T`, the timer peripheral (must implement
/// [`GeneralInstance4Channel`]). The DMA channel is passed at construction
/// time and must implement [`UpDma<T>`] so the DMAMUX can route timer
/// update events to it.
///
/// # Hardware Requirements
///
/// - Data pins (R1, G1, B1, R2, G2, B2, LATCH, BLANK) must be on the same
///   GPIO port, occupying either pins 0-7 or pins 8-15, in the correct order.
/// - Clock pin must be a valid timer channel 1 output for timer `T`.
/// - The timer and DMA channel are consumed by this driver.
pub struct Hub75<'d, T: GeneralInstance4Channel> {
    timer: Timer<'d, T>,
    dma: Channel<'d>,
    dma_request: dma::Request,
    _data_pins: [AnyPin; 8],
    _clock_pin: PwmPin<'d, T, Ch1>,
    odr_byte_addr: *mut u8,
}

impl<'d, T: GeneralInstance4Channel> Hub75<'d, T> {
    /// Creates a new HUB75 driver.
    ///
    /// Configures all data pins as high-speed push-pull outputs, sets up
    /// the timer for PWM clock generation and DMA triggering, and prepares
    /// the DMA channel for byte-width transfers to GPIO.
    ///
    /// The data pins are validated: they must all be on the same GPIO port,
    /// all within the same byte half (0-7 or 8-15), and wired so that R1
    /// maps to bit 0 of the byte, G1 to bit 1, etc.
    ///
    /// # Arguments
    /// * `tim` - Timer peripheral (e.g. `p.TIM2`)
    /// * `clock_pin` - GPIO pin for the pixel clock; must be a valid TIM channel 1
    ///   output for timer `T` (enforced at compile time via [`TimerPin`])
    /// * `dma_ch` - DMA channel peripheral (e.g. `p.DMA1_CH1`)
    /// * `dma_irq` - Interrupt binding for the DMA channel
    /// * `pins` - HUB75 data pin configuration
    /// * `frequency` - Pixel clock frequency (e.g. `Hertz(10_000_000)`)
    ///
    /// # Panics
    /// Panics if the computed timer period is zero.
    pub fn new<D: UpDma<T>>(
        tim: Peri<'d, T>,
        clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma_ch: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pins: Hub75Pins8,
        frequency: Hertz,
    ) -> Self {
        let gpio = pins.pins[0].block();

        for pin in &pins.pins {
            let n = pin.pin() as usize;
            gpio.moder()
                .modify(|w| w.set_moder(n, pac::gpio::vals::Moder::OUTPUT));
            gpio.ospeedr()
                .modify(|w| w.set_ospeedr(n, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));
            gpio.otyper()
                .modify(|w| w.set_ot(n, pac::gpio::vals::Ot::PUSH_PULL));
        }

        // BLANK starts HIGH (display off during init)
        gpio.bsrr()
            .write(|w| w.set_bs(pins.blank_pin_num(), true));

        // ODR byte address: base ODR + 0 for lower pins, + 1 for upper pins
        let byte_offset: usize = if pins.base_pin == 0 { 0 } else { 1 };
        let odr_byte_addr = unsafe { (gpio.odr().as_ptr() as *mut u8).add(byte_offset) };

        let clock_pin = PwmPin::new(clock_pin, OutputType::PushPull);

        // Configure timer for PWM clock output on CH1
        let timer = Timer::new(tim);
        timer.set_counting_mode(CountingMode::EdgeAlignedUp);
        timer.set_frequency(frequency, RoundTo::Slower);
        timer.enable_outputs();

        // PWM mode 2: output LOW when CNT < CCR, HIGH when CNT >= CCR.
        // DMA writes data at the update event (CNT=0 wrap), then CLK rises
        // at CCR, ensuring data is stable before the panel latches on the
        // rising edge.
        timer.set_output_compare_mode(TimChannel::Ch1, OutputCompareMode::PwmMode2);
        timer.set_output_compare_preload(TimChannel::Ch1, true);
        timer.set_autoreload_preload(true);

        let max: u32 = timer.get_max_compare_value().into();
        timer.set_compare_value(TimChannel::Ch1, max.div_ceil(2).try_into().ok().unwrap());

        timer.enable_channel(TimChannel::Ch1, true);
        timer.generate_update_event();

        // Build DMA channel + request (DMAMUX routing for timer update event)
        let dma_request = dma_ch.request();
        let dma = Channel::new(dma_ch, dma_irq);

        Self {
            timer,
            dma,
            dma_request,
            _data_pins: pins.pins,
            _clock_pin: clock_pin,
            odr_byte_addr,
        }
    }

    /// Render a full frame from the framebuffer to the display.
    ///
    /// BCM weighting is achieved by repeating each plane's DMA transfer
    /// proportionally: plane 0 (MSB) is output 2^(N-1) times, while
    /// plane N-1 (LSB) is output once. The timer frequency stays constant.
    pub async fn render(&mut self, fb: &impl FrameBuffer) {
        let plane_count = fb.plane_count();

        for plane_idx in 0..plane_count {
            let (ptr, len) = fb.plane_ptr_len(plane_idx);
            let buf = unsafe { core::slice::from_raw_parts(ptr, len) };

            // plane 0 = MSB → repeat 2^(N-1) times; plane N-1 = LSB → repeat 1 time
            let repeats = 1u32 << (plane_count - 1 - plane_idx);

            self.timer.enable_update_dma(true);
            self.timer.start();
            for _ in 0..repeats {
                unsafe {
                    self.dma
                        .write(
                            self.dma_request,
                            buf,
                            self.odr_byte_addr,
                            TransferOptions::default(),
                        )
                        .await;
                }
            }
            self.timer.stop();
            self.timer.enable_update_dma(false);
        }
    }
}
