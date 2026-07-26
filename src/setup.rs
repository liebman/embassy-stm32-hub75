//! Shared hardware setup for all HUB75 backends.
//!
//! Every backend (`dma`, `gpdma`, `gpdma_2d`) configures the timer
//! peripheral, clock PWM pin, and GPIO data pins identically. This
//! module factors that setup out so there is a single source of truth.

use core::mem::ManuallyDrop;

use critical_section::Mutex;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, RoundTo, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin};
use embassy_stm32::Peri;

use crate::{Config, Hub75Pins};

/// Type alias for the timer static slot used by all HUB75 macros.
///
/// Re-exported by `dma.rs` and `gpdma.rs` so the macros can import it
/// as `dma::TimerSlot` / `gpdma::TimerSlot` without changes.
pub type TimerSlot<T> = Mutex<core::cell::RefCell<Option<ManuallyDrop<Timer<'static, T>>>>>;

/// Configured hardware resources needed by every HUB75 backend.
///
/// Returned by [`hardware`]. Contains the three values produced by
/// timer/GPIO/PWM setup.
pub(crate) struct Hardware<'d, T: GeneralInstance4Channel> {
    /// The configured PWM clock pin, stored in the driver struct.
    pub clock_pin: PwmPin<'d, T, Ch1>,
    /// GPIO ODR register address for DMA writes.
    pub odr_addr: *mut u8,
    /// The configured timer, consumed by [`store_timer`].
    pub timer: Timer<'d, T>,
}

/// Configure GPIO pins, clock pin, and timer for HUB75 PWM output.
///
/// Identical setup shared by `dma::Hub75::new()`, `gpdma::Hub75Gpdma::new()`,
/// and `gpdma_2d::Hub75Gpdma2d::new()`.
pub(crate) fn hardware<'d, T: GeneralInstance4Channel, P: Hub75Pins>(
    tim: Peri<'d, T>,
    clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
    pins: P,
    config: &Config,
) -> Hardware<'d, T> {
    let odr_addr = pins.configure_and_get_odr(config.gpio_speed).as_ptr();
    let clock_pin = PwmPin::new(clock_pin, OutputType::PushPull);

    let timer = Timer::new(tim);
    timer.set_counting_mode(CountingMode::EdgeAlignedUp);
    timer.set_frequency(config.frequency, RoundTo::Slower);
    timer.enable_outputs();

    timer.set_output_compare_mode(TimChannel::Ch1, OutputCompareMode::PwmMode2);
    timer.set_output_compare_preload(TimChannel::Ch1, true);
    timer.set_autoreload_preload(true);

    let max: u32 = timer.get_max_compare_value().into();
    timer.set_compare_value(
        TimChannel::Ch1,
        (u64::from(max) * 4 / 5).try_into().unwrap(),
    );

    timer.enable_channel(TimChannel::Ch1, true);
    timer.generate_update_event();
    timer.enable_update_dma(true);

    Hardware {
        clock_pin,
        odr_addr,
        timer,
    }
}

/// Store a timer into a [`TimerSlot`] inside a critical section.
///
/// Performs the `Timer<'d> → Timer<'static>` transmute wrapped in
/// `ManuallyDrop` to prevent RCC disable on drop.
///
/// # Safety
/// The caller must ensure the timer peripheral is never released — it
/// must live for the program's lifetime. This is coupled to the
/// embassy-stm32 version pinned in Cargo.toml; if Timer's internal
/// representation changes to carry real borrows, this transmute must
/// be revisited.
pub(crate) unsafe fn store_timer<T: GeneralInstance4Channel>(
    timer: Timer<'_, T>,
    timer_slot: &TimerSlot<T>,
    cs: critical_section::CriticalSection,
) {
    let timer_static: ManuallyDrop<Timer<'static, T>> = ManuallyDrop::new(unsafe {
        core::mem::transmute::<Timer<'_, T>, Timer<'static, T>>(timer)
    });
    *timer_slot.borrow_ref_mut(cs) = Some(timer_static);
}
