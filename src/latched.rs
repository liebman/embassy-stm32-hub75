//! ISR-driven HUB75 latched panel driver.
//!
//! Use the [`hub75_define!`] macro to create a per-instance module containing
//! only the timer static, ISR handler, and a thin `init()` wrapper. All driver
//! logic lives in library code ([`Hub75`], [`IsrCore`]) with full IDE support.
//!
//! The ISR stops and resets the timer between planes for deterministic clock
//! alignment, then delegates BCM/DMA work to [`IsrCore::on_dma_complete()`].

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::{Poll, Waker};

use critical_section::Mutex;
use embassy_stm32::dma::{self, Channel, ChannelInstance, Transfer, TransferOptions};
use embassy_stm32::gpio::{Flex, Level, OutputType};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, RoundTo, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

use crate::bcm::{planes_from_fb, BcmState, PlaneInfo};
use crate::framebuffer::FrameBuffer;
use crate::{Config, Hub75Error, Hub75Pins8};

// ---------------------------------------------------------------------------
// Timer slot type alias (used by macro and Hub75::new)
// ---------------------------------------------------------------------------

/// Type alias for the timer static slot used by [`hub75_define!`].
#[doc(hidden)]
pub type TimerSlot<T> = Mutex<RefCell<Option<ManuallyDrop<Timer<'static, T>>>>>;

// ---------------------------------------------------------------------------
// IsrCore — type-erased ISR state (library code, no generics)
// ---------------------------------------------------------------------------

struct IsrCoreState {
    channel: Channel<'static>,
    transfer: Option<Transfer<'static>>,
    dma_request: dma::Request,
    odr_byte_addr: *mut u8,
    bcm: BcmState,
    current_fb_ptr: *const (),
    pending_planes: Option<PlaneInfo>,
    pending_fb_ptr: *const (),
    returned_fb_ptr: *const (),
}

/// Per-instance ISR core state, shared between the ISR handler and the
/// [`Hub75`] driver. Created by [`hub75_define!`] as a `static`.
pub struct IsrCore {
    state: Mutex<RefCell<Option<IsrCoreState>>>,
    swap_done: AtomicBool,
    swap_waker: Mutex<RefCell<Option<Waker>>>,
    frame_count: AtomicU32,
}

// SAFETY: IsrCore only contains Mutex/Atomic fields — all thread-safe.
unsafe impl Sync for IsrCore {}

impl IsrCore {
    /// Create a new uninitialized core. For use in `static` declarations.
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(None)),
            swap_done: AtomicBool::new(false),
            swap_waker: Mutex::new(RefCell::new(None)),
            frame_count: AtomicU32::new(0),
        }
    }

    /// Called from the ISR between `timer.stop()`/`timer.reset()` and
    /// `timer.start()`. Handles DMA transfer teardown, BCM advance, swap
    /// signaling, and new DMA kick.
    ///
    /// # Safety
    /// Must only be called from within a critical section, from the DMA
    /// transfer-complete ISR.
    pub fn on_dma_complete(&self, cs: critical_section::CriticalSection) {
        let mut borrow = self.state.borrow_ref_mut(cs);
        let state = match borrow.as_mut() {
            Some(s) => s,
            None => return,
        };

        if let Some(transfer) = state.transfer.take() {
            drop(transfer);
        }

        let frame_boundary = state.bcm.advance();

        if frame_boundary {
            self.frame_count.fetch_add(1, Ordering::Relaxed);

            if let Some(pending) = state.pending_planes.take() {
                state.returned_fb_ptr = state.current_fb_ptr;
                state.current_fb_ptr = state.pending_fb_ptr;
                state.pending_fb_ptr = core::ptr::null();
                state.bcm.update_planes(pending);
                self.signal_swap_done(cs);
            }
        }

        let (ptr, len) = state.bcm.current_plane();
        let buf = unsafe { core::slice::from_raw_parts(ptr, len) };
        let new_transfer = unsafe {
            state
                .channel
                .write_raw(state.dma_request, buf, state.odr_byte_addr, TransferOptions::default())
        };

        // SAFETY: Transfer<'a> contains Channel<'a> which is just a u8 +
        // PhantomData. The channel it borrows lives in this same static,
        // so the referent outlives the reference.
        state.transfer =
            Some(unsafe { core::mem::transmute::<Transfer<'_>, Transfer<'static>>(new_transfer) });
    }

    /// Returns the number of complete BCM frames rendered.
    pub fn frame_count(&self) -> u32 {
        self.frame_count.load(Ordering::Relaxed)
    }

    /// Queue a framebuffer swap and wait for the ISR to reach a frame boundary.
    /// Returns the raw pointer to the old framebuffer.
    ///
    /// # Safety
    /// `new_fb_ptr` must point to a valid `&'static mut FB` that was previously
    /// passed to `init` or a prior `swap`.
    pub async unsafe fn swap_inner(
        &self,
        new_planes: PlaneInfo,
        new_fb_ptr: *const (),
    ) -> *const () {
        critical_section::with(|cs| {
            let mut borrow = self.state.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb_ptr;
            self.swap_done.store(false, Ordering::Relaxed);
        });

        poll_fn(|cx| {
            if self.swap_done.load(Ordering::Acquire) {
                return Poll::Ready(());
            }
            critical_section::with(|cs| {
                if self.swap_done.load(Ordering::Relaxed) {
                    return Poll::Ready(());
                }
                *self.swap_waker.borrow_ref_mut(cs) = Some(cx.waker().clone());
                Poll::Pending
            })
        })
        .await;

        critical_section::with(|cs| {
            let borrow = self.state.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75 not initialised");
            state.returned_fb_ptr
        })
    }

    fn signal_swap_done(&self, cs: critical_section::CriticalSection) {
        self.swap_done.store(true, Ordering::Release);
        if let Some(waker) = self.swap_waker.borrow_ref_mut(cs).take() {
            waker.wake();
        }
    }

    fn init_state(&self, cs: critical_section::CriticalSection, new_state: IsrCoreState) {
        *self.state.borrow_ref_mut(cs) = Some(new_state);
    }

    fn start_first_transfer(
        &self,
        cs: critical_section::CriticalSection,
        planes: PlaneInfo,
        plane_count: usize,
        fb_ptr: *const (),
    ) {
        let mut borrow = self.state.borrow_ref_mut(cs);
        let state = borrow.as_mut().expect("Hub75 not initialised");

        state.bcm.reset_with_planes(planes, plane_count);
        state.current_fb_ptr = fb_ptr;
        state.pending_planes = None;
        state.pending_fb_ptr = core::ptr::null();
        state.returned_fb_ptr = core::ptr::null();

        self.swap_done.store(false, Ordering::Relaxed);
        self.frame_count.store(0, Ordering::Relaxed);

        let (ptr, len) = state.bcm.current_plane();
        let buf = unsafe { core::slice::from_raw_parts(ptr, len) };
        let transfer = unsafe {
            state
                .channel
                .write_raw(state.dma_request, buf, state.odr_byte_addr, TransferOptions::default())
        };
        state.transfer =
            Some(unsafe { core::mem::transmute::<Transfer<'_>, Transfer<'static>>(transfer) });
    }
}

// ---------------------------------------------------------------------------
// Hub75 — public driver handle (library code, generic over T and FB)
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by an ISR-based BCM refresh loop.
///
/// Created via the [`hub75_define!`] macro's generated `init()` function.
/// The timer type `T` is baked in by the macro; the framebuffer type `FB`
/// is locked in at construction time.
///
/// Use [`Hub75::swap()`] to double-buffer: write to one framebuffer while the
/// ISR renders from another, swapping atomically at frame boundaries.
pub struct Hub75<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> {
    _clock_pin: PwmPin<'d, T, Ch1>,
    core: &'static IsrCore,
    _fb: PhantomData<&'static FB>,
}

impl<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> Hub75<'d, T, FB> {
    /// Create a new HUB75 driver, configure hardware, and start rendering.
    ///
    /// This is called by the macro-generated `init()` wrapper. It configures
    /// GPIO pins, timer, and DMA, stores the timer into `timer_slot`, kicks
    /// the first DMA transfer, but does **not** start the timer — the caller
    /// (macro `init()`) does that after this returns.
    ///
    /// # Safety contract for callers
    /// The caller must call `timer_slot.borrow_ref().as_ref().unwrap().start()`
    /// inside a critical section immediately after this returns.
    #[doc(hidden)]
    pub fn new<D: UpDma<T> + ChannelInstance>(
        tim: Peri<'d, T>,
        clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma_ch: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pins: Hub75Pins8,
        config: Config,
        fb: &'static mut FB,
        core: &'static IsrCore,
        timer_slot: &'static TimerSlot<T>,
    ) -> Self {
        let gpio = pins.pins[0].block();
        let byte_offset: usize = if pins.base_pin == 0 { 0 } else { 1 };
        let odr_byte_addr = unsafe { (gpio.odr().as_ptr() as *mut u8).add(byte_offset) };

        for (i, pin) in pins.pins.into_iter().enumerate() {
            // SAFETY: we own the AnyPin and will leak the Flex to keep it alive.
            let peri = unsafe { Peri::new_unchecked(pin) };
            let mut flex = Flex::new(peri);
            if i == 7 {
                flex.set_level(Level::High);
            }
            flex.set_as_output(config.gpio_speed);
            core::mem::forget(flex);
        }

        let clock_pin = PwmPin::new(clock_pin, OutputType::PushPull);

        let timer = Timer::new(tim);
        timer.set_counting_mode(CountingMode::EdgeAlignedUp);
        timer.set_frequency(config.frequency, RoundTo::Slower);
        timer.enable_outputs();

        timer.set_output_compare_mode(TimChannel::Ch1, OutputCompareMode::PwmMode2);
        timer.set_output_compare_preload(TimChannel::Ch1, true);
        timer.set_autoreload_preload(true);

        let max: u32 = timer.get_max_compare_value().into();
        timer
            .set_compare_value(TimChannel::Ch1, (max * 4 / 5).try_into().ok().unwrap());

        timer.enable_channel(TimChannel::Ch1, true);
        timer.generate_update_event();
        timer.enable_update_dma(true);

        let dma_request = <D as UpDma<T>>::request(&*dma_ch);
        let channel = Channel::new(dma_ch, dma_irq);

        let (planes, plane_count) = planes_from_fb(fb);
        let fb_ptr = fb as *const FB as *const ();

        critical_section::with(|cs| {
            // SAFETY: Timer<'d> → Timer<'static>. The peripheral is consumed by
            // this driver and lives for the program's lifetime. Wrapped in
            // ManuallyDrop to prevent rcc::disable on drop.
            let timer_static: ManuallyDrop<Timer<'static, T>> = ManuallyDrop::new(unsafe {
                core::mem::transmute::<Timer<'_, T>, Timer<'static, T>>(timer)
            });
            *timer_slot.borrow_ref_mut(cs) = Some(timer_static);

            // SAFETY: Channel<'d> → Channel<'static>. Same justification.
            let channel: Channel<'static> =
                unsafe { core::mem::transmute::<Channel<'_>, Channel<'static>>(channel) };

            core.init_state(
                cs,
                IsrCoreState {
                    channel,
                    transfer: None,
                    dma_request,
                    odr_byte_addr,
                    bcm: BcmState::new(),
                    current_fb_ptr: core::ptr::null(),
                    pending_planes: None,
                    pending_fb_ptr: core::ptr::null(),
                    returned_fb_ptr: core::ptr::null(),
                },
            );

            core.start_first_transfer(cs, planes, plane_count, fb_ptr);
        });

        Self {
            _clock_pin: clock_pin,
            core,
            _fb: PhantomData,
        }
    }

    /// Returns the number of complete BCM frames rendered since init.
    pub fn frame_count(&self) -> u32 {
        self.core.frame_count()
    }

    /// Replace the displayed framebuffer, returning the previously-displayed one.
    ///
    /// Queues `new_fb` for display and yields until the ISR reaches a BCM
    /// frame boundary, at which point plane pointers are swapped atomically.
    /// Returns an exclusive reference to the old framebuffer that is no longer
    /// being read by the ISR.
    pub async fn swap(&self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
        let (new_planes, _) = planes_from_fb(new_fb);
        let fb_ptr = new_fb as *const FB as *const ();
        // SAFETY: fb_ptr originated from a valid &'static mut FB.
        let old_ptr = unsafe { self.core.swap_inner(new_planes, fb_ptr).await };
        // SAFETY: The ISR atomically swapped away from this buffer at the frame
        // boundary — it is no longer being read. The pointer originated from a
        // `&'static mut FB` passed to a previous init or swap call.
        Ok(unsafe { &mut *(old_ptr as *mut FB) })
    }
}

// ---------------------------------------------------------------------------
// Slim hub75_define! macro
// ---------------------------------------------------------------------------

/// Define a HUB75 driver instance with its own timer static and ISR handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75DmaHandler` — the DMA interrupt handler for `bind_interrupts!`
/// - `Hub75<'d, FB>` — a type alias for the driver
/// - `init()` — constructs and starts the driver
///
/// All driver logic (GPIO config, timer setup, DMA management, swap, BCM
/// state machine) lives in library code with full IDE support.
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type
/// - `$dma_ch` — the concrete DMA channel type
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_define;
///
/// hub75_define!(hub75, embassy_stm32::peripherals::TIM2, embassy_stm32::peripherals::DMA1_CH1);
///
/// bind_interrupts!(struct Irqs {
///     DMA1_CHANNEL1 =>
///         dma::InterruptHandler<peripherals::DMA1_CH1>,
///         hub75::Hub75DmaHandler;
/// });
///
/// let hub75 = hub75::init(
///     p.TIM2, p.PA0, p.DMA1_CH1, Irqs, pins,
///     Config::new().frequency(Hertz(6_000_000)),
///     fb0,
/// );
/// ```
#[macro_export]
macro_rules! hub75_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ty) => {
        #[allow(non_snake_case)]
        pub mod $mod_name {
            use $crate::__macro_support::critical_section;
            use $crate::__macro_support::embassy_stm32::dma::{self, ChannelInstance};
            use $crate::__macro_support::embassy_stm32::interrupt::typelevel::{Binding, Handler};
            use $crate::__macro_support::embassy_stm32::timer::{Ch1, TimerPin, UpDma};
            use $crate::__macro_support::embassy_stm32::Peri;
            use $crate::framebuffer::FrameBuffer;
            use $crate::latched::{self, IsrCore, TimerSlot};

            static TIMER: TimerSlot<$timer> =
                critical_section::Mutex::new(core::cell::RefCell::new(None));

            static CORE: IsrCore = IsrCore::new();

            /// HUB75 DMA interrupt handler for this instance.
            pub struct Hub75DmaHandler;

            impl Handler<<$dma_ch as ChannelInstance>::Interrupt> for Hub75DmaHandler {
                unsafe fn on_interrupt() {
                    critical_section::with(|cs| {
                        let mut t = TIMER.borrow_ref_mut(cs);
                        let timer = match t.as_mut() {
                            Some(t) => t,
                            None => return,
                        };
                        timer.stop();
                        timer.reset();
                        CORE.on_dma_complete(cs);
                        timer.start();
                    });
                }
            }

            /// Type alias for the HUB75 driver bound to this instance's timer.
            pub type Hub75<'d, FB> = latched::Hub75<'d, $timer, FB>;

            /// Initialize the HUB75 driver, configure hardware, and start
            /// rendering from the provided framebuffer.
            pub fn init<'d, FB: FrameBuffer>(
                tim: Peri<'d, $timer>,
                clock_pin: Peri<'d, impl TimerPin<$timer, Ch1>>,
                dma_ch: Peri<'d, $dma_ch>,
                dma_irq: impl Binding<
                        <$dma_ch as ChannelInstance>::Interrupt,
                        dma::InterruptHandler<$dma_ch>,
                    > + Binding<
                        <$dma_ch as ChannelInstance>::Interrupt,
                        Hub75DmaHandler,
                    > + 'd,
                pins: $crate::Hub75Pins8,
                config: $crate::Config,
                fb: &'static mut FB,
            ) -> Hub75<'d, FB>
            where
                $dma_ch: UpDma<$timer>,
            {
                let hub75 = latched::Hub75::new(
                    tim, clock_pin, dma_ch, dma_irq, pins, config, fb,
                    &CORE, &TIMER,
                );
                critical_section::with(|cs| {
                    TIMER.borrow_ref(cs).as_ref().unwrap().start();
                });
                hub75
            }
        }
    };
}
