//! ISR-driven HUB75 latched panel driver.
//!
//! Once [`Hub75::start()`] is called, DMA transfer-complete interrupts drive
//! the BCM refresh loop autonomously. The timer free-runs, generating a PWM
//! pixel clock on CH1 and triggering DMA byte-transfers from the framebuffer
//! to the GPIO ODR byte.
//!
//! Embassy's [`dma::InterruptHandler`] clears hardware flags portably; our
//! [`Hub75DmaHandler`] runs immediately after to advance the BCM state machine
//! and kick the next DMA transfer. Both are chained on the same interrupt
//! vector via [`bind_interrupts!`](embassy_stm32::bind_interrupts).

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::{Poll, Waker};

use critical_section::Mutex;
use embassy_stm32::dma::{self, Channel, ChannelInstance, Transfer, TransferOptions};
use embassy_stm32::gpio::{OutputType, Pin};
use embassy_stm32::interrupt::typelevel::{Binding, Handler};
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, RoundTo, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

use crate::bcm::{planes_from_fb, BcmState, PlaneInfo};
use crate::framebuffer::FrameBuffer;
use crate::{Hub75Error, Hub75Pins8, Idle};

// ---------------------------------------------------------------------------
// ISR shared state
// ---------------------------------------------------------------------------

struct IsrState {
    channel: Channel<'static>,
    transfer: Option<Transfer<'static>>,
    dma_request: dma::Request,
    odr_byte_addr: *mut u8,
    timer_cnt_addr: *mut u32,
    bcm: BcmState,
    current_fb_ptr: *const (),
    pending_planes: Option<PlaneInfo>,
    pending_fb_ptr: *const (),
    returned_fb_ptr: *const (),
}

// SAFETY: All access is serialised by `critical_section`.
unsafe impl Send for IsrState {}

static ISR_STATE: Mutex<RefCell<Option<IsrState>>> = Mutex::new(RefCell::new(None));
static SWAP_DONE: AtomicBool = AtomicBool::new(false);
static SWAP_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static FRAME_COUNT: AtomicU32 = AtomicU32::new(0);

fn signal_swap_done(cs: critical_section::CriticalSection) {
    SWAP_DONE.store(true, Ordering::Release);
    if let Some(waker) = SWAP_WAKER.borrow_ref_mut(cs).take() {
        waker.wake();
    }
}

// ---------------------------------------------------------------------------
// DMA interrupt handler (runs AFTER embassy's dma::InterruptHandler)
// ---------------------------------------------------------------------------

/// HUB75 DMA interrupt handler.
///
/// Must be bound to the same DMA channel interrupt as
/// [`dma::InterruptHandler`], listed **after** it in [`bind_interrupts!`] so
/// that embassy clears the hardware flags first.
///
/// # Example
/// ```ignore
/// bind_interrupts!(struct Irqs {
///     DMA1_CHANNEL1 =>
///         dma::InterruptHandler<peripherals::DMA1_CH1>,
///         Hub75DmaHandler<peripherals::DMA1_CH1>;
/// });
/// ```
pub struct Hub75DmaHandler<T: ChannelInstance> {
    _phantom: PhantomData<T>,
}

impl<T: ChannelInstance> Handler<T::Interrupt> for Hub75DmaHandler<T> {
    unsafe fn on_interrupt() {
        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = match borrow.as_mut() {
                Some(s) => s,
                None => return,
            };

            // Drop the completed transfer (instant — DMA already finished).
            match state.transfer.take() {
                Some(transfer) => {
                    drop(transfer);
                }
                None => {
                    defmt::error!("transfer is None");
                }
            }

            // Advance BCM state machine.
            let frame_boundary = state.bcm.advance();

            if frame_boundary {
                FRAME_COUNT.fetch_add(1, Ordering::Relaxed);

                if let Some(pending) = state.pending_planes.take() {
                    state.returned_fb_ptr = state.current_fb_ptr;
                    state.current_fb_ptr = state.pending_fb_ptr;
                    state.pending_fb_ptr = core::ptr::null();
                    state.bcm.update_planes(pending);
                    signal_swap_done(cs);
                }
            }

            // Reset the generic timer's counter to 0 to synchronize clock phase
            core::ptr::write_volatile(state.timer_cnt_addr, 0);

            // Start next DMA transfer.
            let (ptr, len) = state.bcm.current_plane();
            let buf = core::slice::from_raw_parts(ptr, len);
            let new_transfer = state.channel.write_raw(
                state.dma_request,
                buf,
                state.odr_byte_addr,
                TransferOptions::default(),
            );
            // SAFETY: Transfer<'a> contains Channel<'a> which is just a u8 +
            // PhantomData. The channel it borrows lives in this same static, so
            // the referent outlives the reference.
            state.transfer =
                Some(core::mem::transmute::<Transfer<'_>, Transfer<'static>>(new_transfer));
        });
    }
}

// ---------------------------------------------------------------------------
// Public driver handle
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by an ISR-based BCM refresh loop.
///
/// Uses a typestate pattern: `Hub75<'d, T, Idle>` is the idle state returned by
/// [`Hub75::new()`]. Calling [`Hub75::start()`] consumes it and returns
/// `Hub75<'d, T, FB>`, locking in the framebuffer type. [`Hub75::swap()`] is
/// only available on the running state and enforces type-safety at compile time.
///
/// The timer `T` is stored directly in the struct (not in the ISR state) since
/// it only needs to be started once and kept alive.
pub struct Hub75<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static = Idle> {
    timer: Timer<'d, T>,
    _clock_pin: PwmPin<'d, T, Ch1>,
    _fb: PhantomData<&'static FB>,
}

impl<'d, T: GeneralInstance4Channel> Hub75<'d, T, Idle> {
    /// Create a new HUB75 driver and configure hardware.
    ///
    /// Sets up GPIO pins, timer PWM clock, and DMA channel. Does **not** start
    /// rendering — call [`Hub75::start()`] with a framebuffer to begin.
    ///
    /// The `dma_irq` parameter must satisfy bindings for **both**
    /// [`dma::InterruptHandler`] and [`Hub75DmaHandler`] on the same interrupt.
    /// Use [`bind_interrupts!`](embassy_stm32::bind_interrupts) to create it.
    pub fn new<D: UpDma<T>>(
        tim: Peri<'d, T>,
        clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma_ch: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>>
            + Binding<D::Interrupt, Hub75DmaHandler<D>>
            + 'd,
        pins: Hub75Pins8,
        frequency: Hertz,
    ) -> Self {
        let gpio = pins.pins[0].block();

        for pin in &pins.pins {
            let n = pin.pin() as usize;
            gpio.moder()
                .modify(|w| w.set_moder(n, pac::gpio::vals::Moder::OUTPUT));
            gpio.ospeedr()
                .modify(|w| w.set_ospeedr(n, pac::gpio::vals::Ospeedr::LOW_SPEED));
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

        // Grab the physical memory address of the Counter (CNT) register directly 
        // from the timer object we just created.
        let timer_cnt_addr = timer.regs_core().cnt().as_ptr() as *mut u32;

        timer.set_counting_mode(CountingMode::EdgeAlignedUp);
        timer.set_frequency(frequency, RoundTo::Slower);
        timer.enable_outputs();

        // PWM mode 2: CLK rises at CCR, DMA writes at update event (CNT=0).
        timer.set_output_compare_mode(TimChannel::Ch1, OutputCompareMode::PwmMode2);
        timer.set_output_compare_preload(TimChannel::Ch1, true);
        timer.set_autoreload_preload(true);

        let max: u32 = timer.get_max_compare_value().into();
        timer.set_compare_value(TimChannel::Ch1, max.div_ceil(2).try_into().ok().unwrap());

        timer.enable_channel(TimChannel::Ch1, true);
        timer.generate_update_event();

        // Enable timer update-event DMA trigger (stays enabled permanently)
        timer.enable_update_dma(true);

        // Build DMA channel + request (DMAMUX routing for timer update event)
        let dma_request = dma_ch.request();
        let channel = Channel::new(dma_ch, dma_irq);

        // Store DMA state for the ISR. Timer is NOT started yet.
        critical_section::with(|cs| {
            // SAFETY: Channel<'d> → Channel<'static>. The peripheral is consumed
            // by this driver and lives for the program's lifetime.
            let channel: Channel<'static> =
                unsafe { core::mem::transmute::<Channel<'_>, Channel<'static>>(channel) };

            *ISR_STATE.borrow_ref_mut(cs) = Some(IsrState {
                channel,
                transfer: None,
                dma_request,
                odr_byte_addr,
                timer_cnt_addr,
                bcm: BcmState::new(),
                current_fb_ptr: core::ptr::null(),
                pending_planes: None,
                pending_fb_ptr: core::ptr::null(),
                returned_fb_ptr: core::ptr::null(),
            });
        });

        Self {
            timer,
            _clock_pin: clock_pin,
            _fb: PhantomData,
        }
    }

    /// Start display refresh, consuming the idle handle and returning a
    /// running handle typed to the framebuffer.
    ///
    /// The framebuffer must be `'static` because the ISR will continuously
    /// read from it until [`Hub75::swap()`] replaces it. Exclusive access is
    /// surrendered to the ISR; the caller gets it back via [`Hub75::swap()`].
    ///
    /// # Panics
    /// Panics if called while a transfer is already in flight.
    pub fn start<FB: FrameBuffer>(
        self,
        fb: &'static mut FB,
    ) -> Result<Hub75<'d, T, FB>, Hub75Error> {
        let (planes, plane_count) = planes_from_fb(fb);
        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");

            if state.transfer.is_some() {
                panic!("start() called while already running");
            }

            state.bcm.reset_with_planes(planes, plane_count);
            state.current_fb_ptr = fb as *const FB as *const ();
            state.pending_planes = None;
            state.pending_fb_ptr = core::ptr::null();
            state.returned_fb_ptr = core::ptr::null();

            SWAP_DONE.store(false, Ordering::Relaxed);
            FRAME_COUNT.store(0, Ordering::Relaxed);

            // Kick the first DMA transfer.
            let (ptr, len) = state.bcm.current_plane();
            let buf = unsafe { core::slice::from_raw_parts(ptr, len) };
            let transfer = unsafe {
                state
                    .channel
                    .write_raw(state.dma_request, buf, state.odr_byte_addr, TransferOptions::default())
            };
            // SAFETY: same transmute justification as in on_interrupt.
            state.transfer =
                Some(unsafe { core::mem::transmute::<Transfer<'_>, Transfer<'static>>(transfer) });
        });

        // Start the timer — update events will now trigger DMA.
        self.timer.start();

        Ok(Hub75 {
            timer: self.timer,
            _clock_pin: self._clock_pin,
            _fb: PhantomData,
        })
    }
}

impl<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> Hub75<'d, T, FB> {
    /// Returns the number of complete BCM frames rendered since [`start()`](Hub75::start).
    pub fn frame_count(&self) -> u32 {
        FRAME_COUNT.load(Ordering::Relaxed)
    }

    /// Replace the displayed framebuffer, returning the previously-displayed one.
    ///
    /// Queues `new_fb` for display and yields until the ISR reaches a BCM
    /// frame boundary, at which point plane pointers are swapped atomically.
    /// Returns an exclusive reference to the old framebuffer that is no longer
    /// being read by the ISR.
    ///
    /// Type safety is enforced at compile time: `swap()` only accepts the same
    /// framebuffer type `FB` that was passed to [`start()`](Hub75::start).
    pub async fn swap(&self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
        let (new_planes, _) = planes_from_fb(new_fb);

        critical_section::with(|cs| {
            let mut borrow = ISR_STATE.borrow_ref_mut(cs);
            let state = borrow.as_mut().expect("Hub75 not initialised");
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb as *const FB as *const ();
            SWAP_DONE.store(false, Ordering::Relaxed);
        });

        poll_fn(|cx| {
            if SWAP_DONE.load(Ordering::Acquire) {
                return Poll::Ready(());
            }
            critical_section::with(|cs| {
                if SWAP_DONE.load(Ordering::Relaxed) {
                    return Poll::Ready(());
                }
                *SWAP_WAKER.borrow_ref_mut(cs) = Some(cx.waker().clone());
                Poll::Pending
            })
        })
        .await;

        critical_section::with(|cs| {
            let borrow = ISR_STATE.borrow_ref(cs);
            let state = borrow.as_ref().expect("Hub75 not initialised");
            let ptr = state.returned_fb_ptr;
            // SAFETY: The ISR atomically swapped away from this buffer at the
            // frame boundary — it is no longer being read. The pointer originated
            // from a `&'static mut FB` passed to a previous start() or swap()
            // call with the same concrete type FB (enforced by the type system).
            Ok(unsafe { &mut *(ptr as *mut FB) })
        })
    }
}
