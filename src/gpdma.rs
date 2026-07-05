//! GPDMA linked-list backend for HUB75 panels.
//!
//! Achieves BCM (Binary Code Modulation) weighting by duplicating
//! [`LinearItem`] descriptors in a single linked-list chain, matching
//! the `full-chain-dma` pattern from `esp-hub75`. Plane `i` gets
//! `2^(N-1-i)` consecutive descriptors all pointing at the same data,
//! giving a total chain length of `2^N - 1` items per BCM frame.
//!
//! With `TR2.TCEM = LAST_LINKED_LIST_ITEM`, the GPDMA walks the
//! entire chain autonomously and fires **one** TC interrupt at the
//! terminal descriptor — one ISR per frame.
//!
//! Works with **any** GPDMA channel (no 2D capability required).
//!
//! The ISR stops and resets the timer at each frame boundary for
//! deterministic clock alignment, then delegates swap/restart work
//! to [`GpdmaIsrCore::on_chain_complete()`].

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::ManuallyDrop;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::{Poll, Waker};

use critical_section::Mutex;
#[doc(hidden)]
pub use embassy_stm32::dma::linked_list::LinearItem;
use embassy_stm32::dma::word::WordSize;
use embassy_stm32::dma::{self, ChannelInstance};
use embassy_stm32::gpio::OutputType;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac::gpdma::{regs, vals, Gpdma};
use embassy_stm32::timer::low_level::{CountingMode, OutputCompareMode, RoundTo, Timer};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, Channel as TimChannel, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

use crate::bcm::{planes_from_fb, PlaneInfo, MAX_PLANES};
use crate::framebuffer::FrameBuffer;
use crate::{Config, Hub75Error, Hub75Pins};

/// Type alias for the timer static slot used by `hub75_gpdma_define!`.
#[doc(hidden)]
pub type TimerSlot<T> = Mutex<RefCell<Option<ManuallyDrop<Timer<'static, T>>>>>;

/// Maximum number of `LinearItem` descriptors in a full BCM chain.
/// Equals `2^MAX_PLANES - 1` = 255.
pub const MAX_DESCRIPTORS: usize = (1 << MAX_PLANES) - 1;

/// Create a zeroed `LinearItem`, suitable for const/static init.
#[doc(hidden)]
#[must_use]
pub const fn zeroed_linear_item() -> LinearItem {
    LinearItem {
        tr1: regs::ChTr1(0),
        tr2: regs::ChTr2(0),
        br1: regs::ChBr1(0),
        sar: 0,
        dar: 0,
        llr: regs::ChLlr(0),
    }
}

/// Number of descriptors for a given plane count: `2^plane_count - 1`.
const fn descriptor_count(plane_count: usize) -> usize {
    (1 << plane_count) - 1
}

// ---------------------------------------------------------------------------
// Register-field helpers
// ---------------------------------------------------------------------------

fn word_size_to_dw(ws: WordSize) -> vals::Dw {
    match ws {
        WordSize::OneByte => vals::Dw::BYTE,
        WordSize::TwoBytes => vals::Dw::HALF_WORD,
        _ => panic!("HUB75 only supports byte and halfword DMA transfers"),
    }
}

fn build_tr1(word_size: WordSize) -> regs::ChTr1 {
    let mut tr1 = regs::ChTr1(0);
    let dw = word_size_to_dw(word_size);
    tr1.set_sdw(dw);
    tr1.set_ddw(dw);
    tr1.set_sinc(true);
    tr1.set_dinc(false);
    tr1.set_sap(vals::Ap::PORT0);
    tr1.set_dap(vals::Ap::PORT1);
    tr1
}

fn build_tr2(request: dma::Request) -> regs::ChTr2 {
    let mut tr2 = regs::ChTr2(0);
    tr2.set_reqsel(request);
    tr2.set_dreq(vals::Dreq::DESTINATION_PERIPHERAL);
    tr2.set_tcem(vals::Tcem::LAST_LINKED_LIST_ITEM);
    tr2
}

fn build_br1(byte_len: u16) -> regs::ChBr1 {
    let mut br1 = regs::ChBr1(0);
    br1.set_bndt(byte_len);
    br1
}

/// Pre-compute the CR register value (without EN bit).
fn build_cr() -> u32 {
    let mut cr = regs::ChCr(0);
    cr.set_prio(vals::Prio::HIGH);
    cr.set_tcie(true);
    cr.set_useie(true);
    cr.set_uleie(true);
    cr.set_dteie(true);
    cr.0
}

// ---------------------------------------------------------------------------
// Item chain construction
// ---------------------------------------------------------------------------

/// LBAR value (upper 16 bits of item array base address).
fn item_lbar(items: &[LinearItem]) -> u16 {
    ((items.as_ptr() as u32) >> 16) as u16
}

/// Lower 16-bit offset address of an item within its 64 KB region.
#[allow(clippy::cast_possible_truncation)]
fn item_offset(item: &LinearItem) -> u16 {
    core::ptr::from_ref(item) as u32 as u16
}

/// Build a `ChLlr` value that links to the item at the given 16-bit
/// offset, with all 6 linear update flags set.
fn build_llr_link(next_offset: u16) -> regs::ChLlr {
    let mut llr = regs::ChLlr(0);
    llr.set_ut1(true);
    llr.set_ut2(true);
    llr.set_ub1(true);
    llr.set_usa(true);
    llr.set_uda(true);
    llr.set_ull(true);
    llr.set_la(next_offset >> 2);
    llr
}

/// Build the initial LLR register value that causes the GPDMA to
/// fetch `items[0]` on first enable.
fn initial_llr(items: &[LinearItem]) -> regs::ChLlr {
    build_llr_link(item_offset(&items[0]))
}

/// Populate the linked-list chain with duplicated descriptors for
/// BCM weighting.
///
/// For each plane `i`, creates `2^(N-1-i)` consecutive `LinearItem`
/// entries pointing at the same plane data. The total chain length
/// is `2^N - 1`. The terminal item has `llr = 0`, which triggers a
/// TC interrupt at the frame boundary (via `TCEM = LAST_LLI`).
///
/// Returns the number of active items in the chain.
#[doc(hidden)]
pub fn build_item_chain(
    items: &mut [LinearItem; MAX_DESCRIPTORS],
    planes: &PlaneInfo,
    plane_count: usize,
    odr_addr: *mut u8,
    word_size: WordSize,
    request: dma::Request,
) -> usize {
    assert!(
        plane_count > 0 && plane_count <= MAX_PLANES,
        "plane_count {plane_count} out of range 1..={MAX_PLANES}"
    );

    let total = descriptor_count(plane_count);
    assert!(total <= MAX_DESCRIPTORS);

    let base = items.as_ptr() as usize;
    let end = base + core::mem::size_of::<LinearItem>() * total - 1;
    assert_eq!(
        base >> 16,
        end >> 16,
        "GPDMA descriptor chain spans a 64 KB boundary"
    );

    let tr1 = build_tr1(word_size);
    let tr2 = build_tr2(request);
    let dar = odr_addr as u32;

    let mut idx = 0;
    for (plane, &(ptr, len)) in planes.iter().enumerate().take(plane_count) {
        #[allow(clippy::cast_possible_truncation)]
        let byte_len = len as u16;
        debug_assert!(
            usize::from(byte_len) == len,
            "bitplane exceeds 64 KB BNDT limit"
        );

        let br1 = build_br1(byte_len);
        let sar = ptr as u32;
        let reps = 1usize << (plane_count - 1 - plane);

        for _ in 0..reps {
            let llr = if idx + 1 < total {
                build_llr_link(item_offset(&items[idx + 1]))
            } else {
                regs::ChLlr(0)
            };

            items[idx] = LinearItem {
                tr1,
                tr2,
                br1,
                sar,
                dar,
                llr,
            };
            idx += 1;
        }
    }

    total
}

/// Patch source-address fields across all active descriptors.
///
/// Iterates the same plane-x-reps pattern used during construction
/// to update each item's `sar` with the new plane pointer.
#[doc(hidden)]
pub fn update_item_sources(
    items: &mut [LinearItem; MAX_DESCRIPTORS],
    planes: &PlaneInfo,
    plane_count: usize,
) {
    let mut idx = 0;
    for (plane, &(ptr, _)) in planes.iter().enumerate().take(plane_count) {
        let reps = 1usize << (plane_count - 1 - plane);
        let sar = ptr as u32;
        for _ in 0..reps {
            items[idx].sar = sar;
            idx += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// GPDMA channel bootstrap / restart (PAC-level)
// ---------------------------------------------------------------------------

/// Configure and start the GPDMA channel for the first linked-list pass.
#[doc(hidden)]
pub fn start_gpdma_chain(gpdma: Gpdma, ch_num: usize, items: &[LinearItem; MAX_DESCRIPTORS]) {
    let ch = gpdma.ch(ch_num);
    let cr_val = build_cr();

    ch.cr().write(|w| w.set_reset(true));
    ch.fcr().write(|w| {
        w.set_tcf(true);
        w.set_htf(true);
        w.set_suspf(true);
        w.set_tof(true);
        w.set_ulef(true);
        w.set_usef(true);
        w.set_dtef(true);
    });
    ch.lbar().write(|w| w.set_lba(item_lbar(items)));
    ch.br1().write(|w| w.set_bndt(0));
    ch.llr().write_value(initial_llr(items));
    ch.tr3().write(|_| {});
    ch.cr().write_value(regs::ChCr(cr_val));
    ch.cr().modify(|w| w.set_en(true));
}

/// Reset and restart the GPDMA linked-list chain from item[0].
///
/// Called from the ISR after frame-boundary processing.
#[doc(hidden)]
pub fn restart_gpdma_chain(gpdma: Gpdma, ch_num: usize, items: &[LinearItem; MAX_DESCRIPTORS]) {
    start_gpdma_chain(gpdma, ch_num, items);
}

// ---------------------------------------------------------------------------
// GpdmaIsrCore — frame-boundary state (library code, no generics)
// ---------------------------------------------------------------------------

struct GpdmaIsrCoreState {
    planes: PlaneInfo,
    plane_count: usize,
    current_fb_ptr: *const (),
    pending_planes: Option<PlaneInfo>,
    pending_fb_ptr: *const (),
    returned_fb_ptr: *const (),
}

// SAFETY: Raw pointers target static framebuffer allocations or are null.
// All access is guarded by the critical-section Mutex.
unsafe impl Send for GpdmaIsrCoreState {}

/// Per-instance ISR core state for the GPDMA backend, shared between
/// the ISR handler and the [`Hub75Gpdma`] driver. Created by
/// `hub75_gpdma_define!` as a `static`.
#[doc(hidden)]
pub struct GpdmaIsrCore {
    state: Mutex<RefCell<Option<GpdmaIsrCoreState>>>,
    swap_done: AtomicBool,
    swap_waker: Mutex<RefCell<Option<Waker>>>,
    frame_count: AtomicU32,
}

// SAFETY: All fields are inherently Sync (Mutex, Atomics) given
// GpdmaIsrCoreState: Send (ensured above).
unsafe impl Sync for GpdmaIsrCore {}

impl GpdmaIsrCore {
    /// Create a new uninitialized core. For use in `static` declarations.
    #[must_use]
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(None)),
            swap_done: AtomicBool::new(false),
            swap_waker: Mutex::new(RefCell::new(None)),
            frame_count: AtomicU32::new(0),
        }
    }

    /// Called from the ISR at each frame boundary (GPDMA chain complete).
    ///
    /// Handles swap signaling and patches item SAR fields when a new
    /// framebuffer is pending.
    ///
    /// # Safety
    /// Must only be called from within a critical section, from the
    /// GPDMA transfer-complete ISR. `items` must point to the static
    /// item array for this instance.
    pub unsafe fn on_chain_complete(
        &self,
        cs: critical_section::CriticalSection,
        items: &mut [LinearItem; MAX_DESCRIPTORS],
    ) {
        let mut borrow = self.state.borrow_ref_mut(cs);
        let Some(state) = borrow.as_mut() else { return };

        self.frame_count.fetch_add(1, Ordering::Relaxed);

        if let Some(pending) = state.pending_planes.take() {
            state.returned_fb_ptr = state.current_fb_ptr;
            state.current_fb_ptr = state.pending_fb_ptr;
            state.pending_fb_ptr = core::ptr::null();
            state.planes = pending;

            update_item_sources(items, &pending, state.plane_count);

            self.signal_swap_done(cs);
        }
    }

    /// Returns the number of complete BCM frames rendered.
    pub fn frame_count(&self) -> u32 {
        self.frame_count.load(Ordering::Relaxed)
    }

    /// Queue a framebuffer swap and wait for the ISR to reach a frame
    /// boundary. Returns the raw pointer to the old framebuffer.
    ///
    /// # Safety
    /// `new_fb_ptr` must point to a valid `&'static mut FB`.
    ///
    /// # Errors
    /// Returns `Hub75Error::NotInitialised` if the driver state has
    /// not been set up.
    pub async unsafe fn swap_inner(
        &self,
        new_planes: PlaneInfo,
        new_fb_ptr: *const (),
    ) -> Result<*const (), Hub75Error> {
        critical_section::with(|cs| {
            let mut borrow = self.state.borrow_ref_mut(cs);
            let state = borrow.as_mut().ok_or(Hub75Error::NotInitialised)?;
            state.pending_planes = Some(new_planes);
            state.pending_fb_ptr = new_fb_ptr;
            self.swap_done.store(false, Ordering::Relaxed);
            Ok(())
        })?;

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
            let state = borrow.as_ref().ok_or(Hub75Error::NotInitialised)?;
            Ok(state.returned_fb_ptr)
        })
    }

    fn signal_swap_done(&self, cs: critical_section::CriticalSection) {
        self.swap_done.store(true, Ordering::Release);
        if let Some(waker) = self.swap_waker.borrow_ref_mut(cs).take() {
            waker.wake();
        }
    }

    #[doc(hidden)]
    pub fn init_state(
        &self,
        cs: critical_section::CriticalSection,
        planes: PlaneInfo,
        plane_count: usize,
        fb_ptr: *const (),
    ) {
        *self.state.borrow_ref_mut(cs) = Some(GpdmaIsrCoreState {
            planes,
            plane_count,
            current_fb_ptr: fb_ptr,
            pending_planes: None,
            pending_fb_ptr: core::ptr::null(),
            returned_fb_ptr: core::ptr::null(),
        });
        self.swap_done.store(false, Ordering::Relaxed);
        self.frame_count.store(0, Ordering::Relaxed);
    }
}

// ---------------------------------------------------------------------------
// Hub75Gpdma — public driver handle (library code, generic over T and FB)
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by a GPDMA linked-list refresh
/// loop.
///
/// BCM weighting is achieved by duplicating `LinearItem` descriptors
/// (one per bitplane repetition) in a single chain. The GPDMA
/// traverses the chain autonomously; one ISR fires per complete
/// BCM frame.
///
/// Created via the `hub75_gpdma_define!` macro's generated `init()`
/// function. Use [`Hub75Gpdma::swap()`] to double-buffer.
pub struct Hub75Gpdma<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> {
    _clock_pin: PwmPin<'d, T, Ch1>,
    core: &'static GpdmaIsrCore,
    _fb: PhantomData<&'static FB>,
}

impl<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> Hub75Gpdma<'d, T, FB> {
    /// Create a new GPDMA-backed HUB75 driver, configure hardware,
    /// and start rendering.
    ///
    /// This is called by the macro-generated `init()` wrapper.
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub fn new<D: UpDma<T> + ChannelInstance, P: Hub75Pins>(
        tim: Peri<'d, T>,
        clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma_ch: Peri<'d, D>,
        _dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pins: P,
        config: Config,
        fb: &'static mut FB,
        core: &'static GpdmaIsrCore,
        timer_slot: &'static TimerSlot<T>,
        items: &'static mut [LinearItem; MAX_DESCRIPTORS],
        gpdma: Gpdma,
        ch_num: usize,
    ) -> Self
    where
        FB: FrameBuffer<Word = P::Word>,
    {
        let odr_addr = pins.configure_and_get_odr(config.gpio_speed).as_ptr();

        let clock_pin = PwmPin::new(clock_pin, OutputType::PushPull);

        // --- Timer setup (identical to the plain DMA backend) ---
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

        let request = <D as UpDma<T>>::request(&*dma_ch);
        let _claimed = ManuallyDrop::new(dma_ch);

        // --- Build the linked-list descriptor chain ---
        let (planes, plane_count) = planes_from_fb(fb);
        let fb_ptr = core::ptr::from_ref::<FB>(fb).cast::<()>();

        build_item_chain(
            items,
            &planes,
            plane_count,
            odr_addr,
            P::DMA_WORD_SIZE,
            request,
        );

        critical_section::with(|cs| {
            // SAFETY: Timer<'d> → Timer<'static>. See dma.rs for rationale.
            let timer_static: ManuallyDrop<Timer<'static, T>> = ManuallyDrop::new(unsafe {
                core::mem::transmute::<Timer<'_, T>, Timer<'static, T>>(timer)
            });
            *timer_slot.borrow_ref_mut(cs) = Some(timer_static);

            core.init_state(cs, planes, plane_count, fb_ptr);

            start_gpdma_chain(gpdma, ch_num, items);
            timer_slot.borrow_ref(cs).as_ref().unwrap().start();
        });

        Self {
            _clock_pin: clock_pin,
            core,
            _fb: PhantomData,
        }
    }

    /// Returns the number of complete BCM frames rendered since init.
    #[must_use]
    pub fn frame_count(&self) -> u32 {
        self.core.frame_count()
    }

    /// Replace the displayed framebuffer, returning the previously-displayed one.
    ///
    /// Queues `new_fb` for display and yields until the ISR reaches a
    /// frame boundary, at which point plane pointers are swapped
    /// atomically. Returns an exclusive reference to the old
    /// framebuffer that is no longer being read by the GPDMA.
    ///
    /// # Errors
    /// Returns `Hub75Error::NotInitialised` if the driver has not
    /// been initialised.
    pub async fn swap(&mut self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
        let (new_planes, _) = planes_from_fb(new_fb);
        let fb_ptr = core::ptr::from_ref::<FB>(new_fb).cast::<()>();
        let old_ptr = unsafe { self.core.swap_inner(new_planes, fb_ptr).await? };
        Ok(unsafe { &mut *(old_ptr as *mut FB) })
    }
}

// ---------------------------------------------------------------------------
// Helper macros — derive PAC instance / channel number from a channel ident
// ---------------------------------------------------------------------------

/// Resolve a GPDMA channel identifier (e.g. `GPDMA1_CH12`) to its PAC
/// register block.
#[doc(hidden)]
#[macro_export]
macro_rules! __gpdma_ch_pac {
    (GPDMA1_CH0) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH1) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH2) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH3) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH4) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH5) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH6) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH7) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH8) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH9) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH10) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH11) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH12) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH13) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH14) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA1_CH15) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA1
    };
    (GPDMA2_CH0) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH1) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH2) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH3) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH4) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH5) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH6) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
    (GPDMA2_CH7) => {
        $crate::__macro_support::embassy_stm32::pac::GPDMA2
    };
}

/// Resolve a GPDMA channel identifier (e.g. `GPDMA1_CH12`) to its
/// channel number within the GPDMA instance.
#[doc(hidden)]
#[macro_export]
macro_rules! __gpdma_ch_num {
    (GPDMA1_CH0) => {
        0usize
    };
    (GPDMA1_CH1) => {
        1usize
    };
    (GPDMA1_CH2) => {
        2usize
    };
    (GPDMA1_CH3) => {
        3usize
    };
    (GPDMA1_CH4) => {
        4usize
    };
    (GPDMA1_CH5) => {
        5usize
    };
    (GPDMA1_CH6) => {
        6usize
    };
    (GPDMA1_CH7) => {
        7usize
    };
    (GPDMA1_CH8) => {
        8usize
    };
    (GPDMA1_CH9) => {
        9usize
    };
    (GPDMA1_CH10) => {
        10usize
    };
    (GPDMA1_CH11) => {
        11usize
    };
    (GPDMA1_CH12) => {
        12usize
    };
    (GPDMA1_CH13) => {
        13usize
    };
    (GPDMA1_CH14) => {
        14usize
    };
    (GPDMA1_CH15) => {
        15usize
    };
    (GPDMA2_CH0) => {
        0usize
    };
    (GPDMA2_CH1) => {
        1usize
    };
    (GPDMA2_CH2) => {
        2usize
    };
    (GPDMA2_CH3) => {
        3usize
    };
    (GPDMA2_CH4) => {
        4usize
    };
    (GPDMA2_CH5) => {
        5usize
    };
    (GPDMA2_CH6) => {
        6usize
    };
    (GPDMA2_CH7) => {
        7usize
    };
}

// ---------------------------------------------------------------------------
// hub75_gpdma_define! macro
// ---------------------------------------------------------------------------

/// Define a GPDMA-backed HUB75 driver instance with its own timer
/// static, descriptor array, and ISR handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75GpdmaHandler` — the GPDMA interrupt handler for `bind_interrupts!`
/// - `Hub75Gpdma<'d, FB>` — a type alias for the driver
/// - `init()` — constructs and starts the driver
///
/// BCM weighting is achieved by duplicating `LinearItem` descriptors
/// in the chain. Any GPDMA channel works (2D capability not required).
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type
/// - `$dma_ch` — the GPDMA channel peripheral name (e.g. `GPDMA1_CH0`)
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_gpdma_define;
///
/// hub75_gpdma_define!(hub75, peripherals::TIM2, GPDMA1_CH0);
///
/// bind_interrupts!(struct Irqs {
///     GPDMA1_CHANNEL0 =>
///         dma::InterruptHandler<peripherals::GPDMA1_CH0>,
///         hub75::Hub75GpdmaHandler;
/// });
///
/// let hub75 = hub75::init(
///     p.TIM2, p.PA0, p.GPDMA1_CH0, Irqs, pins,
///     Config::new().frequency(Hertz(10_000_000)),
///     fb0,
/// );
/// ```
#[macro_export]
macro_rules! hub75_gpdma_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ident) => {
        #[allow(non_snake_case)]
        pub mod $mod_name {
            use $crate::__macro_support::critical_section;
            use $crate::__macro_support::embassy_stm32::dma::{self, ChannelInstance};
            use $crate::__macro_support::embassy_stm32::interrupt::typelevel::{Binding, Handler};
            use $crate::__macro_support::embassy_stm32::timer::{Ch1, TimerPin, UpDma};
            use $crate::__macro_support::embassy_stm32::Peri;
            use $crate::framebuffer::FrameBuffer;
            use $crate::gpdma::{self as gpdma_driver, GpdmaIsrCore, TimerSlot};

            type DmaCh = $crate::__macro_support::embassy_stm32::peripherals::$dma_ch;

            static TIMER: TimerSlot<$timer> =
                critical_section::Mutex::new(core::cell::RefCell::new(None));

            static CORE: GpdmaIsrCore = GpdmaIsrCore::new();

            static mut ITEMS: [$crate::gpdma::LinearItem; $crate::gpdma::MAX_DESCRIPTORS] =
                [$crate::gpdma::zeroed_linear_item(); $crate::gpdma::MAX_DESCRIPTORS];

            /// GPDMA interrupt handler for this HUB75 instance.
            pub struct Hub75GpdmaHandler;

            impl Handler<<DmaCh as ChannelInstance>::Interrupt> for Hub75GpdmaHandler {
                unsafe fn on_interrupt() {
                    critical_section::with(|cs| {
                        let mut t = TIMER.borrow_ref_mut(cs);
                        let timer = match t.as_mut() {
                            Some(t) => t,
                            None => return,
                        };

                        timer.stop();
                        timer.reset();

                        // SAFETY: ITEMS is only mutated here (in this ISR,
                        // inside a critical section) and during init (before
                        // interrupts are enabled for this channel).
                        CORE.on_chain_complete(cs, unsafe { &mut ITEMS });

                        gpdma_driver::restart_gpdma_chain(
                            $crate::__gpdma_ch_pac!($dma_ch),
                            $crate::__gpdma_ch_num!($dma_ch),
                            unsafe { &ITEMS },
                        );

                        timer.start();
                    });
                }
            }

            /// Type alias for the GPDMA-backed HUB75 driver bound to
            /// this instance's timer.
            pub type Hub75Gpdma<'d, FB> = gpdma_driver::Hub75Gpdma<'d, $timer, FB>;

            /// Initialize the GPDMA-backed HUB75 driver, configure
            /// hardware, and start rendering from the provided
            /// framebuffer.
            pub fn init<'d, P: $crate::Hub75Pins, FB>(
                tim: Peri<'d, $timer>,
                clock_pin: Peri<'d, impl TimerPin<$timer, Ch1>>,
                dma_ch: Peri<'d, DmaCh>,
                dma_irq: impl Binding<
                        <DmaCh as ChannelInstance>::Interrupt,
                        dma::InterruptHandler<DmaCh>,
                    > + Binding<
                        <DmaCh as ChannelInstance>::Interrupt,
                        Hub75GpdmaHandler,
                    > + 'd,
                pins: P,
                config: $crate::Config,
                fb: &'static mut FB,
            ) -> Hub75Gpdma<'d, FB>
            where
                DmaCh: UpDma<$timer>,
                FB: FrameBuffer<Word = P::Word>,
            {
                gpdma_driver::Hub75Gpdma::new(
                    tim,
                    clock_pin,
                    dma_ch,
                    dma_irq,
                    pins,
                    config,
                    fb,
                    &CORE,
                    &TIMER,
                    // SAFETY: init is called once before the ISR is active.
                    unsafe { &mut ITEMS },
                    $crate::__gpdma_ch_pac!($dma_ch),
                    $crate::__gpdma_ch_num!($dma_ch),
                )
            }
        }
    };
}
