//! GPDMA linked-list backend for HUB75 panels.
//!
//! Achieves BCM (Binary Code Modulation) weighting by duplicating
//! [`LinearItem`] descriptors in a single linked-list chain, matching
//! the `full-chain-dma` pattern from `esp-hub75`. Each [`BcmSegment`]
//! exposed by the framebuffer gets `reps` consecutive descriptors all
//! pointing at the same segment data.
//!
//! With `TR2.TCEM = LAST_LINKED_LIST_ITEM`, the GPDMA walks the
//! entire chain autonomously and fires **one** TC interrupt at the
//! terminal descriptor — one ISR per frame.
//!
//! Works with **any** GPDMA channel (no 2D capability required).
//!
//! The descriptor chain is circular: the terminal item links back to
//! `item[0]` so the chain loops forever and the GPDMA is started exactly
//! once — no per-frame restart. Because STM32 GPDMA has no orthogonal
//! "end-of-loop" flag (unlike ESP32's `suc_eof`), the terminal node uses
//! `TR2.TCEM = EACH_LINKED_LIST_ITEM` (firing one TC interrupt per frame)
//! while every other node is silent (`LAST_LINKED_LIST_ITEM`, which never
//! fires in a circular chain). The timer free-runs; `swap()` shifts every
//! descriptor source address directly (in a critical section) and then
//! waits for the configured number of transfer-complete interrupts before
//! returning the old framebuffer ([`IsrCore::on_frame()`]). By default it
//! waits for two boundaries, guaranteeing the old framebuffer is fully
//! drained; the `unsafe-swap-wait-1` / `unsafe-swap-wait-0` features
//! shorten that wait at the cost of possible visual tearing.

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::task::{Poll, Waker};

use critical_section::Mutex;
#[doc(hidden)]
pub use embassy_stm32::dma::linked_list::LinearItem;
#[doc(hidden)]
pub use embassy_stm32::dma::linked_list::LinearItemConfig;
#[doc(hidden)]
pub use embassy_stm32::dma::linked_list::LinkedListItem;
#[doc(hidden)]
pub use embassy_stm32::dma::two_d::TwoDItem;
use embassy_stm32::dma::word::WordSize;
use embassy_stm32::dma::{
    self, Channel, ChannelInstance, Item, Priority, Table, TransferCompleteMode, TransferOptions,
};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac::gpdma::regs;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

#[doc(hidden)]
pub use crate::bcm::MAX_SEGMENTS;
use crate::framebuffer::{BcmSegment, FrameBuffer};
use crate::{Config, Hub75Error, Hub75Pins};

/// Re-export of [`crate::setup::TimerSlot`] for the `hub75_define!` macro.
#[doc(hidden)]
pub use crate::setup::TimerSlot;

/// Maximum number of `LinearItem` descriptors in a full BCM chain.
///
/// Equals `2^8 - 1` = 255: a frame-major bitplane framebuffer with
/// `PLANES` planes needs `2^(PLANES-1)` descriptors (8 planes = 128).
/// Row-major layouts need `NROWS * 2^(PLANES-1)` descriptors and are
/// rejected at compile time when they exceed this limit.
pub const MAX_DESCRIPTORS: usize = (1 << 8) - 1;

/// Create a zeroed `LinearItem`, suitable for const/static init.
#[doc(hidden)]
#[must_use]
pub const fn zeroed_linear_item() -> LinearItem {
    LinearItem {
        item: Item {
            tr1: regs::ChTr1(0),
            tr2: regs::ChTr2(0),
            br1: regs::ChBr1(0),
            sar: 0,
            dar: 0,
        },
        llr: regs::ChLlr(0),
    }
}

/// Number of `LinearItem` descriptors needed to stream `FB`'s BCM
/// segment sequence: the sum of the repetition counts of every segment
/// in one complete panel refresh.
///
/// Equals `2^(PLANES-1)` for frame-major bitplane framebuffers.
const fn descriptor_count<FB: FrameBuffer>() -> usize {
    crate::framebuffer::bcm_rep_count::<FB>()
}

// ---------------------------------------------------------------------------
// Item chain construction
// ---------------------------------------------------------------------------

/// Lower 16-bit offset address of an item within its 64 KB region.
#[allow(clippy::cast_possible_truncation)]
fn item_offset(item: &LinearItem) -> u16 {
    core::ptr::from_ref(item) as u32 as u16
}

/// Populate the linked-list chain with duplicated descriptors for
/// BCM weighting, from the framebuffer's [`BcmSegment`] sequence.
///
/// For each segment, creates `reps` consecutive `LinearItem` entries
/// pointing at the same segment data. The terminal item links back to
/// `item[0]` (making the chain circular) and uses `TCEM = EACH_LLI` to
/// fire one TC interrupt per frame boundary.
///
/// Compile-time assertion: the descriptor count computed from
/// [`FrameBuffer::BCM_SEQUENCE`] must fit into [`MAX_DESCRIPTORS`].
///
/// Returns the number of active items in the chain.
#[doc(hidden)]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn build_item_chain<FB: FrameBuffer>(
    items: &mut [LinearItem; MAX_DESCRIPTORS],
    fb: &FB,
    odr_addr: *mut u8,
    word_size: WordSize,
    request: dma::Request,
) -> usize {
    const {
        assert!(
            descriptor_count::<FB>() <= MAX_DESCRIPTORS,
            "framebuffer BCM sequence needs more descriptors than MAX_DESCRIPTORS"
        );
    }
    let segment_count = fb.bcm_segment_count();
    assert!(
        segment_count > 0 && segment_count <= MAX_SEGMENTS,
        "bcm_segment_count {segment_count} out of range 1..={MAX_SEGMENTS}"
    );

    let mut total = 0;
    for i in 0..segment_count {
        let segment = fb.bcm_segment(i);
        total += segment.reps;
    }
    assert!(total <= MAX_DESCRIPTORS);

    let base = items.as_ptr() as usize;
    let end = base + core::mem::size_of::<LinearItem>() * total - 1;
    assert_eq!(
        base >> 16,
        end >> 16,
        "GPDMA descriptor chain spans a 64 KB boundary"
    );

    // Every node is silent (`LastLinkedListItem` — which never fires in a
    // circular chain, because no node is ever "the last"). The terminal
    // node instead uses `EachLinkedListItem` so exactly one TC interrupt
    // fires per frame at the frame boundary.
    let mut config = LinearItemConfig::default();
    config.transfer_complete_mode = TransferCompleteMode::LastLinkedListItem;
    let mut terminal_config = config;
    terminal_config.transfer_complete_mode = TransferCompleteMode::EachLinkedListItem;

    let mut idx = 0;
    for i in 0..segment_count {
        let BcmSegment { ptr, len, reps } = fb.bcm_segment(i);

        for _ in 0..reps {
            let is_last = idx + 1 == total;
            let item_config = if is_last { terminal_config } else { config };

            // Create a write transfer item (memory → peripheral/ODR)
            // using the embassy-stm32 high-level API.
            let mut item = match word_size {
                WordSize::OneByte => {
                    let buf = unsafe { core::slice::from_raw_parts(ptr, len) };
                    unsafe { LinearItem::new_write(request, buf, odr_addr, item_config) }
                }
                WordSize::TwoBytes => {
                    debug_assert!(
                        ptr.align_offset(core::mem::align_of::<u16>()) == 0,
                        "DMA source buffer is not u16-aligned"
                    );
                    debug_assert!(
                        len.is_multiple_of(2),
                        "DMA buffer length is not a multiple of 2"
                    );
                    #[allow(clippy::cast_ptr_alignment)]
                    let buf = unsafe { core::slice::from_raw_parts(ptr.cast::<u16>(), len / 2) };
                    #[allow(clippy::cast_ptr_alignment)]
                    unsafe {
                        LinearItem::new_write(request, buf, odr_addr.cast::<u16>(), item_config)
                    }
                }
                _ => panic!("HUB75 only supports byte and halfword DMA transfers"),
            };

            // Link to the next item. The terminal item links back to
            // item[0] so the chain loops forever.
            let next_idx = if is_last { 0 } else { idx + 1 };
            let next_offset = item_offset(&items[next_idx]);
            item.link_to(next_offset);

            items[idx] = item;
            idx += 1;
        }
    }

    total
}

/// Shift the source-address fields of all active descriptors by
/// `delta` bytes.
///
/// Called at a framebuffer swap: the old and new framebuffers have
/// the same `FB` type and therefore identical layout, so every
/// descriptor's `sar` moves by the same byte offset.
#[doc(hidden)]
pub fn apply_item_delta(items: &mut [LinearItem; MAX_DESCRIPTORS], count: usize, delta: isize) {
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    for item in items.iter_mut().take(count) {
        item.item.sar = item.item.sar.wrapping_add(delta as u32);
    }
}

/// Shift the source-address fields of all active 2D descriptors by
/// `delta` bytes (one item per BCM segment).
#[doc(hidden)]
pub fn apply_item_delta_2d(items: &mut [TwoDItem; MAX_SEGMENTS], count: usize, delta: isize) {
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    for item in items.iter_mut().take(count) {
        item.item.sar = item.item.sar.wrapping_add(delta as u32);
    }
}

/// Default transfer options for GPDMA linked-list HUB75 transfers.
///
/// Shared by the linear and 2D GPDMA backends.
#[doc(hidden)]
pub(crate) fn gpdma_transfer_options() -> TransferOptions {
    let mut options = TransferOptions::default();
    options.priority = Priority::VeryHigh;
    options.complete_transfer_ir = true;
    options.transfer_complete_mode = TransferCompleteMode::LastLinkedListItem;
    options
}

// ---------------------------------------------------------------------------
// IsrCore — frame-boundary state (library code, no generics)
// ---------------------------------------------------------------------------

/// Number of frame-boundary transfer-complete interrupts `swap()` waits for
/// after applying the descriptor delta, selected by feature. The default is
/// two; `unsafe-swap-wait-1` / `unsafe-swap-wait-0` shorten it to one / zero.
#[cfg(feature = "unsafe-swap-wait-0")]
const SWAP_WAIT_COUNT: u8 = 0;
#[cfg(all(feature = "unsafe-swap-wait-1", not(feature = "unsafe-swap-wait-0")))]
const SWAP_WAIT_COUNT: u8 = 1;
#[cfg(not(any(feature = "unsafe-swap-wait-0", feature = "unsafe-swap-wait-1")))]
const SWAP_WAIT_COUNT: u8 = 2;

/// Per-instance ISR core state for the GPDMA backend, shared between
/// the ISR handler and the [`Hub75`] driver. Created by
/// `hub75_define!` as a `static`.
#[doc(hidden)]
pub struct IsrCore {
    swap_waits: Mutex<RefCell<u8>>,
    swap_done: AtomicBool,
    swap_waker: Mutex<RefCell<Option<Waker>>>,
    frame_count: AtomicU32,
}

impl IsrCore {
    /// Create a new core. For use in `static` declarations.
    #[must_use]
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            swap_waits: Mutex::new(RefCell::new(0)),
            swap_done: AtomicBool::new(false),
            swap_waker: Mutex::new(RefCell::new(None)),
            frame_count: AtomicU32::new(0),
        }
    }

    /// Called from the ISR at each frame boundary.
    ///
    /// The chain raises exactly one TC interrupt per frame (on the terminal
    /// node). Framebuffer deltas are applied directly in [`Hub75::swap`]
    /// rather than here, so this handler only counts the frame boundary and,
    /// if a swap is waiting, decrements the remaining-waits counter and
    /// signals completion once it reaches zero.
    #[doc(hidden)]
    pub fn on_frame(&self, cs: critical_section::CriticalSection) {
        self.frame_count.fetch_add(1, Ordering::Relaxed);

        let mut swap_waits = self.swap_waits.borrow_ref_mut(cs);
        if *swap_waits > 0 {
            *swap_waits -= 1;
            if *swap_waits == 0 {
                self.signal_swap_done(cs);
            }
        }
    }

    /// Returns the number of complete BCM frames rendered.
    pub fn frame_count(&self) -> u32 {
        self.frame_count.load(Ordering::Relaxed)
    }

    /// Arm a swap wait and yield until the configured number of
    /// frame-boundary interrupts have been observed. Called from
    /// [`Hub75::swap`] after the descriptor delta has been applied.
    #[doc(hidden)]
    pub async fn wait_swap(&self) {
        if SWAP_WAIT_COUNT != 0 {
            critical_section::with(|cs| {
                *self.swap_waits.borrow_ref_mut(cs) = SWAP_WAIT_COUNT;
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
        }
    }

    fn signal_swap_done(&self, cs: critical_section::CriticalSection) {
        self.swap_done.store(true, Ordering::Release);
        if let Some(waker) = self.swap_waker.borrow_ref_mut(cs).take() {
            waker.wake();
        }
    }
}

// ---------------------------------------------------------------------------
// Hub75 — public driver handle (library code, generic over T and FB)
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by a GPDMA linked-list refresh
/// loop.
///
/// BCM weighting is achieved by duplicating `LinearItem` descriptors
/// (one per BCM segment repetition) in a single chain. The GPDMA
/// traverses the chain autonomously; one ISR fires per complete
/// BCM frame.
///
/// Created via the `hub75_define!` macro's generated `init()`
/// function. Use [`Hub75::swap()`] to double-buffer.
pub struct Hub75<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> {
    _clock_pin: PwmPin<'d, T, Ch1>,
    core: &'static IsrCore,
    items: *mut [LinearItem; MAX_DESCRIPTORS],
    descriptor_count: usize,
    current_fb_ptr: *const (),
    _fb: PhantomData<&'static FB>,
}

// SAFETY: `items` points at this instance's `static` descriptor table and
// `current_fb_ptr` points at a `'static` framebuffer — both outlive the
// driver handle. `FB: Sync` preserves the bound previously implied by
// `PhantomData<&'static FB>`.
unsafe impl<T: GeneralInstance4Channel, FB: FrameBuffer + 'static + Sync> Send
    for Hub75<'_, T, FB>
{
}

impl<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> Hub75<'d, T, FB> {
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
        dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pins: P,
        config: Config,
        fb: &'static mut FB,
        core: &'static IsrCore,
        timer_slot: &'static TimerSlot<T>,
        items: &'static mut Table<LinearItem, MAX_DESCRIPTORS>,
    ) -> Self
    where
        FB: FrameBuffer<Word = P::Word>,
    {
        let hw = crate::setup::hardware(tim, clock_pin, pins, &config);
        let odr_addr = hw.odr_addr;

        let request = <D as UpDma<T>>::request(&*dma_ch);
        let channel = Channel::new(dma_ch, dma_irq);

        // --- Build the linked-list descriptor chain ---
        let fb_ptr = core::ptr::from_ref::<FB>(fb).cast::<()>();

        // Capture a raw pointer to the descriptor table so swap() can patch
        // SAR fields after `items` has been handed to the linked-list transfer.
        let items_ptr = core::ptr::from_mut(&mut items.items);

        let descriptor_count =
            build_item_chain(&mut items.items, fb, odr_addr, P::DMA_WORD_SIZE, request);

        // Transfer options for linked-list configuration.
        // TCEM on individual items is set in build_item_chain; the
        // transfer_complete_mode here only affects the initial channel
        // TR2 which is overwritten by the first LLI.
        let options = gpdma_transfer_options();

        critical_section::with(|cs| {
            // SAFETY: Timer<'d> → Timer<'static>. See setup.rs for rationale.
            unsafe { crate::setup::store_timer(hw.timer, timer_slot, cs) };

            // SAFETY: Channel<'d> → Channel<'static>. The DMA channel
            // peripheral is consumed by this driver and will never be
            // released. Channel internally stores a DmaChannel enum +
            // PhantomData lifetime marker — no actual borrow. Same
            // embassy-stm32 version coupling as the Timer transmute above.
            let mut channel: Channel<'static> =
                unsafe { core::mem::transmute::<Channel<'_>, Channel<'static>>(channel) };

            // Start the linked-list transfer. The returned LinkedListTransfer
            // is forgotten to prevent its Drop impl from resetting the channel.
            let transfer = unsafe { channel.linked_list(items, options) };
            core::mem::forget(transfer);

            timer_slot.borrow_ref(cs).as_ref().unwrap().start();
        });

        Self {
            _clock_pin: hw.clock_pin,
            core,
            items: items_ptr,
            descriptor_count,
            current_fb_ptr: fb_ptr,
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
    /// Immediately shifts every descriptor source address to `new_fb`, then
    /// yields until the configured number of transfer-complete interrupts
    /// have been observed before returning the old framebuffer. By default it
    /// waits for two boundaries, after which the old framebuffer is
    /// guaranteed to no longer be read by the GPDMA; the
    /// `unsafe-swap-wait-1` / `unsafe-swap-wait-0` features shorten the wait
    /// (see the crate-level feature docs).
    ///
    /// # Errors
    ///
    /// This method is infallible for the GPDMA backends (a [`Hub75`] handle
    /// only exists after initialisation); the `Result` return is kept for API
    /// consistency with the plain-DMA backend.
    pub async fn swap(&mut self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
        let new_fb_ptr = core::ptr::from_ref::<FB>(new_fb).cast::<()>();

        // The old and new framebuffers have the same `FB` type and therefore
        // identical layout, so every descriptor's source address shifts by the
        // same byte delta.
        let delta = new_fb_ptr as isize - self.current_fb_ptr as isize;
        let old_ptr = self.current_fb_ptr;
        self.current_fb_ptr = new_fb_ptr;

        if delta != 0 {
            // SAFETY: `self.items` points at this instance's `static`
            // descriptor table, captured before the table was handed to the
            // forgotten linked-list transfer. `swap` holds `&mut self`, so
            // this is the only CPU-side writer; the sole concurrent reader is
            // the GPDMA, which reads a descriptor's SAR as it loads that item
            // — a single-word store cannot race a torn read.
            unsafe {
                apply_item_delta(&mut *self.items, self.descriptor_count, delta);
            }
        }

        self.core.wait_swap().await;

        // SAFETY: `old_ptr` is the previously-displayed `&'static mut FB`,
        // handed back exclusively and no longer read by the GPDMA.
        Ok(unsafe { &mut *(old_ptr as *mut FB) })
    }
}

// ---------------------------------------------------------------------------
// hub75_define! macro
// ---------------------------------------------------------------------------

/// Define a GPDMA-backed HUB75 driver instance with its own timer
/// static, descriptor table, and ISR handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75DmaHandler` — the GPDMA interrupt handler for `bind_interrupts!`
/// - `Hub75<'d, FB>` — a type alias for the driver
/// - `init()` — constructs and starts the driver
///
/// BCM weighting is achieved by duplicating `LinearItem` descriptors
/// in the chain. Any GPDMA channel works (2D capability not required).
///
/// The descriptor chain is circular: the GPDMA is started once and loops
/// forever, and the terminal linked-list node fires one TC interrupt per
/// frame boundary. The pixel-clock timer free-runs — there is no per-frame
/// stop/reset/restart.
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type
/// - `$dma_ch` — the GPDMA channel peripheral type (e.g. `peripherals::GPDMA1_CH0`)
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_define;
///
/// hub75_define!(hub75, peripherals::TIM2, peripherals::GPDMA1_CH0);
///
/// bind_interrupts!(struct Irqs {
///     GPDMA1_CHANNEL0 =>
///         dma::InterruptHandler<peripherals::GPDMA1_CH0>,
///         hub75::Hub75DmaHandler;
/// });
///
/// let hub75 = hub75::init(
///     p.TIM2, p.PA0, p.GPDMA1_CH0, Irqs, pins,
///     Config::new().frequency(Hertz(10_000_000)),
///     fb0,
/// );
/// ```
#[cfg(all(feature = "gpdma", not(feature = "gpdma-2d")))]
#[macro_export]
macro_rules! hub75_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ty) => {
        #[allow(non_snake_case)]
        pub mod $mod_name {
            use $crate::__macro_support::critical_section;
            use $crate::__macro_support::embassy_stm32::dma::{self, ChannelInstance, Table};
            use $crate::__macro_support::embassy_stm32::dma::linked_list::LinearItem;
            use $crate::__macro_support::embassy_stm32::interrupt::typelevel::{Binding, Handler};
            use $crate::__macro_support::embassy_stm32::timer::{Ch1, TimerPin, UpDma};
            use $crate::__macro_support::embassy_stm32::Peri;
            use $crate::framebuffer::FrameBuffer;
            use $crate::gpdma::{self as gpdma_driver, IsrCore, TimerSlot};

            static TIMER: TimerSlot<$timer> =
                critical_section::Mutex::new(core::cell::RefCell::new(None));

            static CORE: IsrCore = IsrCore::new();

            static mut ITEMS: Table<LinearItem, { $crate::gpdma::MAX_DESCRIPTORS }> = Table {
                items: [$crate::gpdma::zeroed_linear_item(); $crate::gpdma::MAX_DESCRIPTORS],
            };

            /// GPDMA interrupt handler for this HUB75 instance.
            pub struct Hub75DmaHandler;

            impl Handler<<$dma_ch as ChannelInstance>::Interrupt> for Hub75DmaHandler {
                unsafe fn on_interrupt() {
                    critical_section::with(|cs| {
                        // Guard: only act once init() has stored the timer.
                        if TIMER.borrow_ref(cs).is_none() {
                            return;
                        }

                        // The chain loops forever, so there is no timer/DMA
                        // restart — a single TC on the terminal node drives
                        // the frame boundary. Framebuffer deltas are applied
                        // in swap() (not here), so the ISR only counts the
                        // boundary.
                        CORE.on_frame(cs);
                    });
                }
            }

            /// Type alias for the GPDMA-backed HUB75 driver bound to
            /// this instance's timer.
            pub type Hub75<'d, FB> = gpdma_driver::Hub75<'d, $timer, FB>;

            /// Initialize the GPDMA-backed HUB75 driver, configure
            /// hardware, and start rendering from the provided
            /// framebuffer.
            pub fn init<'d, P: $crate::Hub75Pins, FB>(
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
                pins: P,
                config: $crate::Config,
                fb: &'static mut FB,
            ) -> Hub75<'d, FB>
            where
                $dma_ch: UpDma<$timer>,
                FB: FrameBuffer<Word = P::Word>,
            {
                gpdma_driver::Hub75::new(
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
                )
            }
        }
    };
}
