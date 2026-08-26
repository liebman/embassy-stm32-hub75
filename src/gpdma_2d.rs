//! GPDMA 2D linked-list backend for HUB75 panels.
//!
//! Uses 2D-capable GPDMA channels with hardware block-repeat to achieve
//! BCM (Binary Code Modulation) weighting. Instead of duplicating
//! descriptors (as the linear backend does), each [`BcmSegment`] exposed
//! by the framebuffer gets a single [`TwoDItem`] with `block_repeat_count`
//! set to `reps - 1`, causing the hardware to repeat the transfer `reps`
//! times total.
//!
//! This reduces the descriptor table to one item per BCM segment —
//! 8 × 32 bytes for a frame-major 8-plane layout — while producing
//! identical BCM timing, and it also supports row-major bitplane
//! layouts (up to [`MAX_SEGMENTS`] segments).
//!
//! Requires a 2D-capable GPDMA channel (enforced at compile time via
//! the [`TwoDChannelInstance`] trait bound).
//!
//! The terminal item links back to `item[0]` so the chain loops forever
//! and the GPDMA is started once. As in the linear backend, the terminal
//! node fires one TC per frame (`TR2.TCEM = EACH_LINKED_LIST_ITEM`) while
//! the others stay silent ([`IsrCore::on_frame()`]).

use core::marker::PhantomData;

use embassy_stm32::dma::linked_list::{LinkedListItem, Table};
use embassy_stm32::dma::two_d::{TwoDConfig, TwoDItem};
use embassy_stm32::dma::word::WordSize;
use embassy_stm32::dma::{
    self, Channel, ChannelInstance, Item, TransferCompleteMode, TwoDChannelInstance,
};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{Ch1, GeneralInstance4Channel, TimerPin, UpDma};
use embassy_stm32::Peri;

use crate::bcm::MAX_SEGMENTS;
use crate::framebuffer::{BcmSegment, FrameBuffer};
use crate::gpdma::{IsrCore, TimerSlot};
use crate::{Config, Hub75Error, Hub75Pins};

/// Create a zeroed `TwoDItem`, suitable for const/static init.
#[doc(hidden)]
#[must_use]
pub const fn zeroed_two_d_item() -> TwoDItem {
    use embassy_stm32::pac::gpdma::regs;
    TwoDItem {
        item: Item {
            tr1: regs::ChTr1(0),
            tr2: regs::ChTr2(0),
            br1: regs::ChBr1(0),
            sar: 0,
            dar: 0,
        },
        tr3: regs::ChTr3(0),
        br2: regs::ChBr2(0),
        llr: regs::ChLlr(0),
    }
}

// ---------------------------------------------------------------------------
// Item chain construction (2D)
// ---------------------------------------------------------------------------

/// Lower 16-bit offset address of a 2D item within its 64 KB region.
#[allow(clippy::cast_possible_truncation)]
fn item_offset(item: &TwoDItem) -> u16 {
    core::ptr::from_ref(item) as u32 as u16
}

/// Populate the 2D linked-list chain for BCM weighting.
///
/// Creates one `TwoDItem` per [`BcmSegment`], using the hardware
/// `block_repeat_count` to repeat each segment transfer `reps` times.
/// The total number of items equals the framebuffer's segment count
/// (`BCM_SEGMENT_COUNT`).
///
/// Compile-time assertion: the framebuffer's segment count must fit into
/// [`MAX_SEGMENTS`].
///
/// Returns the number of active items.
#[doc(hidden)]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn build_item_chain_2d<FB: FrameBuffer>(
    items: &mut [TwoDItem; MAX_SEGMENTS],
    fb: &FB,
    odr_addr: *mut u8,
    word_size: WordSize,
    request: dma::Request,
) -> usize {
    const {
        assert!(
            FB::BCM_SEGMENT_COUNT <= MAX_SEGMENTS,
            "framebuffer BCM segment count exceeds MAX_SEGMENTS"
        );
    }
    let segment_count = fb.bcm_segment_count();
    assert!(
        segment_count > 0 && segment_count <= MAX_SEGMENTS,
        "bcm_segment_count {segment_count} out of range 1..={MAX_SEGMENTS}"
    );

    let base = items.as_ptr() as usize;
    let end = base + core::mem::size_of::<TwoDItem>() * segment_count - 1;
    assert_eq!(
        base >> 16,
        end >> 16,
        "GPDMA 2D descriptor chain spans a 64 KB boundary"
    );

    for i in 0..segment_count {
        let BcmSegment { ptr, len, reps } = fb.bcm_segment(i);
        assert!(
            (1..=2048).contains(&reps),
            "segment {i} reps {reps} out of range 1..=2048"
        );

        let mut config = TwoDConfig::default();
        // Every node is silent (`LastLinkedListItem` — which never fires in a
        // circular chain). The terminal node instead uses `EachLinkedListItem`
        // so exactly one TC interrupt fires per frame.
        config.linear.transfer_complete_mode = if i + 1 == segment_count {
            TransferCompleteMode::EachLinkedListItem
        } else {
            TransferCompleteMode::LastLinkedListItem
        };
        config.block_repeat_count = u16::try_from(reps - 1).expect("reps range checked above");
        config.block_src_addr_offset =
            -i32::try_from(len).expect("segment length exceeds i32::MAX");

        let mut item = match word_size {
            WordSize::OneByte => {
                let buf = unsafe { core::slice::from_raw_parts(ptr, len) };
                unsafe { TwoDItem::new_write(request, buf, odr_addr, config) }
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
                    TwoDItem::new_write(request, buf, odr_addr.cast::<u16>(), config)
                }
            }
            _ => panic!("HUB75 only supports byte and halfword DMA transfers"),
        };

        // Link to the next item. The terminal item links back to item[0] so
        // the chain loops forever.
        let next_idx = if i + 1 < segment_count { i + 1 } else { 0 };
        let next_offset = item_offset(&items[next_idx]);
        item.link_to(next_offset);

        items[i] = item;
    }

    segment_count
}

// ---------------------------------------------------------------------------
// Hub75 — public driver handle
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by a 2D GPDMA linked-list refresh
/// loop.
///
/// BCM weighting is achieved via the hardware block-repeat count on each
/// `TwoDItem`. Only one descriptor per BCM segment is needed (typically
/// 6-8 for frame-major layouts), rather than one per repetition.
///
/// Created via the `hub75_define!` macro's generated `init()`
/// function. Use [`Hub75::swap()`] to double-buffer.
pub struct Hub75<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> {
    _clock_pin: PwmPin<'d, T, Ch1>,
    core: &'static IsrCore,
    items: *mut [TwoDItem; MAX_SEGMENTS],
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
    /// Create a new 2D GPDMA-backed HUB75 driver, configure hardware,
    /// and start rendering.
    ///
    /// This is called by the macro-generated `init()` wrapper.
    #[doc(hidden)]
    #[allow(clippy::too_many_arguments)]
    pub fn new<D: UpDma<T> + ChannelInstance + TwoDChannelInstance, P: Hub75Pins>(
        tim: Peri<'d, T>,
        clock_pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma_ch: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'd,
        pins: P,
        config: Config,
        fb: &'static mut FB,
        core: &'static IsrCore,
        timer_slot: &'static TimerSlot<T>,
        items: &'static mut Table<TwoDItem, { MAX_SEGMENTS }>,
    ) -> Self
    where
        FB: FrameBuffer<Word = P::Word>,
    {
        let hw = crate::setup::hardware(tim, clock_pin, pins, &config);
        let odr_addr = hw.odr_addr;

        let request = <D as UpDma<T>>::request(&*dma_ch);
        let channel = Channel::new(dma_ch, dma_irq);

        let fb_ptr = core::ptr::from_ref::<FB>(fb).cast::<()>();

        // Capture a raw pointer to the descriptor table so swap() can patch
        // SAR fields after `items` has been handed to the linked-list transfer.
        let items_ptr = core::ptr::from_mut(&mut items.items);

        let descriptor_count =
            build_item_chain_2d(&mut items.items, fb, odr_addr, P::DMA_WORD_SIZE, request);

        let options = crate::gpdma::gpdma_transfer_options();

        critical_section::with(|cs| {
            // SAFETY: Timer<'d> → Timer<'static>. See clock.rs for rationale.
            unsafe { crate::setup::store_timer(hw.timer, timer_slot, cs) };

            let mut channel: Channel<'static> =
                unsafe { core::mem::transmute::<Channel<'_>, Channel<'static>>(channel) };

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
                crate::gpdma::apply_item_delta_2d(&mut *self.items, self.descriptor_count, delta);
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

/// Define a 2D GPDMA-backed HUB75 driver instance with its own timer
/// static, descriptor table, and ISR handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75DmaHandler` — the GPDMA interrupt handler for `bind_interrupts!`
/// - `Hub75<'d, FB>` — a type alias for the driver
/// - `init()` — constructs and starts the driver
///
/// BCM weighting is achieved via the 2D block-repeat feature. Only one
/// `TwoDItem` per bitplane is needed. Requires a 2D-capable GPDMA
/// channel (compile-time enforced via `TwoDChannelInstance` trait bound).
///
/// The descriptor chain is circular: the GPDMA is started once and loops
/// forever, and the terminal linked-list node fires one TC interrupt per
/// frame boundary. The pixel-clock timer free-runs — there is no per-frame
/// stop/reset/restart.
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type
/// - `$dma_ch` — the GPDMA channel peripheral type (must be 2D-capable)
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_define;
///
/// hub75_define!(hub75, peripherals::TIM2, peripherals::GPDMA1_CH4);
///
/// bind_interrupts!(struct Irqs {
///     GPDMA1_CHANNEL4 =>
///         dma::InterruptHandler<peripherals::GPDMA1_CH4>,
///         hub75::Hub75DmaHandler;
/// });
///
/// let hub75 = hub75::init(
///     p.TIM2, p.PA0, p.GPDMA1_CH4, Irqs, pins,
///     Config::new().frequency(Hertz(10_000_000)),
///     fb0,
/// );
/// ```
#[cfg(feature = "gpdma-2d")]
#[macro_export]
macro_rules! hub75_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ty) => {
        #[allow(non_snake_case)]
        pub mod $mod_name {
            use $crate::__macro_support::critical_section;
            use $crate::__macro_support::embassy_stm32::dma::{
                self, ChannelInstance, TwoDChannelInstance,
            };
            use $crate::__macro_support::embassy_stm32::dma::linked_list::Table;
            use $crate::__macro_support::embassy_stm32::dma::two_d::TwoDItem;
            use $crate::__macro_support::embassy_stm32::interrupt::typelevel::{Binding, Handler};
            use $crate::__macro_support::embassy_stm32::timer::{Ch1, TimerPin, UpDma};
            use $crate::__macro_support::embassy_stm32::Peri;
            use $crate::framebuffer::FrameBuffer;
            use $crate::gpdma::{IsrCore, TimerSlot, MAX_SEGMENTS};
            use $crate::gpdma_2d::{self as gpdma_2d_driver};

            static TIMER: TimerSlot<$timer> =
                critical_section::Mutex::new(core::cell::RefCell::new(None));

            static CORE: IsrCore = IsrCore::new();

            static mut ITEMS: Table<TwoDItem, { MAX_SEGMENTS }> = Table {
                items: [$crate::gpdma_2d::zeroed_two_d_item(); MAX_SEGMENTS],
            };

            /// GPDMA interrupt handler for this 2D HUB75 instance.
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

            /// Type alias for the 2D GPDMA-backed HUB75 driver bound to
            /// this instance's timer.
            pub type Hub75<'d, FB> = gpdma_2d_driver::Hub75<'d, $timer, FB>;

            /// Initialize the 2D GPDMA-backed HUB75 driver, configure
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
                $dma_ch: UpDma<$timer> + TwoDChannelInstance,
                FB: FrameBuffer<Word = P::Word>,
            {
                gpdma_2d_driver::Hub75::new(
                    tim,
                    clock_pin,
                    dma_ch,
                    dma_irq,
                    pins,
                    config,
                    fb,
                    &CORE,
                    &TIMER,
                    unsafe { &mut ITEMS },
                )
            }
        }
    };
}
