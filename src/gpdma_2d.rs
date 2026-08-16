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
use crate::gpdma::{GpdmaIsrCore, TimerSlot};
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
        debug_assert!(
            {
                let (shape_len, shape_reps) = FB::BCM_SEGMENT_SHAPES[i % FB::BCM_SEQUENCE_LEN];
                len == shape_len && reps == shape_reps
            },
            "bcm_segment({i}) disagrees with BCM_SEGMENT_SHAPES {:?}",
            FB::BCM_SEGMENT_SHAPES[i % FB::BCM_SEQUENCE_LEN],
        );
        assert!(
            (1..=2048).contains(&reps),
            "segment {i} reps {reps} out of range 1..=2048"
        );

        let mut config = TwoDConfig::default();
        config.linear.transfer_complete_mode = TransferCompleteMode::LastLinkedListItem;
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

        if i + 1 < segment_count {
            let next_offset = item_offset(&items[i + 1]);
            item.link_to(next_offset);
        }

        items[i] = item;
    }

    segment_count
}

// ---------------------------------------------------------------------------
// Hub75Gpdma2d — public driver handle
// ---------------------------------------------------------------------------

/// HUB75 LED matrix controller driven by a 2D GPDMA linked-list refresh
/// loop.
///
/// BCM weighting is achieved via the hardware block-repeat count on each
/// `TwoDItem`. Only one descriptor per BCM segment is needed (typically
/// 6-8 for frame-major layouts), rather than one per repetition.
///
/// Created via the `hub75_gpdma_2d_define!` macro's generated `init()`
/// function. Use [`Hub75Gpdma2d::swap()`] to double-buffer.
pub struct Hub75Gpdma2d<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> {
    _clock_pin: PwmPin<'d, T, Ch1>,
    core: &'static GpdmaIsrCore,
    _fb: PhantomData<&'static FB>,
}

impl<'d, T: GeneralInstance4Channel, FB: FrameBuffer + 'static> Hub75Gpdma2d<'d, T, FB> {
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
        core: &'static GpdmaIsrCore,
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

            core.init_state(cs, channel, options, descriptor_count, fb_ptr);

            timer_slot.borrow_ref(cs).as_ref().unwrap().start();
        });

        Self {
            _clock_pin: hw.clock_pin,
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
    /// frame boundary, at which point all descriptor source addresses
    /// are shifted to the new framebuffer atomically. Returns an
    /// exclusive reference to the old framebuffer that is no longer
    /// being read by the GPDMA.
    ///
    /// # Errors
    /// Returns `Hub75Error::NotInitialised` if the driver has not
    /// been initialised.
    pub async fn swap(&mut self, new_fb: &'static mut FB) -> Result<&'static mut FB, Hub75Error> {
        let fb_ptr = core::ptr::from_ref::<FB>(new_fb).cast::<()>();
        let old_ptr = unsafe { self.core.swap_inner(fb_ptr).await? };
        Ok(unsafe { &mut *(old_ptr as *mut FB) })
    }
}

// ---------------------------------------------------------------------------
// hub75_gpdma_2d_define! macro
// ---------------------------------------------------------------------------

/// Define a 2D GPDMA-backed HUB75 driver instance with its own timer
/// static, descriptor table, and ISR handler.
///
/// Each invocation creates a public module containing:
/// - `Hub75Gpdma2dHandler` — the GPDMA interrupt handler for `bind_interrupts!`
/// - `Hub75Gpdma2d<'d, FB>` — a type alias for the driver
/// - `init()` — constructs and starts the driver
///
/// BCM weighting is achieved via the 2D block-repeat feature. Only one
/// `TwoDItem` per bitplane is needed. Requires a 2D-capable GPDMA
/// channel (compile-time enforced via `TwoDChannelInstance` trait bound).
///
/// # Parameters
/// - `$mod_name` — name of the generated module
/// - `$timer` — the concrete timer peripheral type
/// - `$dma_ch` — the GPDMA channel peripheral name (must be 2D-capable)
///
/// # Example
/// ```ignore
/// use embassy_stm32::{bind_interrupts, dma, peripherals};
/// use embassy_stm32_hub75::hub75_gpdma_2d_define;
///
/// hub75_gpdma_2d_define!(hub75, peripherals::TIM2, GPDMA1_CH4);
///
/// bind_interrupts!(struct Irqs {
///     GPDMA1_CHANNEL4 =>
///         dma::InterruptHandler<peripherals::GPDMA1_CH4>,
///         hub75::Hub75Gpdma2dHandler;
/// });
///
/// let hub75 = hub75::init(
///     p.TIM2, p.PA0, p.GPDMA1_CH4, Irqs, pins,
///     Config::new().frequency(Hertz(10_000_000)),
///     fb0,
/// );
/// ```
#[macro_export]
macro_rules! hub75_gpdma_2d_define {
    ($mod_name:ident, $timer:ty, $dma_ch:ident) => {
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
            use $crate::gpdma::{GpdmaIsrCore, TimerSlot, MAX_SEGMENTS};
            use $crate::gpdma_2d::{self as gpdma_2d_driver};

            type DmaCh = $crate::__macro_support::embassy_stm32::peripherals::$dma_ch;

            static TIMER: TimerSlot<$timer> =
                critical_section::Mutex::new(core::cell::RefCell::new(None));

            static CORE: GpdmaIsrCore = GpdmaIsrCore::new();

            static mut ITEMS: Table<TwoDItem, { MAX_SEGMENTS }> = Table {
                items: [$crate::gpdma_2d::zeroed_two_d_item(); MAX_SEGMENTS],
            };

            /// GPDMA interrupt handler for this 2D HUB75 instance.
            pub struct Hub75Gpdma2dHandler;

            impl Handler<<DmaCh as ChannelInstance>::Interrupt> for Hub75Gpdma2dHandler {
                unsafe fn on_interrupt() {
                    critical_section::with(|cs| {
                        let mut t = TIMER.borrow_ref_mut(cs);
                        let timer = match t.as_mut() {
                            Some(t) => t,
                            None => return,
                        };

                        timer.stop();
                        timer.reset();

                        CORE.on_chain_complete_2d(cs, unsafe { &mut ITEMS.items });

                        CORE.restart_chain_2d(cs, unsafe { &ITEMS });

                        timer.start();
                    });
                }
            }

            /// Type alias for the 2D GPDMA-backed HUB75 driver bound to
            /// this instance's timer.
            pub type Hub75Gpdma2d<'d, FB> = gpdma_2d_driver::Hub75Gpdma2d<'d, $timer, FB>;

            /// Initialize the 2D GPDMA-backed HUB75 driver, configure
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
                        Hub75Gpdma2dHandler,
                    > + 'd,
                pins: P,
                config: $crate::Config,
                fb: &'static mut FB,
            ) -> Hub75Gpdma2d<'d, FB>
            where
                DmaCh: UpDma<$timer> + TwoDChannelInstance,
                FB: FrameBuffer<Word = P::Word>,
            {
                gpdma_2d_driver::Hub75Gpdma2d::new(
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
