//! BCM (Binary Code Modulation) state machine for ISR-driven HUB75 refresh.
//!
//! Caches the [`BcmSegment`] scan sequence exposed by the framebuffer (see
//! [`FrameBuffer`]) and tracks which segment and repetition the DMA is
//! currently outputting, advancing through the BCM weighting sequence on
//! each transfer-complete interrupt.
//!
//! Framebuffer swaps use the pointer-delta technique from `esp-hub75`: both
//! framebuffers have the same type and therefore identical layout, so at a
//! frame boundary the ISR only adds the byte delta between the old and the
//! new framebuffer to every cached segment pointer.

use core::ptr::null;

use crate::framebuffer::BcmSegment;
use crate::framebuffer::FrameBuffer;

/// Maximum number of BCM segments that can be cached for ISR use.
///
/// Sized for the worst-case row-major layout: 32 row-pairs x (8 planes + 1
/// inter-row gap + 1 end-of-row trailer) = 320 segments.
#[doc(hidden)]
pub const MAX_SEGMENTS: usize = 320;

/// Empty segment used for const/static initialisation of the cache.
const EMPTY_SEGMENT: BcmSegment = BcmSegment {
    ptr: null(),
    len: 0,
    reps: 0,
};

/// Cached BCM segment sequence for ISR use.
///
/// Stores the full segment sequence extracted from a [`FrameBuffer`] so the
/// ISR can drive DMA without calling trait methods (the framebuffer type is
/// erased in the ISR statics).
#[doc(hidden)]
pub struct SegmentCache {
    /// Segment storage; only the first `count` entries are valid.
    pub segments: [BcmSegment; MAX_SEGMENTS],
    /// Number of valid entries in `segments`.
    pub count: usize,
    /// Consecutive segments that form one DMA transfer group.
    ///
    /// The STM32 backends synchronise per segment and do not batch groups;
    /// this is retained for parity with the `esp-hub75` cache layout.
    pub segments_per_group: usize,
}

impl Default for SegmentCache {
    fn default() -> Self {
        Self::new()
    }
}

impl SegmentCache {
    /// Create an empty segment cache.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            segments: [EMPTY_SEGMENT; MAX_SEGMENTS],
            count: 0,
            segments_per_group: 1,
        }
    }

    /// Shift every cached segment pointer by `delta` bytes.
    ///
    /// Used for framebuffer swaps: the old and new framebuffers have the
    /// same `FB` type and therefore identical layout, so all segment
    /// pointers move by the same byte offset.
    pub fn apply_delta(&mut self, delta: isize) {
        for segment in self.segments.iter_mut().take(self.count) {
            segment.ptr = segment.ptr.wrapping_byte_offset(delta);
        }
    }
}

/// Extract the BCM segment sequence from a framebuffer into `cache`.
///
/// Compile-time assertion: the framebuffer's static segment count must fit
/// into [`MAX_SEGMENTS`] (evaluated per monomorphisation).
#[doc(hidden)]
pub fn segments_from_fb_into<FB: FrameBuffer>(fb: &FB, cache: &mut SegmentCache) {
    const {
        assert!(
            FB::BCM_SEGMENT_COUNT <= MAX_SEGMENTS,
            "framebuffer BCM segment count exceeds MAX_SEGMENTS"
        );
    }
    let count = fb.bcm_segment_count();
    assert!(
        count > 0 && count <= MAX_SEGMENTS,
        "bcm_segment_count {count} out of range 1..={MAX_SEGMENTS}"
    );
    for (i, slot) in cache.segments.iter_mut().enumerate().take(count) {
        let segment = fb.bcm_segment(i);
        debug_assert!(
            !segment.ptr.is_null(),
            "segment {i} returned a null pointer"
        );
        *slot = segment;
    }
    cache.count = count;
    cache.segments_per_group = fb.bcm_segments_per_group();
}

/// ISR-driven BCM state machine.
///
/// Walks the cached segment sequence in order, streaming each segment
/// `reps` times before advancing to the next — the BCM weighting is
/// entirely described by the framebuffer's segment sequence.
#[doc(hidden)]
pub struct BcmState {
    cache: SegmentCache,
    current_segment: usize,
    current_rep: usize,
}

impl BcmState {
    #[must_use]
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            cache: SegmentCache::new(),
            current_segment: 0,
            current_rep: 0,
        }
    }

    /// Load the segment sequence from `fb`, restarting the scan from
    /// segment 0.
    pub fn load<FB: FrameBuffer>(&mut self, fb: &FB) {
        segments_from_fb_into(fb, &mut self.cache);
        self.current_segment = 0;
        self.current_rep = 0;
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full frame boundary is reached (all segments
    /// with all repetitions have been output).
    pub fn advance(&mut self) -> bool {
        debug_assert!(
            self.cache.count > 0,
            "BcmState::advance called before initialization"
        );
        self.current_rep += 1;
        if self.current_rep >= self.cache.segments[self.current_segment].reps {
            self.current_rep = 0;
            self.current_segment += 1;
            if self.current_segment >= self.cache.count {
                self.current_segment = 0;
                return true;
            }
        }
        false
    }

    /// Returns the (`pointer`, `byte_length`) for the current segment's
    /// DMA data.
    #[must_use]
    pub fn current_segment(&self) -> (*const u8, usize) {
        let segment = &self.cache.segments[self.current_segment];
        (segment.ptr, segment.len)
    }

    /// Shift all cached segment pointers by `delta` bytes (called at a
    /// frame-boundary framebuffer swap).
    pub fn apply_delta(&mut self, delta: isize) {
        self.cache.apply_delta(delta);
    }
}
