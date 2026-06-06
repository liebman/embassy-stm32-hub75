//! BCM (Binary Code Modulation) state machine for ISR-driven HUB75 refresh.
//!
//! Tracks which bitplane and repetition count the DMA is currently outputting,
//! advancing through the BCM weighting sequence on each transfer-complete
//! interrupt.

use core::ptr::null;

use crate::framebuffer::FrameBuffer;

pub(crate) const MAX_PLANES: usize = 8;

/// Per-plane pointer + byte-length, indexed by plane number.
pub(crate) type PlaneInfo = [(*const u8, usize); MAX_PLANES];

/// Extract plane pointers from a framebuffer into a `PlaneInfo` array.
pub(crate) fn planes_from_fb(fb: &impl FrameBuffer) -> (PlaneInfo, usize) {
    let plane_count = fb.plane_count();
    assert!(
        plane_count > 0 && plane_count <= MAX_PLANES,
        "plane_count {plane_count} out of range 1..={MAX_PLANES}"
    );
    let mut planes: PlaneInfo = [(null::<u8>(), 0usize); MAX_PLANES];
    for (i, slot) in planes.iter_mut().enumerate().take(plane_count) {
        *slot = fb.plane_ptr_len(i);
    }
    (planes, plane_count)
}

/// ISR-driven BCM state machine.
///
/// Walks through bitplanes with exponential weighting: plane 0 (MSB) is
/// repeated 2^(N-1) times, plane N-1 (LSB) is repeated once.
pub(crate) struct BcmState {
    planes: PlaneInfo,
    plane_count: usize,
    current_plane: usize,
    current_rep: usize,
}

impl BcmState {
    pub const fn new() -> Self {
        Self {
            planes: [(null::<u8>(), 0usize); MAX_PLANES],
            plane_count: 0,
            current_plane: 0,
            current_rep: 0,
        }
    }

    /// Reset with new plane pointers, restarting the BCM sequence from plane 0.
    pub fn reset_with_planes(&mut self, planes: PlaneInfo, plane_count: usize) {
        debug_assert!(plane_count > 0 && plane_count <= MAX_PLANES);
        self.planes = planes;
        self.plane_count = plane_count;
        self.current_plane = 0;
        self.current_rep = 0;
    }

    /// Advance the BCM state machine after a transfer completes.
    /// Returns `true` when a full BCM frame boundary is reached (all planes
    /// with all repetitions have been output).
    pub fn advance(&mut self) -> bool {
        self.current_rep += 1;
        let reps = 1usize << (self.plane_count - 1 - self.current_plane);
        if self.current_rep >= reps {
            self.current_rep = 0;
            self.current_plane += 1;
            if self.current_plane >= self.plane_count {
                self.current_plane = 0;
                return true;
            }
        }
        false
    }

    /// Returns the (pointer, byte_length) for the current plane's DMA data.
    pub fn current_plane(&self) -> (*const u8, usize) {
        self.planes[self.current_plane]
    }

    /// Replace all plane pointers (called at frame-boundary swap).
    pub fn update_planes(&mut self, new_planes: PlaneInfo) {
        self.planes = new_planes;
    }
}
