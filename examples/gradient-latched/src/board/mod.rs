//! Board- and backend-specific setup for the `gradient-latched` example.
//!
//! This module isolates everything that differs between the supported
//! targets so `main.rs` stays chip-agnostic:
//!
//! - the `embassy_stm32::Config` passed to `embassy_stm32::init` (clock
//!   tree / PLL configuration), via `config()`
//! - the HUB75 driver instance (a single `hub75_define!` invocation),
//!   exposed as the `hub75` module
//! - the DMA / GPDMA interrupt bindings (the `Irqs` struct)
//! - a `Hub75` driver type re-export so `main.rs` can name the driver
//!   type regardless of backend
//!
//! The dumb DMA driver is the default backend; on STM32H563 the `gpdma` /
//! `gpdma-2d` features switch to the linked-list backends. `hub75_define!`
//! dispatches to the matching backend at compile time.
//!
//! Each chip's implementation lives in its own file, selected at compile
//! time via `#[cfg_attr(..., path = "...")]` on the private `chip` module.

// ---------------------------------------------------------------------------
// Board / backend selection validation
// ---------------------------------------------------------------------------

#[cfg(not(any(feature = "stm32f722", feature = "stm32h563")))]
compile_error!("no board selected; enable exactly one of: `stm32f722`, `stm32h563`");

#[cfg(all(feature = "stm32f722", feature = "stm32h563"))]
compile_error!("multiple board features enabled; enable exactly one of: `stm32f722`, `stm32h563`");

#[cfg(all(feature = "gpdma", feature = "gpdma-2d"))]
compile_error!("enable at most one of: `gpdma`, `gpdma-2d`");

#[cfg(all(feature = "stm32f722", any(feature = "gpdma", feature = "gpdma-2d")))]
compile_error!("`stm32f722` has no GPDMA; `gpdma` and `gpdma-2d` are only valid with `stm32h563`");

// ---------------------------------------------------------------------------
// Chip-specific implementation
// ---------------------------------------------------------------------------

#[cfg_attr(feature = "stm32f722", path = "stm32f722.rs")]
#[cfg_attr(feature = "stm32h563", path = "stm32h563.rs")]
mod chip;

pub use chip::*;
