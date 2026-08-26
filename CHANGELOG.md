# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

### ⚠️ Breaking

* Renamed blank delay features from `blank-delay-1/2/4/8` to separate `lead-blank-1/2/4/8/16` and `trail-blank-1/2/4/8/16` features. The lead blank delay controls how many clock cycles the output is blanked before the row address is changed, and the trail blank delay controls blanking after the row address is changed. The new `16` value is also available. Default is 1 for plain framebuffers and 0 for latched framebuffers (which handle timing via extra `Address` entries to manage the address change).

### Added

* Support for row-major bitplane framebuffers (`framebuffer::bitplane::{plain,latched}::row::DmaFrameBuffer`): the basic DMA and GPDMA 2D backends drive them directly; the GPDMA linear backend is limited to sequences of at most `gpdma::MAX_DESCRIPTORS` (255) descriptors.
* New feature passthroughs: `lead-blank-32`, `trail-blank-32`, `inter-row-blank-4/8/16/32`, and `reverse-row-order`.
* New features `unsafe-swap-wait-1` and `unsafe-swap-wait-0` to shorten the GPDMA `swap()` wait to one / zero transfer-complete interrupts (opt-in; may cause visual tearing).

### Changed

* The GPDMA backends (`gpdma` / `gpdma-2d`) now apply the framebuffer swap delta directly in `swap()` and wait for the configured number of transfer-complete interrupts (two by default) before returning the old framebuffer.

## [0.2.0] - 2026-07-04

### ⚠️ Breaking

- `Hub75::swap()` now takes `&mut self` instead of `&self`
- `Hub75Pins` trait is now sealed (cannot be implemented outside this crate)

### Added

- `Hub75Error` now implements `Display` and `core::error::Error`

### Fixed

- Potential `u32` overflow in timer compare value calculation on 32-bit timers

## [0.1.0] - 2026-07-03

- Initial release
- ISR-driven DMA refresh with BCM grayscale (1-8 planes)
- Double buffering with atomic frame swap
- 8-bit latched mode (`Hub75Pins8`) with external 74HC574 latch
- 16-bit plain mode (`Hub75Pins16`) using full GPIO port width
- `hub75_define!` macro for multiple independent instances
- Examples for STM32WL55, STM32F722, and STM32H723

<!-- next-url -->
[Unreleased]: https://github.com/liebman/embassy-stm32-hub75/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/liebman/embassy-stm32-hub75/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/liebman/embassy-stm32-hub75/compare/v0.1.0...v0.1.0
