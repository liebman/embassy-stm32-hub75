# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

- Initial release
- ISR-driven DMA refresh with BCM grayscale (1-8 planes)
- Double buffering with atomic frame swap
- 8-bit latched mode (`Hub75Pins8`) with external 74HC574 latch
- 16-bit plain mode (`Hub75Pins16`) using full GPIO port width
- `hub75_define!` macro for multiple independent instances
- Examples for STM32WL55, STM32F722, and STM32H723

<!-- next-url -->
[Unreleased]: https://github.com/liebman/embassy-stm32-hub75/compare/v0.1.0...HEAD
