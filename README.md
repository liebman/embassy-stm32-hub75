# embassy-stm32-hub75

> **Work in progress** -- API is unstable and subject to change.

A `no_std` Rust driver for HUB75-style LED matrix panels on STM32
microcontrollers, built on [Embassy](https://embassy.dev).

HUB75 is a standard interface for driving large, bright, and colorful RGB LED
displays commonly used in digital signage and art installations. This library
uses an ISR-driven DMA refresh loop to continuously output framebuffer data to
a GPIO port with zero CPU involvement per pixel.

## How it works

A hardware timer generates a PWM pixel clock on CH1 and triggers DMA
byte-transfers from a bitplane framebuffer to the GPIO ODR register on each
update event. DMA transfer-complete interrupts drive a Binary Code Modulation
(BCM) state machine that advances through bitplanes with exponential weighting.
The ISR stops and resets the timer between planes for deterministic clock
alignment.

## Features

- **ISR-driven refresh** -- once started, rendering runs entirely in hardware
  interrupts with no per-pixel CPU cost
- **Double buffering** -- write to one framebuffer while the ISR renders from
  another, swapping atomically at frame boundaries
- **Multiple instances** -- the `hub75_define!` macro stamps out independent
  per-instance state, so multiple panels can run simultaneously on different
  timer/DMA pairs
- **Configurable clock and GPIO speed** -- pixel clock frequency and GPIO
  output speed are set explicitly via [`Config`](src/lib.rs)
- **BCM grayscale** -- configurable bit depth (1-8 planes) via the
  `hub75-framebuffer` crate

## Supported modes

### Latched (8-pin)

The **latched** configuration uses an external 74HC574-style latch for row
address lines. Only 8 data pins are needed: R1, G1, B1, R2, G2, B2, LATCH,
and BLANK. The 8 pins must occupy either the lower byte (pins 0-7) or upper
byte (pins 8-15) of a single GPIO port, wired in order. Byte-width DMA writes
update only those 8 pins without disturbing the other half of the port.

The required latch circuit schematic and explanation can be found in the
[hub75-framebuffer README](https://github.com/liebman/hub75-framebuffer#the-latch-circuit).

## Quick start

```rust
#![no_std]
#![no_main]

use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{hub75_define, Color, Config, Hertz, Hub75Pins8};
use static_cell::StaticCell;

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 1;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

// 1. Define the driver instance (timer + DMA channel)
hub75_define!(hub75, embassy_stm32::peripherals::TIM2, embassy_stm32::peripherals::DMA1_CH1);

// 2. Bind the DMA interrupt
bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 =>
        dma::InterruptHandler<peripherals::DMA1_CH1>,
        hub75::Hub75DmaHandler;
});

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(Default::default());

    // 3. Configure pins (must be 8 consecutive pins on one port)
    let pins = Hub75Pins8::new(
        (*p.PB8).into(), (*p.PB9).into(), (*p.PB10).into(), (*p.PB11).into(),
        (*p.PB12).into(), (*p.PB13).into(), (*p.PB14).into(), (*p.PB15).into(),
    ).expect("invalid pin configuration");

    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    // 4. Initialize and start rendering
    let hub75 = hub75::init(
        p.TIM2, p.PA0, p.DMA1_CH1, Irqs, pins,
        Config::new().frequency(Hertz(6_000_000)),
        fb0,
    );

    // 5. Double-buffered loop
    let mut write_fb = fb1;
    loop {
        write_fb.erase();
        // draw into write_fb using embedded-graphics...
        write_fb = hub75.swap(write_fb).await.expect("swap failed");
    }
}
```

## Pin wiring

The 8 data pins map to the `hub75-framebuffer` latched byte layout:

| Bit | Signal |
|-----|--------|
| 0   | R1     |
| 1   | G1     |
| 2   | B1     |
| 3   | R2     |
| 4   | G2     |
| 5   | B2     |
| 6   | LATCH  |
| 7   | BLANK  |

The clock pin is passed separately and must be a valid TIM CH1 output for the
chosen timer (enforced at compile time).

## Examples

Working examples are provided for two targets:

- **STM32WL55** (`examples/stm32wl55/`) -- 64x64 panel at 6 MHz pixel clock
- **STM32F722** (`examples/stm32f722/`) -- 64x64 panel at 20 MHz pixel clock,
  includes a multi-task latched example with FPS counters

Each example overrides `Config::frequency` for its target while using the
default `Speed::Medium` GPIO output speed.

Build an example:

```bash
cd examples/stm32f722
cargo build --bin hello
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.
