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
(BCM) state machine that walks the framebuffer's BCM segment sequence,
streaming each segment for its weighted repetition count. The ISR stops and
resets the timer between segments for deterministic clock alignment.

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

### Plain (16-pin)

The **plain** configuration uses all 16 pins of a GPIO port. Half-word-width
DMA writes the full ODR register on each clock cycle. Row address, control,
and data signals are all driven directly — no external latch is required. Pin
layout and bit assignments depend on the framebuffer implementation used (see
the `hub75-framebuffer` crate for details).

Use `Hub75Pins16::new(pins)` with an array of 16 `AnyPin` values, all on the
same GPIO port occupying pins 0-15 in order.

When driving the panel with plain DMA, you will almost certainly want to
enable the `tail-closes-latch` feature. The last word output leaves the latch
open, and the timer typically still emits a few clock pulses before it is
stopped. `tail-closes-latch` appends an extra word that closes the latch on
the next clock cycle, before any more pixels are clocked in.

## Cargo features

All `hub75-framebuffer` features are forwarded through this crate so you do not
need a direct dependency on `hub75-framebuffer`:

| Feature | Description |
|---------|-------------|
| `defmt` | Enable `defmt` logging (forwards to embassy-stm32, hub75-framebuffer, and embedded-graphics) |
| `skip-black-pixels` | Skip writing black pixels to the framebuffer, leaving bitplane data unchanged |
| `invert-oe` | Invert the output-enable signal in the framebuffer |
| `tail-closes-latch` | Append a tail word that closes the latch on the next clock cycle after data is shifted in (plain 16-bit mode only; strongly recommended for plain DMA) |
| `lead-blank-{1,2,4,8,16,32}` | Blank delay cycles before the row-address change (mutually exclusive) |
| `trail-blank-{1,2,4,8,16,32}` | Blank delay cycles after the row-address change (mutually exclusive) |
| `inter-row-blank-{4,8,16,32}` | Blank cycles inserted between rows (mutually exclusive) |
| `reverse-row-order` | Stream rows in reverse order |
| `gpdma` | Use the GPDMA linear linked-list backend (`gpdma::Hub75`). The descriptor chain is circular: the DMA engine starts once and loops forever with no per-frame restart |
| `gpdma-2d` | Use the 2D GPDMA linked-list backend (`gpdma_2d::Hub75`); implies `gpdma` |
| `unsafe-swap-wait-1` | GPDMA backends only: shorten `swap()`'s transfer-complete wait from two interrupts to one |
| `unsafe-swap-wait-0` | GPDMA backends only: return from `swap()` as soon as the descriptor delta is applied (no interrupt wait) |

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

### 16-bit plain mode

All 16 pins of a GPIO port are used. The bit mapping from the
`hub75-framebuffer` bitplane plain layout:

| Bit   | Signal       |
|-------|--------------|
| 0-4   | A..E (row address) |
| 5     | LAT (latch)  |
| 6-7   | (unused)     |
| 8     | OE (blank)   |
| 9     | R1           |
| 10    | G1           |
| 11    | B1           |
| 12    | R2           |
| 13    | G2           |
| 14    | B2           |
| 15    | (unused)     |

### 8-bit latched mode

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

### Clock pin

The clock pin is passed separately and must be a valid TIM CH1 output for the
chosen timer (enforced at compile time).

## Examples

Two working examples are provided, each driving a 64x64 panel with a gradient
plus refresh-rate / render-time overlays:

- **`examples/gradient/`** -- 16-bit plain mode (`Hub75Pins16`) on PD0-PD15
  with TIM1 CLK on PE9. Supports `stm32f722`, `stm32h563`, and `stm32h723`.
- **`examples/gradient-latched/`** -- 8-bit latched mode (`Hub75Pins8`) on
  PD0-PD7 (R1/G1/B1/R2/G2/B2/LATCH/BLANK) with TIM1 CLK on PE9 and a
  SmartLEDShield-style latch circuit. Supports `stm32f722` and `stm32h563`.

These examples are not limited to the boards they are wired for: the same code
works on almost any STM32, as long as the required pins are available as
consecutive pins on a single GPIO port (any port will do). See
[Supported modes](#supported-modes) for the exact pin requirements.

Each example declares cargo aliases in its own `.cargo/config.toml` to build,
run, and clippy it for every board it supports. From inside an example
directory:

```bash
cd examples/gradient
cargo run-f722        # build (release) + flash an STM32F722
cargo run-h563        # build (release) + flash an STM32H563
cargo run-h723        # build (release) + flash an STM32H723

cargo build-h723      # build only, without flashing
cargo clippy-h723     # clippy
```

`run-<board>` loads the matching board config
(`.cargo/config-stm32<board>.toml`) and flashes the target with `probe-rs`.

The pixel clock defaults to 10 MHz, and the simplest (dumb) DMA driver is the
default backend. Pass `-F` with a comma-separated feature list to override
these, switch to a GPDMA backend, or pass through driver options:

```bash
cargo run-h723 -F 20mhz                       # 20 MHz pixel clock
cargo run-h563 -F gpdma                       # GPDMA linear linked-list
cargo run-h563 -F gpdma-2d                    # GPDMA 2D linked-list
cargo run-h563 -F gpdma,trail-blank-4,20mhz,unsafe-swap-wait-1
```

The `gpdma` / `gpdma-2d` backends are only available on chips that have a
GPDMA peripheral (e.g. `stm32h563`).

`gradient-latched` supports `stm32f722` and `stm32h563` only, so its aliases
are `run-f722` / `run-h563` (plus matching `build-*` / `clippy-*`).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.
