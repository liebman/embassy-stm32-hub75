//! Example: HUB75 latched panel on PB8-PB15 with TIM2 CLK on PA0.
//!
//! Draws "Hello" on a 64x32 panel using bitplane latched framebuffer with
//! ISR-driven continuous rendering and double buffering.
//!
//! Pin wiring:
//!   PB8:  R1      PB12: G2
//!   PB9:  G1      PB13: B2
//!   PB10: B1      PB14: LATCH
//!   PB11: R2      PB15: BLANK/OE
//!   PA0:  CLK (TIM2_CH1)
//!
//! DMA1 Channel 1 is used for framebuffer → GPIO transfers, triggered
//! by TIM2 update events. The ISR-driven refresh loop runs autonomously.

#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::rcc::{MSIRange, Sysclk};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use panic_probe as _;
use static_cell::StaticCell;

use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{Color, Hertz, Hub75, Hub75DmaHandler, Hub75Pins8};

const ROWS: usize = 32;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 7;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 =>
        dma::InterruptHandler<peripherals::DMA1_CH1>,
        Hub75DmaHandler<peripherals::DMA1_CH1>;
});

#[link_section = ".shared"]
static SHARED_DATA: MaybeUninit<embassy_stm32::SharedData> = MaybeUninit::uninit();

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting main");
    info!("Initializing primary");
    let mut config = embassy_stm32::Config::default();
    config.rcc.msi = Some(MSIRange::RANGE48M);
    config.rcc.sys = Sysclk::MSI;
    let p = embassy_stm32::init_primary(config, &SHARED_DATA);
    info!("Primary initialized (48 MHz MSI)");

    info!("Initializing pins");
    let pins = Hub75Pins8::new(
        (*p.PB8).into(),
        (*p.PB9).into(),
        (*p.PB10).into(),
        (*p.PB11).into(),
        (*p.PB12).into(),
        (*p.PB13).into(),
        (*p.PB14).into(),
        (*p.PB15).into(),
    )
    .expect("invalid pin configuration");

    info!("Initializing hub75");
    let hub75 = Hub75::new(p.TIM2, p.PA0, p.DMA1_CH1, Irqs, pins, Hertz(20_000_000));

    info!("Initializing framebuffers");
    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();

    info!("Drawing text into fb0");
    Text::with_alignment("Hello", Point::new(32, 20), text_style, Alignment::Center)
        .draw(fb0)
        .expect("failed to draw text");

    info!("Starting ISR-driven rendering");
    let hub75 = hub75.start(fb0).expect("failed to start Hub75");
    info!("Hub75 started");
    // Double-buffered loop: draw into fb1, swap, repeat.
    let mut write_fb: &'static mut FBType = fb1;
    loop {
        Text::with_alignment("Hello", Point::new(32, 20), text_style, Alignment::Center)
            .draw(write_fb)
            .expect("failed to draw text");

        write_fb = hub75.swap(write_fb).await.expect("swap failed");
    }
}
