//! Example: HUB75 latched panel on PB8-PB15 with TIM2 CLK on PA0.
//!
//! Draws "Hello" on a 64x32 panel using bitplane latched framebuffer.
//!
//! Pin wiring:
//!   PB8:  R1      PB12: G2
//!   PB9:  G1      PB13: B2
//!   PB10: B1      PB14: LATCH
//!   PB11: R2      PB15: BLANK/OE
//!   PA0:  CLK (TIM2_CH1)
//!
//! DMA1 Channel 1 is used for framebuffer → GPIO transfers, triggered
//! by TIM2 update events.

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
use stm_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use stm_hub75::framebuffer::compute_rows;
use stm_hub75::{Color, Hertz, Hub75, Hub75Pins8};

const ROWS: usize = 32;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 7;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
});

#[link_section = ".shared"]
static SHARED_DATA: MaybeUninit<embassy_stm32::SharedData> = MaybeUninit::uninit();

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
    let mut hub75 = Hub75::new(p.TIM2, p.PA0, p.DMA1_CH1, Irqs, pins, Hertz(20_000_000));

    info!("Initializing framebuffer");
    let mut fb = FBType::new();
    info!("Framebuffer initialized");

    info!("Initializing text style");
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();

    info!("Drawing text");
    Text::with_alignment("Hello", Point::new(32, 20), text_style, Alignment::Center)
        .draw(&mut fb)
        .expect("failed to draw text");
    info!("Text drawn");

    // info!("row: {}", fb.);
    loop {
        hub75.render(&fb).await;
    }
}
