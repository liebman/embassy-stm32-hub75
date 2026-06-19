//! Example: HUB75 latched panel on PD0-PD7 with TIM1 CLK on PE9.
//!
//! Draws "Hello" on a 64x64 panel using bitplane latched framebuffer with
//! ISR-driven continuous rendering and double buffering.
//!
//! Pin wiring:
//!   PD0:  R1      PD4: G2
//!   PD1:  G1      PD5: B2
//!   PD2:  B1      PD6: LATCH
//!   PD3:  R2      PD7: BLANK/OE
//!   PE9:  CLK (TIM1_CH1)
//!
//! DMA2 Channel 5 is used for framebuffer → GPIO transfers, triggered
//! by TIM1 update events. The ISR-driven refresh loop runs autonomously.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embedded_graphics::geometry::{Point, Size};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::primitives::{Primitive, PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use numtoa::NumToA;
use panic_probe as _;
use static_cell::StaticCell;

use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{hub75_define, Color, Hertz, Hub75Pins8};

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 1;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

hub75_define!(hub75, embassy_stm32::peripherals::TIM1, embassy_stm32::peripherals::DMA2_CH5);

bind_interrupts!(struct Irqs {
    DMA2_STREAM5 =>
        dma::InterruptHandler<peripherals::DMA2_CH5>,
        hub75::Hub75DmaHandler;
});

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting main");
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.hsi = true;
    config.rcc.pll_src = PllSource::HSI;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::DIV8,
        mul: PllMul::MUL216,
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV9),
        divr: Some(PllRDiv::DIV2),
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV4;
    config.rcc.apb2_pre = APBPrescaler::DIV2;

    let p = embassy_stm32::init(config);

    let _pwm = Output::new(p.PC3, Level::High, Speed::High);

    info!("Initializing pins");
    let pins = Hub75Pins8::new(
        (*p.PD0).into(),
        (*p.PD1).into(),
        (*p.PD2).into(),
        (*p.PD3).into(),
        (*p.PD4).into(),
        (*p.PD5).into(),
        (*p.PD6).into(),
        (*p.PD7).into(),
    )
    .expect("invalid pin configuration");

    info!("Initializing framebuffers");
    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();

    let rect_style = PrimitiveStyleBuilder::new()
        .stroke_color(Color::GREEN)
        .stroke_width(1)
        .build();
    let rect_style_red = PrimitiveStyleBuilder::new()
        .stroke_color(Color::RED)
        .stroke_width(1)
        .build();

    info!("Starting ISR-driven rendering");
    let hub75 = hub75::init(p.TIM1, p.PE9, p.DMA2_CH5, Irqs, pins, Hertz(20_000_000), fb0);
    info!("Hub75 started");
    // Double-buffered loop: draw into fb1, swap, repeat.
    let mut write_fb: &'static mut FBType = fb1;
    let mut last_frame_count: u32 = 0;
    let mut count = 0;
    let mut last_count_time = embassy_time::Instant::now();
    loop {
        let mut buffer = [0u8; 32];

        let now = embassy_time::Instant::now();
        if now.duration_since(last_count_time) > embassy_time::Duration::from_millis(1000) {
            let frame_count = hub75.frame_count();
            count = frame_count.saturating_sub(last_frame_count);
            last_count_time = now;
            last_frame_count = frame_count;
        }

        let renders = count.numtoa_str(10, &mut buffer);

        write_fb.erase();
        Text::with_alignment("Hello", Point::new(32, 35), text_style, Alignment::Center)
            .draw(write_fb)
            .expect("failed to draw text");

        Text::with_alignment(renders, Point::new(32, 52), text_style, Alignment::Center)
            .draw(write_fb)
            .expect("failed to draw text");

        Rectangle::new(Point { x: 1, y: 1 }, Size::new(62, 62))
            .into_styled(rect_style)
            .draw(write_fb)
            .expect("failed to draw rectangle");
        Rectangle::new(Point { x: 0, y: 0 }, Size::new(64, 64))
            .into_styled(rect_style_red)
            .draw(write_fb)
            .expect("failed to draw rectangle");

        write_fb = hub75.swap(write_fb).await.expect("swap failed");
    }
}
