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
use embassy_stm32::gpio::Level;
use embassy_stm32::gpio::Output;
use embassy_stm32::gpio::Speed;
use embassy_stm32::rcc::{MSIRange, Sysclk};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embedded_graphics::geometry::Point;
use embedded_graphics::geometry::Size;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::primitives::Primitive;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use panic_probe as _;
use static_cell::StaticCell;

use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{hub75_define, Color, Config, Hertz, Hub75Pins8};

use numtoa::NumToA;

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 1;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

hub75_define!(
    hub75,
    embassy_stm32::peripherals::TIM2,
    embassy_stm32::peripherals::DMA1_CH1
);

bind_interrupts!(struct Irqs {
    DMA1_CHANNEL1 =>
        dma::InterruptHandler<peripherals::DMA1_CH1>,
        hub75::Hub75DmaHandler;
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

    let _pwm = Output::new(p.PC3, Level::High, Speed::High);

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
    let hub75 = hub75::init(
        p.TIM2,
        p.PA0,
        p.DMA1_CH1,
        Irqs,
        pins,
        Config::new().frequency(Hertz(6_000_000)),
        fb0,
    );
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
