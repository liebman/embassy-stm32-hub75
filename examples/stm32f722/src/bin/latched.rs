//! Example: HUB75 latched panel on PB8-PB15 with TIM2 CLK on PA0.
//!
//! Draws "Hello" on a 64x32 panel using bitplane latched framebuffer with
//! ISR-driven continuous rendering and double buffering.
//!
//! Pin wiring:
//!   PBD0  R1      PD4: G2
//!   PBD1:  G1      PD5: B2
//!   PBD2: B1      PD6: LATCH
//!   PBD3: R2      PD7: BLANK/OE
//!   PE9:  CLK (TIM1)
//!
//! DMA2 Channel 5 is used for framebuffer → GPIO transfers, triggered
//! by TIM1 update events. The ISR-driven refresh loop runs autonomously.

#![no_std]
#![no_main]

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource, Sysclk};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_time::Timer;
use embassy_time::{Duration, Instant};
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use embedded_graphics::Drawable;
use panic_probe as _;
use static_cell::StaticCell;
use heapless::String;

use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{Color, Hertz, Hub75, Hub75DmaHandler, Hub75Pins8};

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 6;

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

bind_interrupts!(struct Irqs {
    DMA2_STREAM5 =>
        dma::InterruptHandler<peripherals::DMA2_CH5>,
        Hub75DmaHandler<peripherals::DMA2_CH5>;
});

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn display_task(hub75: Hub75<'static, peripherals::TIM1, FBType>, mut fb: &'static mut FBType) {
    info!("display_task: starting!");
    let fps_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    let mut render_count = 0u32;
    let mut refresh_count_start = hub75.frame_count();
    let mut start = Instant::now();
    let mut refresh_rate = 0u32;

    loop {
        fb.erase();

        const STEP: u8 = (256 / COLS) as u8;
        for x in 0..COLS {
            let brightness = (x as u8) * STEP;
            for y in 0..NBARS {
                fb.set_pixel(Point::new(x as i32, y), Color::new(brightness, 0, 0));
                fb.set_pixel(
                    Point::new(x as i32, y + NBARS),
                    Color::new(0, brightness, 0),
                );
                fb.set_pixel(
                    Point::new(x as i32, y + 2 * NBARS),
                    Color::new(0, 0, brightness),
                );
            }
        }

        let mut buffer: String<64> = String::new();

        fmt::write(&mut buffer, format_args!("Refresh: {:4}", refresh_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE3),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(
            &mut buffer,
            format_args!("Render: {:5}", RENDER_RATE.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE2),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        buffer.clear();
        fmt::write(
            &mut buffer,
            format_args!("Simple: {:5}", SIMPLE_COUNTER.load(Ordering::Relaxed)),
        )
        .unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE1),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        fb = hub75.swap(fb).await.expect("DMA transfer failed");

        render_count += 1;
        const FPS_INTERVAL: Duration = Duration::from_secs(1);
        if start.elapsed() > FPS_INTERVAL {
            RENDER_RATE.store(render_count, Ordering::Relaxed);
            let current_frame_count = hub75.frame_count();
            refresh_rate = current_frame_count.wrapping_sub(refresh_count_start);
            refresh_count_start = current_frame_count;
            render_count = 0;
            start = Instant::now();
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
        // HCLK (AHB bus) = SYSCLK / 1 = 216 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV1; 

    // PCLK1 (APB1 bus) = HCLK / 4 = 54 MHz (Safe: usually max 54 MHz on F7)
    config.rcc.apb1_pre = APBPrescaler::DIV4; 

    // PCLK2 (APB2 bus) = HCLK / 2 = 108 MHz (Safe: usually max 108 MHz on F7)
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

    info!("Initializing hub75");
    let hub75 = Hub75::new(p.TIM1, p.PE9, p.DMA2_CH5, Irqs, pins, Hertz(22_000_000));

    info!("Initializing framebuffers");
    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    info!("Starting ISR-driven rendering");
    let hub75 = hub75.start(fb0).expect("failed to start Hub75");
    info!("Hub75 started");

    spawner.spawn(display_task(hub75, fb1).unwrap());

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
