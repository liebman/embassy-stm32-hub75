//! Example: HUB75 8-bit latched panel driven by GPDMA linked-list on
//! Nucleo-H563ZI.
//!
//! BCM weighting is achieved by duplicating `LinearItem` descriptors
//! in the linked-list chain. Only one ISR fires per frame (at chain
//! completion), compared to the plain DMA backend's ISR-per-plane-repetition.
//!
//! Any GPDMA channel works (2D capability not required).
//!
//! Pin wiring (Port D, lower byte):
//!   PD0: R1      PD4: G2
//!   PD1: G1      PD5: B2
//!   PD2: B1      PD6: LATCH
//!   PD3: R2      PD7: BLANK/OE
//!   PE9: CLK (TIM1 CH1)
//!
//! GPDMA1 Channel 7 transfers framebuffer → GPIO, paced by TIM1
//! update events.

#![no_std]
#![no_main]

use core::fmt;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_time::{Duration, Instant, Timer};
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::Drawable;
use heapless::String;
use panic_probe as _;
use static_cell::StaticCell;

use embassy_stm32_hub75::framebuffer::bitplane::latched::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{hub75_gpdma_define, Color, Config, Hertz, Hub75Pins8};

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 6;

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

hub75_gpdma_define!(hub75, embassy_stm32::peripherals::TIM1, GPDMA1_CH7);

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL7 =>
        dma::InterruptHandler<peripherals::GPDMA1_CH7>,
        hub75::Hub75GpdmaHandler;
});

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn display_task(mut hub75: hub75::Hub75Gpdma<'static, FBType>, mut fb: &'static mut FBType) {
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
        fmt::write(&mut buffer, format_args!("Refresh: {:4}", refresh_rate)).unwrap();
        Text::with_alignment(
            buffer.as_str(),
            Point::new(0, LINE3),
            fps_style,
            Alignment::Left,
        )
        .draw(fb)
        .unwrap();

        fb = hub75.swap(fb).await.expect("swap failed");

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
    info!("Starting main — GPDMA linked-list latched demo (H563)");

    let mut config = embassy_stm32::Config::default();

    // HSE (8 MHz on Nucleo-H563ZI) → PLL → 250 MHz SYSCLK
    config.rcc.hse = Some(Hse {
        freq: Hertz(8_000_000),
        mode: HseMode::BypassDigital,
    });
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div2,
        mul: PllMul::from(124),
        divp: Some(PllDiv::Div2),
        divq: Some(PllDiv::Div2),
        divr: Some(PllDiv::Div2),
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.apb3_pre = APBPrescaler::Div1;
    config.rcc.voltage_scale = VoltageScale::Scale0;

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

    info!("Starting GPDMA-driven rendering");
    let hub75 = hub75::init(
        p.TIM1,
        p.PE9,
        p.GPDMA1_CH7,
        Irqs,
        pins,
        Config::new().frequency(Hertz(20_000_000)),
        fb0,
    );
    info!("Hub75 GPDMA started");

    spawner.spawn(display_task(hub75, fb1).unwrap());

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
