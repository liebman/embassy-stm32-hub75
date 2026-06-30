//! Example: HUB75 16-bit plain panel on PD0-PD15 with TIM1 CLK on PE9.
//!
//! Draws gradient bars + FPS counters on a 64x64 panel using the 16-bit
//! bitplane plain framebuffer with ISR-driven continuous rendering and double
//! buffering.
//!
//! Pin wiring (Port D, pins 0-15):
//!   PD0-PD15: 16-bit HUB75 data bus
//!   PE9:      CLK (TIM1 CH1)
//!
//! DMA1 Stream 0 is used for framebuffer → GPIO transfers, triggered
//! by TIM1 update events. The ISR-driven refresh loop runs autonomously.

#![no_std]
#![no_main]

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::Speed;
use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};
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
use heapless::String;
use panic_probe as _;
use static_cell::StaticCell;

use embassy_stm32_hub75::framebuffer::bitplane::plain::DmaFrameBuffer;
use embassy_stm32_hub75::framebuffer::compute_rows;
use embassy_stm32_hub75::{hub75_define, Color, Config, Hertz, Hub75Pins16};

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 6;

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

hub75_define!(
    hub75,
    embassy_stm32::peripherals::TIM1,
    embassy_stm32::peripherals::DMA1_CH0
);

bind_interrupts!(struct Irqs {
    DMA1_STREAM0 =>
        dma::InterruptHandler<peripherals::DMA1_CH0>,
        hub75::Hub75DmaHandler;
});

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn display_task(hub75: hub75::Hub75<'static, FBType>, mut fb: &'static mut FBType) {
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
                // fb.set_pixel(
                //     Point::new(x as i32, y + 3 * NBARS),
                //     Color::new(brightness, brightness, brightness),
                // );
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

    config.rcc.hse = Some(Hse {
        freq: Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV5,
        mul: PllMul::from(100),
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV2;
    config.rcc.apb3_pre = APBPrescaler::DIV2;
    config.rcc.apb4_pre = APBPrescaler::DIV2;
    config.rcc.voltage_scale = VoltageScale::Scale0;

    let p = embassy_stm32::init(config);

    info!("Initializing pins");
    let pins = Hub75Pins16::new([
        (*p.PD0).into(),
        (*p.PD1).into(),
        (*p.PD2).into(),
        (*p.PD3).into(),
        (*p.PD4).into(),
        (*p.PD5).into(),
        (*p.PD6).into(),
        (*p.PD7).into(),
        (*p.PD8).into(),
        (*p.PD9).into(),
        (*p.PD10).into(),
        (*p.PD11).into(),
        (*p.PD12).into(),
        (*p.PD13).into(),
        (*p.PD14).into(),
        (*p.PD15).into(),
    ])
    .expect("invalid pin configuration");

    info!("Initializing framebuffers");
    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    info!("Starting ISR-driven rendering");
    let hub75 = hub75::init(
        p.TIM1,
        p.PE9,
        p.DMA1_CH0,
        Irqs,
        pins,
        Config::new()
            .frequency(Hertz(20_000_000))
            .gpio_speed(Speed::VeryHigh),
        fb0,
    );
    info!("Hub75 started");

    spawner.spawn(display_task(hub75, fb1).unwrap());

    loop {
        if SIMPLE_COUNTER.fetch_add(1, Ordering::Relaxed) >= 99999 {
            SIMPLE_COUNTER.store(0, Ordering::Relaxed);
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
