//! Embassy (async) HUB75 plain demo driving a 64x64 panel with a 16-bit
//! bitplane framebuffer and the full HUB75 signal set on a single GPIO port.
//!
//! Select the target board with exactly one of the `stm32f722`, `stm32h563`,
//! or `stm32h723` features. The dumb DMA driver is the default backend; on
//! `stm32h563`, add the `gpdma` (linear linked-list) or `gpdma-2d` (2D)
//! feature to switch to a linked-list backend.
//!
//! The pixel clock defaults to 10 MHz; enable the `20mhz` feature for a
//! 20 MHz pixel clock.
//!
//! The ISR runs the BCM refresh loop; the async `swap()` method exchanges
//! framebuffers without blocking. The display task draws a gradient plus
//! refresh-rate, render-rate, and simple-counter overlays.
//!
//! Pin wiring (identical for all targets):
//!   PD0-PD15: 16-bit HUB75 data bus
//!   PE9:      CLK (TIM1 CH1)
//!
//! Note that you most likely need level converters 3.3v to 5v for all HUB75
//! signals.

#![no_std]
#![no_main]

mod board;

use core::fmt;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
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
// `bcm_rep_count` is used by the dumb and GPDMA-linear backends;
// `FrameBuffer` (for `BCM_SEGMENT_COUNT`) by the dumb and GPDMA-2D backends.
#[cfg(not(feature = "gpdma-2d"))]
use embassy_stm32_hub75::framebuffer::bcm_rep_count;
#[cfg(not(feature = "gpdma"))]
use embassy_stm32_hub75::framebuffer::FrameBuffer;
use embassy_stm32_hub75::{Color, Config, Hertz, Hub75Pins16};

const ROWS: usize = 64;
const COLS: usize = 64;
const NROWS: usize = compute_rows(ROWS);
const PLANES: usize = 6;

// Pixel clock: 10 MHz by default, 20 MHz with the `20mhz` feature.
#[cfg(not(feature = "20mhz"))]
const PIXEL_CLOCK: Hertz = Hertz(10_000_000);
#[cfg(feature = "20mhz")]
const PIXEL_CLOCK: Hertz = Hertz(20_000_000);

const LINE1: i32 = ROWS as i32 - 1 - 14;
const LINE2: i32 = ROWS as i32 - 1 - 7;
const LINE3: i32 = ROWS as i32 - 1;
const NBARS: i32 = ROWS as i32 / 8;

type FBType = DmaFrameBuffer<NROWS, COLS, PLANES>;

static FB0: StaticCell<FBType> = StaticCell::new();
static FB1: StaticCell<FBType> = StaticCell::new();

static RENDER_RATE: AtomicU32 = AtomicU32::new(0);
static SIMPLE_COUNTER: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
async fn display_task(mut hub75: board::Hub75<'static, FBType>, mut fb: &'static mut FBType) {
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

unsafe extern "C" {
    // Provided by the cortex-m-rt linker script
    static _stack_start: u32;
    static _stack_end: u32;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Starting main");
    let p = embassy_stm32::init(board::config());

    info!("Main starting!");
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start).offset_from(core::ptr::addr_of!(_stack_end))
    });
    info!("ROWS: {}", ROWS);
    info!("COLS: {}", COLS);
    info!("PLANES: {}", PLANES);
    info!("FB size: {}", core::mem::size_of::<FBType>());

    // DMA descriptor accounting. Only the GPDMA linked-list backends use a
    // descriptor table; the default dumb-DMA driver has none and instead
    // re-kicks one transfer per BCM segment repetition from the ISR.
    #[cfg(not(any(feature = "gpdma", feature = "gpdma-2d")))]
    info!(
        "DMA: dumb backend (no descriptor table): {} BCM segments, {} transfers/frame",
        FBType::BCM_SEGMENT_COUNT,
        bcm_rep_count::<FBType>()
    );

    #[cfg(feature = "gpdma")]
    info!(
        "DMA: GPDMA linear linked-list: {} descriptors, {} bytes used",
        bcm_rep_count::<FBType>(),
        bcm_rep_count::<FBType>()
            * core::mem::size_of::<embassy_stm32::dma::linked_list::LinearItem>()
    );

    #[cfg(feature = "gpdma-2d")]
    info!(
        "DMA: GPDMA 2D: {} descriptors, {} bytes used",
        FBType::BCM_SEGMENT_COUNT,
        FBType::BCM_SEGMENT_COUNT * core::mem::size_of::<embassy_stm32::dma::two_d::TwoDItem>()
    );

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

    let fb0 = FB0.init(FBType::new());
    let fb1 = FB1.init(FBType::new());

    info!("fb0: {:?}", fb0);
    info!("fb1: {:?}", fb1);

    #[cfg(feature = "stm32f722")]
    let dma = p.DMA2_CH5;
    #[cfg(feature = "stm32h563")]
    let dma = p.GPDMA1_CH7;
    #[cfg(feature = "stm32h723")]
    let dma = p.DMA1_CH0;

    let hub75 = board::hub75::init(
        p.TIM1,
        p.PE9,
        dma,
        board::Irqs,
        pins,
        Config::new().frequency(PIXEL_CLOCK),
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
