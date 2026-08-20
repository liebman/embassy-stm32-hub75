//! STM32H563 board support (dumb DMA by default; optional GPDMA backends).

use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};

use embassy_stm32::dma;
use embassy_stm32::peripherals;

// The backend is selected at compile time by the `gpdma` / `gpdma-2d`
// features (dumb DMA is the default); `hub75_define!` dispatches to the
// matching backend.
embassy_stm32_hub75::hub75_define!(
    hub75,
    embassy_stm32::peripherals::TIM1,
    embassy_stm32::peripherals::GPDMA1_CH7
);

embassy_stm32::bind_interrupts!(pub struct Irqs {
    GPDMA1_CHANNEL7 =>
        dma::InterruptHandler<peripherals::GPDMA1_CH7>,
        hub75::Hub75DmaHandler;
});

pub use hub75::Hub75;

pub fn config() -> embassy_stm32::Config {
    // HSE (8 MHz on Nucleo-H563ZI) → PLL → 250 MHz SYSCLK.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32_hub75::Hertz(8_000_000),
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
    config
}
