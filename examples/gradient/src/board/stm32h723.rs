//! STM32H723 board support (classic DMA backend).

use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk,
    VoltageScale,
};

use embassy_stm32::dma;
use embassy_stm32::peripherals;

embassy_stm32_hub75::hub75_define!(
    hub75,
    embassy_stm32::peripherals::TIM1,
    embassy_stm32::peripherals::DMA1_CH0
);

embassy_stm32::bind_interrupts!(pub struct Irqs {
    DMA1_STREAM0 =>
        dma::InterruptHandler<peripherals::DMA1_CH0>,
        hub75::Hub75DmaHandler;
});

pub use hub75::Hub75;

pub fn config() -> embassy_stm32::Config {
    // HSE (25 MHz) → PLL → 250 MHz SYSCLK.
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(Hse {
        freq: embassy_stm32_hub75::Hertz(25_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.pll1 = Some(Pll {
        source: PllSource::Hse,
        prediv: PllPreDiv::Div5,
        mul: PllMul::from(100),
        divp: Some(PllDiv::Div2),
        divq: Some(PllDiv::Div4),
        divr: Some(PllDiv::Div2),
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div2;
    config.rcc.apb2_pre = APBPrescaler::Div2;
    config.rcc.apb3_pre = APBPrescaler::Div2;
    config.rcc.apb4_pre = APBPrescaler::Div2;
    config.rcc.voltage_scale = VoltageScale::Scale0;
    config
}
