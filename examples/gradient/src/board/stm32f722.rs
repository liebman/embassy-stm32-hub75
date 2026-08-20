//! STM32F722 board support (classic DMA backend).

use embassy_stm32::rcc::{
    AHBPrescaler, APBPrescaler, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource,
    Sysclk,
};

use embassy_stm32::dma;
use embassy_stm32::peripherals;

embassy_stm32_hub75::hub75_define!(
    hub75,
    embassy_stm32::peripherals::TIM1,
    embassy_stm32::peripherals::DMA2_CH5
);

embassy_stm32::bind_interrupts!(pub struct Irqs {
    DMA2_STREAM5 =>
        dma::InterruptHandler<peripherals::DMA2_CH5>,
        hub75::Hub75DmaHandler;
});

pub type Hub75<'d, FB> = hub75::Hub75<'d, FB>;

pub fn config() -> embassy_stm32::Config {
    // HSI (16 MHz) → PLL → 216 MHz SYSCLK.
    let mut config = embassy_stm32::Config::default();
    config.rcc.sys = Sysclk::Pll1P;
    config.rcc.hsi = true;
    config.rcc.pll_src = PllSource::Hsi;
    config.rcc.pll = Some(Pll {
        prediv: PllPreDiv::Div8,
        mul: PllMul::Mul216,
        divp: Some(PllPDiv::Div2),
        divq: Some(PllQDiv::Div9),
        divr: Some(PllRDiv::Div2),
    });
    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div4;
    config.rcc.apb2_pre = APBPrescaler::Div2;
    config
}
