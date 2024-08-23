#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use cordic as sw_cordic;
use stm32g4xx_hal as hal;

#[defmt_test::tests]
mod tests {
    use super::*;

    use fixed::types::{I1F31, I4F28};
    use hal::{
        cordic,
        cordic::Ext as _,
        pwr::{self, PwrExt as _},
        rcc::{self, RccExt as _},
        stm32::Peripherals,
    };

    type TestCordic = cordic::Cordic<
        cordic::data_type::Q31,
        cordic::data_type::Q31,
        cordic::func::SinCos,
        cordic::prec::P60,
    >;

    #[init]
    fn init() -> TestCordic {
        const PLL_CFG: rcc::PllConfig = rcc::RawPllConfig {
            mux: rcc::PllSrc::HSI,        // 16MHz
            m: rcc::PllMDiv::DIV_4,       // /4 = 4MHz
            n: rcc::PllNMul::MUL_85,      // x85 = 340MHz
            r: Some(rcc::PllRDiv::DIV_2), // /2 = 170MHz
            q: None,
            p: None,
        }
        .validate();

        let p = unsafe { Peripherals::steal() };

        let pwr_cfg = p
            .PWR
            .constrain()
            .vos(pwr::VoltageScale::Range1 { enable_boost: true })
            .freeze();
        let mut rcc = p
            .RCC
            .constrain()
            .freeze(rcc::Config::pll().pll_cfg(PLL_CFG), pwr_cfg);

        defmt::debug!("{}", rcc.clocks);

        p.CORDIC.constrain(&mut rcc).freeze()
    }

    #[test]
    fn sweep(cordic: &mut TestCordic) {
        let mut angle = -1f32;

        while angle < 1. {
            cordic.start(I1F31::from_num(angle));

            let (hw_sin, hw_cos) = cordic.result();
            let (sw_sin, sw_cos) = sw_cordic::sin_cos(I4F28::from_num(angle * 3.14159));

            let (sin_diff, cos_diff) = (
                hw_sin.to_num::<f32>() - sw_sin.to_num::<f32>(),
                hw_cos.to_num::<f32>() - sw_cos.to_num::<f32>(),
            );

            defmt::assert!(sin_diff < 0.00001 && sin_diff > -0.00001);
            defmt::assert!(cos_diff < 0.00001 && cos_diff > -0.00001);

            angle += 0.001;
        }
    }
}
