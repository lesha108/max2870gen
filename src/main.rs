#![no_std]
#![no_main]

mod max2870;

extern crate panic_halt;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};

use crate::max2870::*;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // Configure the syst timer
    let mut delay = cp.SYST.delay(&clocks);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    let pins = (
        gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl), // sck
        gpioa.pa6.into_floating_input(&mut gpioa.crl), // miso - не используем по факту и не подключаем
        gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl), // mosi
    );
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi1(dp.SPI1, pins, &mut afio.mapr, spi_mode, 1.MHz(), clocks);

    // Configure gpio C pin 13 as a push-pull output. Светодиод на bluepill
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    let mut ce = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
    let mut le = gpiob.pb11.into_push_pull_output(&mut gpiob.crh);

    // включаем чип
    le.set_high();
    ce.set_high();
    delay.delay(100.millis());

    let mut gen = Max2870::new(spi, le);
    gen.clear();
    gen.set_defaults();

    // делаем выходную частоту 25 Мгц при опоре 25 Мгц
    let r0 = EN_INT | N_SET | F_SET | REG_0;
    gen.set_reg(0, r0);
    let r2 = SDN | MUX_2 | R_DIV | CP_SET | LDF | PDP | REG_2;
    gen.set_reg(2, r2);
    let r4 = REG4HEAD | BS_MSB | DIVA | BS_LSB | RFA_EN | APWR | REG_4;
    gen.set_reg(4, r4);

    let mut res_delay = 1.secs();
    match gen.init(&mut delay) {
        Err(_) => {
            res_delay = 200.millis();
        }
        Ok(_) => {}
    };

    // после успешной настройки генератора просто мигаем светодиодом
    loop {
        led.set_high();
        delay.delay(res_delay);
        led.set_low();
        delay.delay(res_delay);
    }
}
