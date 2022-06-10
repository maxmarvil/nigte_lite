#![no_std]
#![no_main]
#![deny(warnings)]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::{block, prelude::*, stm32};
use stm32g0xx_hal::rcc::Config;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::lsi());
    //let  delay = dp.TIM14.delay(&mut rcc);

    let  timeout = 4000;
    let mut timer = dp.TIM16.timer(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    //let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpioa.pa11.into_push_pull_output();
    let pir = gpioa.pa12.into_pull_down_input();
    //pir.listen(SignalEdge.Rising, );
    let mut preview_pir_status = false;

    //let mut opto_pin = gpioc.pc14.into_analog();
    loop {
        let pir_status = match pir.is_high() {
            Ok(true) => true,
            Ok(false) => false,
            _ => unreachable!(),
        };
        if pir_status {
            timer.reset();
            led.set_high().unwrap();
            preview_pir_status = true;
            rprintln!("pir up");
        } else {
            if preview_pir_status {
                rprintln!("pir down after up");
                timer.start(timeout.ms());
                rprintln!("start timer");
                block!(timer.wait()).unwrap();
                led.set_low().unwrap();
                //timer.reset();
                //timer.clear_irq();
                rprintln!("timer down {}",timer.get_current());
            }
            preview_pir_status = false;
        }



    }
}
