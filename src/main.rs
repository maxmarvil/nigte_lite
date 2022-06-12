#![no_std]
#![no_main]
#![deny(warnings)]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::{ prelude::*, stm32, analog::adc::{OversamplingRatio, Precision, SampleTime}};
use stm32g0xx_hal::rcc::Config;


#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    //let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::lsi());
    let mut adc = dp.ADC.constrain(&mut rcc);
    let mut delay = dp.TIM14.delay(&mut rcc);

    adc.set_sample_time(SampleTime::T_80);
    adc.set_precision(Precision::B_12);
    adc.set_oversampling_ratio(OversamplingRatio::X_16);
    adc.set_oversampling_shift(16);
    adc.oversampling_enable(true);

    delay.delay(20.us()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let timeout = 5000;
    let mut timer = dp.TIM16.timer(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    //let gpioc = dp.GPIOC.split(&mut rcc);
    let mut led = gpioa.pa11.into_push_pull_output();
    let pir = gpioa.pa12.into_pull_down_input();
    let mut opto_pin = gpiob.pb8.into_analog();
    //pir.listen(SignalEdge.Rising, );
    let mut preview_pir_status = false;
    let mut timer_need_start = false;
    let mut timer_started = false;
    //timer.start(timeout.ms());

    //let mut opto_pin = gpioc.pc14.into_analog();
    loop {
        let pir_status = pir.is_high().unwrap();
        let opto_data = adc.read_voltage(&mut opto_pin).unwrap();

        rprintln!("opto read data {}", opto_data);
        if pir_status {
            if preview_pir_status == false {

                rprintln!("pir up");
                led.set_high().unwrap();
                preview_pir_status = true;
                timer_need_start = true;
            }
            if timer_started {
                timer.reset();
            }
        } else if preview_pir_status{

            if timer_need_start {
                timer_started = true;
                timer.start(timeout.ms());
            }
            timer_need_start = false;

            rprintln!("timer down {}", timer.get_current());

            if timer.wait().is_ok() {
                // новый цикл таймера
                rprintln!("timer is ok");
                led.set_low().unwrap();

                timer.reset();
                timer.clear_irq();
                preview_pir_status = false;
            }

        }



    }
}
