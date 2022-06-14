#![no_std]
#![no_main]
#![deny(warnings)]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::{ prelude::*, stm32, analog::adc::{OversamplingRatio, Precision, SampleTime}};
use stm32g0xx_hal::rcc::{Config, Prescaler};
//use stm32g0xx_hal::rcc::SysClockSrc;
use stm32g0xx_hal::stm32::TIM3;
use stm32g0xx_hal::stm32::TIM14;
use stm32g0xx_hal::timer::Channel3;
use stm32g0xx_hal::timer::delay::Delay;
use stm32g0xx_hal::timer::pwm::PwmPin;


#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    //let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi(Prescaler::NotDivided));//Config::new(SysClockSrc::PLL)
    let mut adc = dp.ADC.constrain(&mut rcc);
    let mut delay = dp.TIM14.delay(&mut rcc);

    adc.set_sample_time(SampleTime::T_80);
    adc.set_precision(Precision::B_12);
    adc.set_oversampling_ratio(OversamplingRatio::X_16);
    adc.set_oversampling_shift(16);
    adc.oversampling_enable(true);

    delay.delay(20.micros()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let timeout = 5000;
    let mut timer = dp.TIM16.timer(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let pwm = dp.TIM3.pwm(1.kHz(), &mut rcc);

    let mut pwm_pin = pwm.bind_pin(gpiob.pb0);
    let pir = gpioa.pa12.into_pull_down_input();
    let mut opto_pin = gpiob.pb7.into_analog(); // opto-transistor temt6000
    let mut preview_pir_status = false;
    let mut timer_need_start = false;
    let mut timer_started = false;
    let mut led_on_status = false;

    pwm_pin.enable();
    let max = pwm_pin.get_max_duty();
    rprintln!("pwm {}", max);

    loop {
        let pir_status = pir.is_high().unwrap();
        let opto_data = adc.read_voltage(&mut opto_pin).unwrap();

        rprintln!("opto data {}", opto_data);

        if opto_data > 20 && led_on_status == false {
            preview_pir_status = false;
            timer_need_start = false;
            continue;
        }

        if pir_status {
            if preview_pir_status == false {

                rprintln!("pir up");
                led_on( &mut delay, &mut  pwm_pin );
                led_on_status = true;
                preview_pir_status = true;
                timer_need_start = true;
            }
            if timer_started {
                timer.reset();
            }
        } else if preview_pir_status{

            if timer_need_start {
                timer_started = true;
                timer.start(timeout.millis());
            }
            timer_need_start = false;

            if timer.wait().is_ok() {
                // новый цикл таймера
                rprintln!("timer is ok");
                led_off(&mut delay, &mut pwm_pin );
                led_on_status = false;
                timer.reset();
                timer.clear_irq();
                preview_pir_status = false;
            }
        }
    }
}

fn led_on (delay: &mut Delay<TIM14>,  pin: &mut PwmPin<TIM3, Channel3>) {
    let max =  pin.get_max_duty();
    rprintln!("led_on max {}", max);
    for pr in 0..100 {
        delay.delay(10.millis());
        pin.set_duty(max*pr/100);
    }
    pin.set_duty(max);
}

fn led_off ( delay:&mut Delay<TIM14>, pin:&mut  PwmPin<TIM3, Channel3>) {
    let max = pin.get_max_duty();
    rprintln!("led_off max {}", max);
    for pr in 0..100 {
        let re_pr = 100 - pr;
        delay.delay(10.millis());
        pin.set_duty(max*re_pr/100);
    }
    pin.set_duty(0);

}