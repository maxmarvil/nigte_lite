#![no_std]
#![no_main]
#![deny(warnings)]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::{ prelude::*, stm32, analog::adc::{OversamplingRatio, Precision, SampleTime}};
use stm32g0xx_hal::rcc::{Config, Prescaler};
//use stm32g0xx_hal::rcc::SysClockSrc;
use stm32g0xx_hal::stm32::TIM1;
use stm32g0xx_hal::stm32::TIM14;
use stm32g0xx_hal::timer::Channel2;
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

    delay.delay(20.us()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let timeout = 5000;
    let mut timer = dp.TIM16.timer(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let pwm = dp.TIM1.pwm(10.khz(), &mut rcc);

    //let gpioc = dp.GPIOC.split(&mut rcc);
    let mut pwm_pin = pwm.bind_pin(gpiob.pb0);
    //let mut led = gpioa.pa11.into_push_pull_output();
    let pir = gpioa.pa12.into_pull_down_input();
    //let mut opto_pin = gpiob.pb7.into_analog();
    //pir.listen(SignalEdge.Rising, );
    let mut preview_pir_status = false;
    let mut timer_need_start = false;
    let mut timer_started = false;
    //timer.start(timeout.ms());

    let max = pwm_pin.get_max_duty();
    rprintln!("pwm {}", max);
    //let mut opto_pin = gpioc.pc14.into_analog();
    loop {
        let pir_status = pir.is_high().unwrap();
        //let opto_data = adc.read_voltage(&mut opto_pin).unwrap();

        // if opto_data > 20 {
        //     //preview_pir_status = false;
        //     //continue;
        // }
        //rprintln!("opto data {}", opto_data);
        if pir_status {
            if preview_pir_status == false {

                rprintln!("pir up");
                led_on( &mut delay, &mut  pwm_pin );
                //led.set_high().unwrap();
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

            if timer.wait().is_ok() {
                // новый цикл таймера
                rprintln!("timer is ok");
                led_off(&mut delay, &mut pwm_pin );
                //led.set_low().unwrap();

                timer.reset();
                timer.clear_irq();
                preview_pir_status = false;
            }
        }
    }
}

fn led_on (delay: &mut Delay<TIM14>,  pin: &mut PwmPin<TIM1, Channel2>) {
    let max:u16 =  pin.get_max_duty();
    rprintln!("led_on max {}", max);
    delay.delay(5.ms());
    for pr in 0..100 {
        delay.delay(10.ms());
        rprintln!("led_on duty {}", (max/100)*pr);
        pin.set_duty((max/100)*pr);
    }
}

fn led_off ( delay:&mut Delay<TIM14>, pin:&mut  PwmPin<TIM1, Channel2>) {
    let max:u16 = pin.get_max_duty();
    rprintln!("led_off max {}", max);
    delay.delay(5.ms());
    for pr in 0..100 {
        let re_pr = 100 - pr;
        delay.delay(10.ms());
        rprintln!("led_off duty {}", (max/100)*re_pr);
        pin.set_duty((max/100)*re_pr);
    }

}