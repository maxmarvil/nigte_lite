#![no_std]
#![no_main]
//#![deny(warnings)]

//use cortex_m_semihosting::hprintln;

extern crate cortex_m_rt as rt;
extern crate panic_probe;
extern crate stm32g0xx_hal as hal;

use core::cell::RefCell;
use core::ops::DerefMut;
use cortex_m::asm;
use cortex_m::interrupt::Mutex;
use hal::analog::adc::{OversamplingRatio, Precision, SampleTime};
use hal::exti::Event;
use hal::gpio::SignalEdge;
use hal::stm32::EXTI;
use hal::{prelude::*, rcc::Config, stm32};
use rt::{entry, exception, ExceptionFrame};
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::interrupt;

static DEVICE_EXTI: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    let mut rcc = dp.RCC.freeze(Config::pll());
    let mut exti = dp.EXTI;

    let mut adc = dp.ADC.constrain(&mut rcc);
    let pwm = dp.TIM1.pwm(10.khz(), &mut rcc);
    let mut delay = dp.TIM3.delay(&mut rcc);

    adc.set_sample_time(SampleTime::T_80);
    adc.set_precision(Precision::B_12);
    adc.set_oversampling_ratio(OversamplingRatio::X_16);
    adc.set_oversampling_shift(16);
    adc.oversampling_enable(true);

    delay.delay(20.ms()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let mut pwm_ch1 = pwm.bind_pin(gpioa.pa8);
    let pir_pin = gpioa.pa6.into_pull_down_input();
    let max = pwm_ch1.get_max_duty();
    pwm_ch1.enable();
    gpioa.pa6.listen(SignalEdge::All, &mut exti);
    let mut opto_pin = gpioa.pa0.into_analog();

    cortex_m::interrupt::free(|cs| {
        *DEVICE_EXTI.borrow(cs).borrow_mut() = Some(exti);
        // *LED.borrow(cs).borrow_mut() = Some(led);
        // *TIMER.borrow(cs).borrow_mut() = Some(timer);
        // *COUNT_D.borrow(cs).borrow_mut() = Some(0);
        // *ADC.borrow(cs).borrow_mut() = Some(adc);
        // *OPTOPIN.borrow(cs).borrow_mut() = Some(opto_pin);
        // *SENSPIN.borrow(cs).borrow_mut() = Some(sens_pin);
    });
    //rprintln!("pwm max {}", max);

    loop {
        pwm_ch1.set_duty(0);
        rprintln!("pwm 0");
        for pr in 0..100 {
            //let re_pr = 100 -pr ;
            delay.delay(10.ms());
            let duty = (max as f32) * (pr as f32) / 100.0;
            pwm_ch1.set_duty(duty as u16);
        }
        pwm_ch1.set_duty(max);
        //rprintln!("pwm {}", max);
        delay.delay(200.ms());
        let opto_value = adc.read_voltage(&mut opto_pin).expect("adc read failed");
        rprintln!("opto {}", opto_value);
        for pr in 0..100 {
            let re_pr = 100 - pr;
            delay.delay(10.ms());
            let duty = (max as f32) * (re_pr as f32) / 100.0;
            pwm_ch1.set_duty(duty as u16);
        }
        pwm_ch1.set_duty(0);
        //rprintln!("pwm {}", 0);
        delay.delay(500.ms());
    }
}

#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        if let (&mut Some(ref mut exti)) = (DEVICE_EXTI.borrow(cs).borrow_mut().deref_mut()) {
            exti.unpend(Event::GPIO6);
        }
    });
}
