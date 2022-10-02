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
use hal::analog::adc::{Adc, OversamplingRatio, Precision, SampleTime};
use hal::exti::Event;
use hal::gpio::{
    gpioa::{PA0, PA1},
    Analog, SignalEdge,
};
use hal::stm32::{self, Interrupt, EXTI, NVIC, SYSCFG};
use hal::{prelude::*, rcc::Config, timer::Timer};
use rt::{entry, exception, ExceptionFrame};
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::interrupt;

static DEVICE_EXTI: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));
static LED_STATUS: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(Some(false)));
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM14>>>> = Mutex::new(RefCell::new(None));
static COUNT_D: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));
static ADC: Mutex<RefCell<Option<Adc>>> = Mutex::new(RefCell::new(None));
static OPTOPIN: Mutex<RefCell<Option<PA0<Analog>>>> = Mutex::new(RefCell::new(None));
static SENSPIN: Mutex<RefCell<Option<PA1<Analog>>>> = Mutex::new(RefCell::new(None));

static TIMEOUT: u8 = 4;

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    let mut rcc = dp.RCC.freeze(Config::pll());
    let mut exti = dp.EXTI;

    let mut adc = dp.ADC.constrain(&mut rcc);
    let pwm = dp.TIM1.pwm(10.khz(), &mut rcc);
    let mut delay = dp.TIM3.delay(&mut rcc);
    let mut timer = dp.TIM14.timer(&mut rcc);

    timer.start(1000.ms());
    timer.listen();

    adc.set_sample_time(SampleTime::T_80);
    adc.set_precision(Precision::B_12);
    adc.set_oversampling_ratio(OversamplingRatio::X_16);
    adc.set_oversampling_shift(16);
    adc.oversampling_enable(true);

    delay.delay(20.ms()); // Wait for ADC voltage regulator to stabilize
    adc.calibrate();

    let mut gpioa = dp.GPIOA.split(&mut rcc);

    let mut pwm_ch1 = pwm.bind_pin(gpioa.pa8);
    let pir_pin = gpioa.pa6.into_pull_down_input();
    let max = pwm_ch1.get_max_duty();
    pwm_ch1.enable();
    delay.delay(20.ms());
    pir_pin.listen(SignalEdge::All, &mut exti);
    delay.delay(20.ms());
    let mut opto_pin = gpioa.pa0.into_analog();
    pwm_ch1.set_duty(max);
    delay.delay(50.ms());
    pwm_ch1.set_duty(0);

    cortex_m::interrupt::free(|cs| {
        *DEVICE_EXTI.borrow(cs).borrow_mut() = Some(exti);
        // *LED.borrow(cs).borrow_mut() = Some(led);
        *TIMER.borrow(cs).borrow_mut() = Some(timer);
        *COUNT_D.borrow(cs).borrow_mut() = Some(0);
        // *ADC.borrow(cs).borrow_mut() = Some(adc);
        // *OPTOPIN.borrow(cs).borrow_mut() = Some(opto_pin);
        // *SENSPIN.borrow(cs).borrow_mut() = Some(sens_pin);
    });
    //rprintln!("pwm max {}", max);
    let mut set_led_on = false;
    let mut set_led_off = false;
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(Interrupt::EXTI4_15, 1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI4_15);
    }
    delay.delay(50.ms());
    loop {
        //pwm_ch1.set_duty(0);
        //rprintln!("pwm 0");

        // если включился пир (флаг включения светодиода сменился)
        // и если света мало - (с корректировкой)
        cortex_m::interrupt::free(|cs| {
            if let &mut Some(ref mut led_status) = LED_STATUS.borrow(cs).borrow_mut().deref_mut() {
                if pwm_ch1.get_duty() == 0 && *led_status {
                    set_led_on = true
                } else if pwm_ch1.get_duty() > 0 && *led_status == false {
                    set_led_off = true;
                }
            }
        });

        if set_led_on {
            for pr in 0..100 {
                //let re_pr = 100 -pr ;
                delay.delay(10.ms());
                let duty = (max as f32) * (pr as f32) / 100.0;
                pwm_ch1.set_duty(duty as u16);
            }
            pwm_ch1.set_duty(max);
        }

        if set_led_off {
            for pr in 0..100 {
                let re_pr = 100 - pr;
                delay.delay(10.ms());
                let duty = (max as f32) * (re_pr as f32) / 100.0;
                pwm_ch1.set_duty(duty as u16);
            }
            pwm_ch1.set_duty(0);
        }

        //rprintln!("pwm {}", max);
        delay.delay(200.ms());
        let opto_value = adc.read_voltage(&mut opto_pin).expect("adc read failed");
        rprintln!("opto {}", opto_value);

        //rprintln!("pwm {}", 0);
        //delay.delay(500.ms());
    }
}

#[interrupt]
fn EXTI4_15() {
    rprintln!("pir event");
    cortex_m::interrupt::free(|cs| {
        if let (&mut Some(ref mut exti), &mut Some(ref mut led_status), &mut Some(ref mut timer)) = (
            DEVICE_EXTI.borrow(cs).borrow_mut().deref_mut(),
            LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
            TIMER.borrow(cs).borrow_mut().deref_mut(),
        ) {
            use SignalEdge::*;
            let pir_up = exti.is_pending(Event::GPIO6, Rising);
            let pir_down = exti.is_pending(Event::GPIO6, Falling);
            rprintln!("pir event {} - {}", pir_up, pir_down);
            if pir_up && *led_status == false {
                rprintln!("pir up");
                *led_status = true;
                timer.unlisten();
                *COUNT_D.borrow(cs).borrow_mut() = Some(0);
            }

            if pir_down && *led_status {
                *led_status = false;
                timer.reset();
                timer.listen();
            }

            exti.unpend(Event::GPIO6);
        }
    });
}

#[interrupt]
fn TIM14() {
    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut timer), Some(ref mut count), Some(ref mut led_status)) = (
            TIMER.borrow(cs).borrow_mut().deref_mut(),
            COUNT_D.borrow(cs).borrow_mut().deref_mut(),
            LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
        ) {
            timer.clear_irq();

            *count = *count + 1;
            if *count > TIMEOUT && *led_status == true {
                timer.unlisten();
                *count = 0;
                *led_status = false;
            }
        }
    });
}
