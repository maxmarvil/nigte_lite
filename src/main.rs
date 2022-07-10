#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_semihosting;

use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32g0xx_hal::{
    analog::adc::{OversamplingRatio, Precision, SampleTime},
    exti::Event,
    gpio::{
        gpioa::{self, PA12},
        gpiob::{self, PB0},
        Output, PushPull, SignalEdge,
    },
    pac::{self, Peripherals, EXTI},
    prelude::*,
    rcc::{Config, Prescaler},
    stm32::{self, interrupt, Interrupt, TIM14, TIM3},
    timer::{delay::Delay, pwm::PwmPin, Channel3},
};
//use stm32g0xx_hal::rcc::SysClockSrc;
use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals};

static LED: Mutex<RefCell<Option<gpiob::PB0<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static INT: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi(Prescaler::NotDivided)); //Config::new(SysClockSrc::PLL)
                                                                     //let mut adc = dp.ADC.constrain(&mut rcc);
    let mut delay = dp.TIM14.delay(&mut rcc);

    // adc.set_sample_time(SampleTime::T_80);
    // adc.set_precision(Precision::B_12);
    // adc.set_oversampling_ratio(OversamplingRatio::X_16);
    // adc.set_oversampling_shift(16);
    // adc.oversampling_enable(true);

    delay.delay(20.micros()); // Wait for ADC voltage regulator to stabilize
                              //adc.calibrate();

    let timeout = 5000;
    //let mut timer = dp.TIM16.timer(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut pwm_pin = gpiob.pb0.into_push_pull_output();
    //pwm_pin.enable();
    let mut exti = dp.EXTI;
    let syscfg = dp.SYSCFG;

    //let mut pwm_pin = pwm.bind_pin(gpiob.pb0);
    pwm_pin.set_low();
    let mut pir = gpioa.pa12.into_pull_down_input();
    // let pwm = dp.TIM3.pwm(1.kHz(), &mut rcc);

    // let mut opto_pin = gpiob.pb7.into_analog(); // opto-transistor temt6000
    // let mut preview_pir_status = false;
    // let mut timer_need_start = false;
    // let mut timer_started = false;
    // let mut led_on_status = false;
    //pir.listen(SignalEdge::Rising, &mut exti);
    exti.exticr4.modify(|r, w| unsafe { w.exti8_15().bits(1) });
    exti.imr1.modify(|_, w| w.im12().set_bit());
    exti.rtsr1.modify(|_, w| w.tr12().set_bit());
    exti.ftsr1.modify(|_, w| w.tr12().set_bit());

    cortex_m::interrupt::free(move |cs| {
        //syscfg.cfgr2exticr2.modify(|_, w| unsafe { w.exti4().bits(1) });

        *LED.borrow(cs).borrow_mut() = Some(pwm_pin);
        *INT.borrow(cs).borrow_mut() = Some(exti);
    });
    //let max = pwm_pin.get_max_duty();
    //rprintln!("pwm {}", max);
    //let mut nvic = cp.NVIC;
    //unsafe {
    //nvic.set_priority(Interrupt::EXTI4_15, 1);
    //cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI4_15);
    //}

    loop {
        continue;

        // let pir_status = pir.is_high().unwrap();
        // let opto_data = adc.read_voltage(&mut opto_pin).unwrap();
        //
        // //rprintln!("opto data {}", opto_data);
        //
        // if opto_data > 20 && led_on_status == false {
        //     preview_pir_status = false;
        //     timer_need_start = false;
        //     continue;
        // }
        //
        // if pir_status {
        //     if preview_pir_status == false {
        //         rprintln!("pir up");
        //         led_on(&mut delay, &mut pwm_pin);
        //         led_on_status = true;
        //         preview_pir_status = true;
        //         timer_need_start = true;
        //     }
        //     if timer_started {
        //         timer.reset();
        //     }
        // } else if preview_pir_status {
        //     if timer_need_start {
        //         timer_started = true;
        //         timer.start(timeout.millis());
        //     }
        //     timer_need_start = false;
        //
        //     if timer.wait().is_ok() {
        //         // новый цикл таймера
        //         rprintln!("timer is ok");
        //         led_off(&mut delay, &mut pwm_pin);
        //         led_on_status = false;
        //         timer.reset();
        //         timer.clear_irq();
        //         preview_pir_status = false;
        //     }
        // }
    }
}
//
// fn led_on(delay: &mut Delay<TIM14>, pin: &mut PwmPin<TIM3, Channel3>) {
//     let max = pin.get_max_duty();
//     rprintln!("led_on");
//     for pr in 0..100 {
//         delay.delay(10.millis());
//         pin.set_duty(max * pr / 100);
//     }
//     pin.set_duty(max);
// }
//
// fn led_off(delay: &mut Delay<TIM14>, pin: &mut PwmPin<TIM3, Channel3>) {
//     let max = pin.get_max_duty();
//     rprintln!("led_off");
//     for pr in 0..100 {
//         let re_pr = 100 - pr;
//         delay.delay(10.millis());
//         pin.set_duty(max * re_pr / 100);
//     }
//     pin.set_duty(0);
// }

#[interrupt]
fn EXTI4_15() {
    rprintln!("EXTI interrupt");
    cortex_m::interrupt::free(|cs| {
        // Obtain all Mutex protected resources
        if let (&mut Some(ref mut led), &mut Some(ref mut exti)) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            INT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            // Turn on LED
            led.toggle().ok();

            // Turn off LED
            rprintln!("EXTI take");

            // Clear event triggering the interrupt
            exti.unpend(Event::GPIO13);
        }
    });
}
