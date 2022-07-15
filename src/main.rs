#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_probe;

use core::borrow::BorrowMut;
use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{asm, interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32l0::stm32l0x1::{interrupt, TIM2, TIM3};
use stm32l0xx_hal::{
    adc::{Adc, Ready},
    exti::{
        self, Exti, ExtiLine, GpioLine,
        TriggerEdge::{self, Falling, Rising},
    },
    gpio::{
        gpioa::{PA0, PA1, PA3, PA9},
        Analog, Input, Output, PullDown, PushPull,
    },
    pac::{self, Interrupt, Peripherals, EXTI},
    prelude::*,
    pwm::{self, Assigned, Pwm, C4},
    rcc::Config,
    syscfg::SYSCFG,
    timer::Timer,
};

static LED: Mutex<RefCell<Option<Pwm<TIM2, C4, Assigned<PA3<Analog>>>>>> =
    Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM3>>>> = Mutex::new(RefCell::new(None));
static COUNT_D: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));
static ADC: Mutex<RefCell<Option<Adc<Ready>>>> = Mutex::new(RefCell::new(None));
static OPTOPIN: Mutex<RefCell<Option<PA1<Analog>>>> = Mutex::new(RefCell::new(None));
static SHIFT: Mutex<RefCell<Option<u16>>> = Mutex::new(RefCell::new(None));
static LED_STATUS: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(Some(false)));
static TIMEOUT: u8 = 4;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut signal = gpioa.pa5.into_push_pull_output();

    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let mut exti = Exti::new(dp.EXTI);
    let mut timer = dp.TIM3.timer(1.Hz(), &mut rcc);
    let mut adc = dp.ADC.constrain(&mut rcc);
    let mut opto_pin = gpioa.pa1.into_analog();
    let mut sens_pin = gpioa.pa0.into_analog();
    let pwm = pwm::Timer::new(dp.TIM2, 1_000.Hz(), &mut rcc);
    let mut pwm_pin: Pwm<TIM2, C4, Assigned<PA3<Analog>>> = pwm.channel4.assign(gpioa.pa3);
    pwm_pin.enable();

    let correcting: u16 = adc.read(&mut sens_pin).unwrap();

    let mut pir = gpioa.pa9.into_pull_down_input();
    let mut pir2 = gpioa.pa10.into_pull_down_input();

    let line = GpioLine::from_raw_line(pir.pin_number()).unwrap();
    let line2 = GpioLine::from_raw_line(pir2.pin_number()).unwrap();

    exti.listen_gpio(&mut syscfg, pir.port(), line, Rising);
    exti.listen_gpio(&mut syscfg, pir2.port(), line2, Falling);

    cortex_m::interrupt::free(move |cs| {
        *LED.borrow(cs).borrow_mut() = Some(pwm_pin);
        *TIMER.borrow(cs).borrow_mut() = Some(timer);
        *COUNT_D.borrow(cs).borrow_mut() = Some(0);
        *ADC.borrow(cs).borrow_mut() = Some(adc);
        *OPTOPIN.borrow(cs).borrow_mut() = Some(opto_pin);
        *SHIFT.borrow(cs).borrow_mut() = Some(correcting);
    });

    unsafe {
        NVIC::unmask(Interrupt::EXTI4_15);
        NVIC::unmask(Interrupt::TIM2);
    }

    loop {
        asm::nop();
    }
}

#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        if let (
            &mut Some(ref mut led),
            &mut Some(ref mut timer),
            &mut Some(ref mut adc),
            &mut Some(ref mut opto_pin),
            &mut Some(ref mut correcting),
            &mut Some(ref mut led_status),
        ) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            TIMER.borrow(cs).borrow_mut().deref_mut(),
            ADC.borrow(cs).borrow_mut().deref_mut(),
            OPTOPIN.borrow(cs).borrow_mut().deref_mut(),
            SHIFT.borrow(cs).borrow_mut().deref_mut(),
            LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
            //SENSPIN.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let event_risen = Exti::is_pending(GpioLine::from_raw_line(9).unwrap());
            let event_falling = Exti::is_pending(GpioLine::from_raw_line(10).unwrap());
            let mut val: u16 = adc.read(opto_pin).unwrap();
            let mut index: f32 = (*correcting as f32) / 4095.0;
            let mut val_new = (val as f32) * index;
            rprintln!("opto {} ", (val_new as u32));
            if event_risen && val_new < 100.0 && *led_status == false {
                // && val < 200
                rprintln!("EXTI high");

                timer.unlisten();
                *COUNT_D.borrow(cs).borrow_mut() = Some(0);
                *LED_STATUS.borrow(cs).borrow_mut() = Some(true);
                //led.set_high().unwrap();
                let max_value = led.get_max_duty();
                led.set_duty(max_value)
            }
            if event_falling && *led_status == true {
                rprintln!("EXTI low");
                timer.reset();
                timer.listen();
            }
            Exti::unpend(GpioLine::from_raw_line(9).unwrap());
            Exti::unpend(GpioLine::from_raw_line(10).unwrap());
        }
    });
}

#[interrupt]
fn TIM2() {
    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut timer), Some(ref mut count), Some(ref mut led_status)) = (
            TIMER.borrow(cs).borrow_mut().deref_mut(),
            COUNT_D.borrow(cs).borrow_mut().deref_mut(),
            LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
        ) {
            // Clear the interrupt flag.
            timer.clear_irq();

            *count = *count + 1;
            rprintln!("Timer catch {}", count);
            if *count > TIMEOUT && *led_status == true {
                timer.unlisten();
                *count = 0;
                *LED_STATUS.borrow(cs).borrow_mut() = Some(false);
                //*COUNT_D.borrow(cs).borrow_mut() = Some(0);
                // Change the LED state on each interrupt.
                if let Some(ref mut led) = LED.borrow(cs).borrow_mut().deref_mut() {
                    //let max_value = led.get_max_duty();
                    led.set_duty(0)
                }
            }
        }
    });
}
