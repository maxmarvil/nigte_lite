#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_probe;

use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32l0::stm32l0x1::{interrupt, TIM2};
use stm32l0xx_hal::{
    adc::{Adc, Ready},
    exti::{Exti, ExtiLine, GpioLine, TriggerEdge},
    gpio::{
        gpioa::{PA0, PA1, PA2, PA3}, // ,PA5
        Analog,
    },
    pac::{self, Interrupt},
    prelude::*,
    pwm::{self, Assigned, Pwm, C3},
    pwr::{self, PWR},
    rcc::Config,
    syscfg::SYSCFG,
    timer::Timer,
};

static LED: Mutex<RefCell<Option<Pwm<TIM2, C3, Assigned<PA2<Analog>>>>>> =
    Mutex::new(RefCell::new(None));
static LED_STATUS: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(Some(false)));
static COUNT_D: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM22>>>> = Mutex::new(RefCell::new(None));
static ADC: Mutex<RefCell<Option<Adc<Ready>>>> = Mutex::new(RefCell::new(None));
static OPTOPIN: Mutex<RefCell<Option<PA0<Analog>>>> = Mutex::new(RefCell::new(None));
static SENSPIN: Mutex<RefCell<Option<PA1<Analog>>>> = Mutex::new(RefCell::new(None));
static SHIFT: Mutex<RefCell<Option<u16>>> = Mutex::new(RefCell::new(None));

static TIMEOUT: u8 = 4;

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    rtt_init_print!();
    rprintln!("start");
    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led_red = gpioa.pa5.into_push_pull_output();
    led_red.set_high();
    let mut exti = Exti::new(dp.EXTI);
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let mut delay = cp.SYST.delay(rcc.clocks);
    let timer = dp.TIM22.timer(1.Hz(), &mut rcc);
    let adc = dp.ADC.constrain(&mut rcc);
    let opto_pin = gpioa.pa0.into_analog();
    let sens_pin = gpioa.pa1.into_analog();
    let mut scb = cp.SCB;
    let mut light_status = false;

    rprintln!("pre-init");
    // Those are the user button and blue LED on the B-L072Z-LRWAN1 Discovery
    // board.
    let trigger_up = gpioa.pa9.into_floating_input();

    let pir2 = gpioa.pa10.into_pull_down_input();
    let pwm = pwm::Timer::new(dp.TIM2, 1_000.Hz(), &mut rcc);
    let mut led: Pwm<TIM2, C3, Assigned<PA2<Analog>>> = pwm.channel3.assign(gpioa.pa2);
    led.enable();
    //led.set_duty(0);

    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);

    let line = GpioLine::from_raw_line(trigger_up.pin_number()).unwrap();
    let line2 = GpioLine::from_raw_line(pir2.pin_number()).unwrap();

    exti.listen_gpio(&mut syscfg, trigger_up.port(), line, TriggerEdge::Rising);

    cortex_m::interrupt::free(|cs| {
        *LED.borrow(cs).borrow_mut() = Some(led);
        *TIMER.borrow(cs).borrow_mut() = Some(timer);
        *COUNT_D.borrow(cs).borrow_mut() = Some(0);
        *ADC.borrow(cs).borrow_mut() = Some(adc);
        *OPTOPIN.borrow(cs).borrow_mut() = Some(opto_pin);
        *SENSPIN.borrow(cs).borrow_mut() = Some(sens_pin);
    });

    delay.delay_ms(5000u32);

    loop {
        if light_status == false {
            delay.delay_ms(100u32);
            //led_red.set_high().unwrap();
            delay.delay_ms(100u32);
            //led_red.set_low().unwrap();

            exti.wait_for_irq(
                line,
                pwr.stop_mode(
                    &mut scb,
                    &mut rcc,
                    pwr::StopModeConfig {
                        ultra_low_power: true,
                    },
                ),
            );
            cortex_m::interrupt::free(|cs| {
                if let (&mut Some(ref mut adc), &mut Some(ref mut sens_pin)) = (
                    ADC.borrow(cs).borrow_mut().deref_mut(),
                    SENSPIN.borrow(cs).borrow_mut().deref_mut(),
                ) {
                    let correcting: u16 = adc.read(sens_pin).unwrap();
                    *SHIFT.borrow(cs).borrow_mut() = Some(correcting);
                }
            });

            exti.listen_gpio(&mut syscfg, pir2.port(), line2, TriggerEdge::Falling);
            unsafe {
                NVIC::unmask(Interrupt::EXTI4_15);
                NVIC::unmask(Interrupt::TIM22);
            }

            delay.delay_ms(1u32);

            cortex_m::interrupt::free(|cs| {
                if let (
                    &mut Some(ref mut led),
                    &mut Some(ref mut led_status),
                    &mut Some(ref mut adc),
                    &mut Some(ref mut opto_pin),
                    &mut Some(ref mut correcting),
                ) = (
                    LED.borrow(cs).borrow_mut().deref_mut(),
                    LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
                    ADC.borrow(cs).borrow_mut().deref_mut(),
                    OPTOPIN.borrow(cs).borrow_mut().deref_mut(),
                    SHIFT.borrow(cs).borrow_mut().deref_mut(),
                ) {
                    let val: u16 = adc.read(opto_pin).unwrap();
                    let index: f32 = (*correcting as f32) / 4095.0;
                    let val_new = (val as f32) * index;

                    if val_new < 100.0 {
                        let max_value = led.get_max_duty();
                        for pr in 0..100 {
                            delay.delay_ms(10u32);
                            let duty = (max_value as f32) * (pr as f32) / 100.0;
                            led.set_duty(duty as u16);
                            //rprintln!("on {}", (pr as f32)); //((max_value as f32) * (pr as f32) / 100.0)
                        }
                        led.set_duty(max_value);

                        light_status = true;
                        *led_status = true;
                    }
                }
            });
        }

        delay.delay_ms(5u32);

        cortex_m::interrupt::free(|cs| {
            if let (&mut Some(ref mut led), &mut Some(ref mut led_status)) = (
                LED.borrow(cs).borrow_mut().deref_mut(),
                LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
            ) {
                if *led_status == false && light_status == true {
                    //led.set_low().unwrap();
                    let max_value = led.get_max_duty();
                    for pr in 0..100 {
                        let re_pr = 100 - pr;
                        delay.delay_ms(10u32);
                        let duty = (max_value as f32) * (re_pr as f32) / 100.0;
                        led.set_duty(duty as u16);
                    }
                    led.set_duty(0);
                    light_status = false;
                }
            }
        });
    }
}

#[interrupt]
fn EXTI4_15() {
    cortex_m::interrupt::free(|cs| {
        if let (&mut Some(ref mut timer), &mut Some(ref mut led_status)) = (
            TIMER.borrow(cs).borrow_mut().deref_mut(),
            LED_STATUS.borrow(cs).borrow_mut().deref_mut(),
        ) {
            let event_risen = Exti::is_pending(GpioLine::from_raw_line(9).unwrap());
            let event_falling = Exti::is_pending(GpioLine::from_raw_line(10).unwrap());

            if event_risen && *led_status {
                *led_status = true;
                timer.unlisten();
                *COUNT_D.borrow(cs).borrow_mut() = Some(0);
            }

            if event_falling && *led_status {
                timer.reset();
                timer.listen();
            }
            Exti::unpend(GpioLine::from_raw_line(9).unwrap());
            Exti::unpend(GpioLine::from_raw_line(10).unwrap());
        }
    });
}

#[interrupt]
fn TIM22() {
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
