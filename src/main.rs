#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_probe;

use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{asm, interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32l0xx_hal::{
    //adc::{OversamplingRatio, Precision, SampleTime},
    exti::{Exti, ExtiLine, GpioLine, TriggerEdge},
    gpio::{
        gpioa::{PA4, PA9},
        Input, Output, PullDown, PushPull,
    },
    pac::{Interrupt, Peripherals},
    prelude::*,
    rcc::Config,
    syscfg::SYSCFG,
};

static LED: Mutex<RefCell<Option<PA4<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static PIR: Mutex<RefCell<Option<PA9<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[interrupt]
fn EXTI4_15() {
    rprintln!("EXTI interrupt");

    cortex_m::interrupt::free(|cs| {
        Exti::unpend(GpioLine::from_raw_line(9).unwrap());

        if let (&mut Some(ref mut led), &mut Some(ref mut pir)) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            PIR.borrow(cs).borrow_mut().deref_mut(),
        ) {
            if pir.is_low().is_ok() {
                led.set_low().unwrap();
            } else {
                led.set_high().unwrap();
            }
        }
    });
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut pwm_pin = gpioa.pa4.into_push_pull_output();

    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let mut exti = Exti::new(dp.EXTI);

    let mut pir = gpioa.pa9.into_pull_down_input();

    let line = GpioLine::from_raw_line(pir.pin_number()).unwrap();
    exti.listen_gpio(&mut syscfg, pir.port(), line, TriggerEdge::Both);

    cortex_m::interrupt::free(move |cs| {
        *LED.borrow(cs).borrow_mut() = Some(pwm_pin);
        *PIR.borrow(cs).borrow_mut() = Some(pir);
    });

    unsafe {
        NVIC::unmask(Interrupt::EXTI4_15);
    }

    loop {
        asm::wfi();
    }
}
