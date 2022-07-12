#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_probe;

use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{asm, interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};
use stm32l0::stm32l0x1::interrupt;
use stm32l0xx_hal::exti::TriggerEdge::Rising;
use stm32l0xx_hal::{
    exti,
    exti::{Exti, ExtiLine, GpioLine, TriggerEdge},
    gpio::{
        gpioa::{PA3, PA9},
        Input, Output, PullDown, PushPull,
    },
    pac::{Interrupt, Peripherals, EXTI},
    prelude::*,
    rcc::Config,
    syscfg::SYSCFG,
};

static LED: Mutex<RefCell<Option<PA3<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static INTER: Mutex<RefCell<Option<Exti>>> = Mutex::new(RefCell::new(None));
static LINE_PIR: Mutex<RefCell<Option<GpioLine>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("start");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut pwm_pin = gpioa.pa3.into_push_pull_output();
    let mut signal = gpioa.pa5.into_push_pull_output();
    //signal.set_high();
    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);
    let mut exti = Exti::new(dp.EXTI);

    let mut pir = gpioa.pa9.into_pull_down_input();

    let line = GpioLine::from_raw_line(pir.pin_number()).unwrap();
    exti.listen_gpio(&mut syscfg, pir.port(), line, TriggerEdge::Rising);

    cortex_m::interrupt::free(move |cs| {
        *LED.borrow(cs).borrow_mut() = Some(pwm_pin);
        *INTER.borrow(cs).borrow_mut() = Some(exti);
    });

    unsafe {
        NVIC::unmask(Interrupt::EXTI4_15);
    }

    loop {
        asm::nop();
    }
}

#[interrupt]
fn EXTI4_15() {
    rprintln!("EXTI interrupt");

    cortex_m::interrupt::free(|cs| {
        if let (&mut Some(ref mut led), &mut Some(ref mut exti)) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            INTER.borrow(cs).borrow_mut().deref_mut(),
        ) {
            //let bm = 1 << line.raw_line();
            rprintln!(
                "EXTI {:?}",
                Exti::is_pending(GpioLine::from_raw_line(9).unwrap())
            );
            //let status_up = exti.is_pending(EXTI::Event::GPIO9, Rising);
            //if status_up {
            rprintln!("EXTI high");
            led.set_high().unwrap();
            // } else {
            //     rprintln!("EXTI low");
            //     led.set_low().unwrap();
            // }
            Exti::unpend(GpioLine::from_raw_line(9).unwrap());
        }
    });
}
