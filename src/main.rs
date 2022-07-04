#![no_main]
#![no_std]

use panic_semihosting as _;

use stm32f0xx_hal as hal;

use crate::hal::{pac, prelude::*, gpio::gpioa};
use pac::{interrupt, Interrupt, Peripherals, EXTI};
use cortex_m_rt::entry;
use cortex_m::{interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals};
use core::{cell::RefCell, ops::DerefMut};
use stm32f0xx_hal::gpio::{Output, PushPull};

static LED: Mutex<RefCell<Option<gpioa::PA6<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static INT: Mutex<RefCell<Option<EXTI>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {

    if let (Some(p), Some(cp)) = (Peripherals::take(), c_m_Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let rcc = p.RCC;
            rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

            let mut flash = p.FLASH;
            let mut rcc = rcc.configure().sysclk(8.mhz()).freeze(&mut flash);

            let gpioa = p.GPIOA.split(&mut rcc);
            let syscfg = p.SYSCFG;
            let exti = p.EXTI;
            // (Re-)configure PA1 as output
            let mut led = cortex_m::interrupt::free(move |cs| gpioa.pa6.into_push_pull_output(cs));

            // Set up a timer expiring after 1s
            //let timer = Timer::tim1(p.TIM1, Hertz(1), &mut rcc);
    // Turn off LED
            led.set_low().ok();

            // Enable external interrupt for PB1
            syscfg.exticr2.modify(|_, w| unsafe { w.exti4().bits(1) });

            // Set interrupt request mask for line 1
            exti.imr.modify(|_, w| w.mr1().set_bit());

            // Set interrupt rising trigger for line 1
            exti.rtsr.modify(|_, w| w.tr1().set_bit());

            // Move control over LED and DELAY and EXTI into global mutexes
            *LED.borrow(cs).borrow_mut() = Some(led);
            *INT.borrow(cs).borrow_mut() = Some(exti);
            // Enable EXTI IRQ, set prio 1 and clear any pending IRQs
            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::EXTI4_15, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI4_15);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI4_15);
        });
    }

    loop {
        continue;
    }
}

#[interrupt]
fn EXTI4_15() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Obtain all Mutex protected resources
        if let (&mut Some(ref mut led), &mut Some(ref mut exti)) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            INT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            // Turn on LED
            led.toggle().ok();

            // Turn off LED


            // Clear event triggering the interrupt
            exti.pr.write(|w| w.pr4().set_bit());
        }
    });
}