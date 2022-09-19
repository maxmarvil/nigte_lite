#![no_std]
#![no_main]
//#![deny(warnings)]

extern crate panic_probe;

use core::{cell::RefCell, ops::DerefMut};
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
use cortex_m_rt::entry;
use stm32l0::stm32l0x1::{interrupt, TIM2};
use stm32l0xx_hal::gpio::{Output, PushPull};
use stm32l0xx_hal::{
    adc::{Adc, Ready},
    exti::{Exti, ExtiLine, GpioLine, TriggerEdge},
    gpio::{
        gpioa::{PA0, PA1, PA3},
        Analog,
    },
    pac::{self, Interrupt},
    prelude::*,
    pwm::{self, Assigned, Pwm, C4},
    pwr::{self, PWR},
    rcc::Config,
    syscfg::SYSCFG,
    timer::Timer,
};

static LED: Mutex<RefCell<Option<PA3<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
// static LED: Mutex<RefCell<Option<Pwm<TIM2, C4, Assigned<PA3<Analog>>>>>> =
//     Mutex::new(RefCell::new(None));
static LED_STATUS: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(Some(false)));
static COUNT_D: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer<pac::TIM22>>>> = Mutex::new(RefCell::new(None));
static ADC: Mutex<RefCell<Option<Adc<Ready>>>> = Mutex::new(RefCell::new(None));
static OPTOPIN: Mutex<RefCell<Option<PA1<Analog>>>> = Mutex::new(RefCell::new(None));
static SENSPIN: Mutex<RefCell<Option<PA0<Analog>>>> = Mutex::new(RefCell::new(None));
static SHIFT: Mutex<RefCell<Option<u16>>> = Mutex::new(RefCell::new(None));

static TIMEOUT: u8 = 4;

#[entry]
fn main() -> ! {
    let cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    //let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let mut delay = cp.SYST.delay(rcc.clocks);

    let mut led = gpioa.pa3.into_push_pull_output();

    let mut syscfg = SYSCFG::new(dp.SYSCFG, &mut rcc);

    led.set_high().unwrap();
    delay.delay_ms(2000u32);
    led.set_low().unwrap();

    loop {}
}
