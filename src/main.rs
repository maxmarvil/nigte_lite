#![no_std]
#![no_main]
#![deny(warnings)]

//use cortex_m_semihosting::hprintln;

extern crate panic_semihosting;

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [CEC])]
mod app {
    use stm32g0xx_hal as hal;
    use cortex_m::asm;
    use hal::exti;
    //use hal::cortex_m::asm::delay;
    //use hal::exti::Event;
    //use hal::gpio::gpioa::{ PA12};
    use hal::prelude::*;
    use hal::stm32;
    use hal::stm32::SYST;
    use hal::timer::{pwm, Timer, Channel4, delay::Delay};
    use hal::gpio::{ SignalEdge};
    use rtt_target::{rprintln, rtt_init_print};

    #[shared]
    struct Shared {
        pir_status: bool,
        trigger: Option<SignalEdge>,
        tim_ower: Timer<stm32::TIM17>,
        led: pwm::PwmPin<stm32::TIM1, Channel4>,//PA12<Output<PushPull>>,
        pwm_max: u16,
    }

    #[local]
    struct Local {
        exti: stm32::EXTI,
        delay: Delay<SYST>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.constrain();
        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        //let gpioc = ctx.device.GPIOC.split(&mut rcc);
        let pir = gpioa.pa12.into_pull_down_input();
        let mut tim_ower = ctx.device.TIM17.timer(&mut rcc);
        let pwm = ctx.device.TIM1.pwm(10.khz(), &mut rcc);
        let trigger = None;
        let pwm_ch4 = pwm.bind_pin(gpioa.pa11);
        let delay = ctx.core.SYST.delay(&mut rcc);

        rtt_init_print!();
        tim_ower.listen();
        rprintln!("start");
        let mut exti = ctx.device.EXTI;
        pir.listen(SignalEdge::All, &mut exti);
        let pwm_max = pwm_ch4.get_max_duty();

        (
            Shared {
                pir_status: false,
                trigger,
                tim_ower,
                led: pwm_ch4,
                pwm_max,
            },
            Local {
                exti,
                delay
            },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM17, priority = 3, local = [], shared=[tim_ower, pir_status])]
    fn timer_tick(ctx: timer_tick::Context) {
        let mut pir_status = ctx.shared.pir_status;
        let mut tim_ower = ctx.shared.tim_ower;
        let status  = pir_status.lock(|pir_stat| *pir_stat);

        rprintln!("timer pir status {}", status);

        if status {
            tim_ower.lock(|tim_ower| {
                tim_ower.reset();
                //tim_ower.start(4000.ms());
            });

        } else {
            tim_ower.lock(|tim_ower| {
                fade_out::spawn().unwrap();
                tim_ower.reset();
                tim_ower.clear_irq();
            });
        }
    }

    #[task(binds = EXTI4_15, priority = 2, local = [exti], shared=[pir_status, tim_ower, trigger])]
    fn pir_signal(ctx: pir_signal::Context) {
        //let mut trigger = ctx.shared.trigger;
        let mut pir_status = ctx.shared.pir_status;
        let exti = ctx.local.exti;
        let mut tim_ower = ctx.shared.tim_ower;
        use SignalEdge::*;

        // let status = trigger.lock(|trigger|
        //     {
        //         let status = match trigger {
        //             Some(Falling) => exti.is_pending(exti::Event::GPIO11, Falling),
        //             Some(Rising) => exti.is_pending(exti::Event::GPIO11, Rising),
        //             _ => false,
        //         };
        //
        //         status
        //     });
        let status = exti.is_pending(exti::Event::GPIO12, Rising);
        rprintln!("exti pir status {}", status);
        pir_status.lock(|pir_status|{
            *pir_status = status;
        });


        (tim_ower).lock(|tim_ower| {
            if status {
                fade_in::spawn().unwrap();
            } else {
                tim_ower.start(6000.ms());
            }
        });
        exti.unpend(exti::Event::GPIO12);
    }

    #[task(priority = 4, local=[delay], shared=[led, pwm_max])]
    fn fade_in(ctx: fade_in::Context) {
        let mut led = ctx.shared.led;
        //let delay = ctx.local.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);

        for val in 0 .. 100 {
            //delay.delay(2000.ms());
            ( led).lock(|led| {
                led.set_duty((max*val) / 100);
            });
        }
    }

    #[task(priority = 4, shared=[led, pwm_max])]
    fn fade_out(ctx: fade_out::Context) {
        let mut led = ctx.shared.led;
        //let delay = ctx.local.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);
        //delay.delay(100.ms());

        for  val in 100 .. 0 {
            ( led).lock(|led| {
               led.set_duty((max*val) / 100);
            });
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
            //cortex_m::asm::wfi();
        }
    }
}