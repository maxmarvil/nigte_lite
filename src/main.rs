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
    use hal::stm32::TIM14;
    use hal::timer::{pwm, Channel3, delay::Delay};
    use hal::gpio::{ SignalEdge};
    use rtt_target::{rprintln, rtt_init_print};
    use systick_monotonic::{ fugit::Duration, Systick};//

    #[shared]
    struct Shared {
        pir_status: bool,
        trigger: Option<SignalEdge>,
        //tim_ower: Timer<stm32::TIM17>,
        led: pwm::PwmPin<stm32::TIM3, Channel3>,//PA12<Output<PushPull>>,
        pwm_max: u32,
        delay: Delay<TIM14>,
        led_status: bool,
        fade_out_handle: Option<fade_out::SpawnHandle>,
        fade_order: bool,
    }

    #[local]
    struct Local {
        exti: stm32::EXTI,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.constrain();

            let mono = Systick::new(ctx.core.SYST, 36_000_000);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let pir = gpioa.pa12.into_pull_down_input();
        let mut tim_ower = ctx.device.TIM17.timer(&mut rcc);
        let pwm = ctx.device.TIM3.pwm(1.khz(), &mut rcc);
        let trigger = None;
        let mut pwm_ch2 = pwm.bind_pin(gpiob.pb0);
        let delay = ctx.device.TIM14.delay(&mut rcc);

        rtt_init_print!();
        tim_ower.listen();
        rprintln!("start");
        let mut exti = ctx.device.EXTI;
        pwm_ch2.enable();
        pwm_ch2.set_duty(0);
        pir.listen(SignalEdge::All, &mut exti);
        let pwm_max = pwm_ch2.get_max_duty();
        //let handler = fade_out::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();

        (
            Shared {
                pir_status: false,
                trigger,
                led: pwm_ch2,
                pwm_max,
                delay,
                led_status: false,
                fade_out_handle: None,
                fade_order: false
            },
            Local {
                exti,
            },
            init::Monotonics(mono),
        )
    }

    // #[task(binds = TIM17, priority = 3, local = [], shared=[tim_ower, pir_status])]
    // fn timer_tick(ctx: timer_tick::Context) {
    //     let mut pir_status = ctx.shared.pir_status;
    //     let mut tim_ower = ctx.shared.tim_ower;
    //     let status  = pir_status.lock(|pir_stat| *pir_stat);
    //
    //     rprintln!("timer pir status {}", status);
    //
    //     if status {
    //         (tim_ower).lock(|tim_ower| {
    //             rprintln!("timer reset");
    //             tim_ower.reset();
    //             //tim_ower.start(4000.ms());
    //         });
    //
    //     } else {
    //         (tim_ower).lock(|tim_ower| {
    //             rprintln!("timer reset and clear");
    //             fade_out::spawn().unwrap();
    //             tim_ower.reset();
    //             tim_ower.clear_irq();
    //         });
    //     }
    // }

    #[task(binds = EXTI4_15, priority = 2, local = [exti], shared=[pir_status, fade_out_handle, fade_order])]
    fn pir_signal(ctx: pir_signal::Context) {
        //let mut trigger = ctx.shared.trigger;
        let mut pir_status = ctx.shared.pir_status;
        let mut fade_out_handle = ctx.shared.fade_out_handle;
        let mut fade_order = ctx.shared.fade_order;
        let fade_order_status = fade_order.lock(|fade_order| *fade_order);
        let exti = ctx.local.exti;
        use SignalEdge::*;

        let status = exti.is_pending(exti::Event::GPIO12, Rising);
        let status_fall = exti.is_pending(exti::Event::GPIO12, Falling);

        rprintln!("exti pir status {}", status);
        pir_status.lock(|pir_status|{
            *pir_status = status;
        });

        if status {

            fade_in::spawn().unwrap();
            if fade_order_status {
                fade_out_handle.lock(|fade_out_h| {
                    rprintln!("fade_out_handle ");
                    match fade_out_h.take() {
                        Some(handler) => {
                            rprintln!("fade_out_handle some take {:?}",handler);
                            handler.cancel().unwrap();
                        },
                        None => rprintln!("free handler"),
                    }
                });
            }
            fade_order.lock(|fade_order| { *fade_order = false; } );

        } else if status_fall {
            let handler_new = Some(fade_out::spawn_after(Duration::<u64, 1, 1000>::from_ticks(5000)).unwrap());
            (fade_out_handle, fade_order).lock(|fade_out_h, order| {
                *fade_out_h = handler_new;
                *order = true;
            });
        }

        exti.unpend(exti::Event::GPIO12);
    }

    #[task(priority = 4, shared=[led, pwm_max, delay, led_status])]
    fn fade_in(ctx: fade_in::Context) {
        let mut led = ctx.shared.led;
        let mut led_status = ctx.shared.led_status;
        let mut delay = ctx.shared.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);
        let status  = led_status.lock(|led_status| *led_status);
        rprintln!("led on");
        if status == false {
            for val in 0 .. 100 {
                delay.lock(|delay| delay.delay(10.ms()));
                ( led).lock(|led| {
                    led.set_duty(max*val/100);
                });
            }
            ( led).lock(|led| {
                led.set_duty(max);
            });
            led_status.lock(|led_status|{
                *led_status = true;
            });
        }
    }

    #[task(priority = 4,  shared=[led, pwm_max, delay, led_status])]
    fn fade_out(ctx: fade_out::Context) {
        let mut led = ctx.shared.led;
        let mut led_status = ctx.shared.led_status;
        let mut delay = ctx.shared.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);
        let status  = led_status.lock(|led_status| *led_status);

        rprintln!("led off");
        if status {
            for val in 0..100 {
                delay.lock(|delay| delay.delay(10.ms()));
                let re_pr = 100 - val;
                (led).lock(|led| {
                    led.set_duty(max * re_pr / 100);
                });
            }
            (led).lock(|led| {
                led.set_duty(0);
            });
            led_status.lock(|led_status|{
                *led_status = false;
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