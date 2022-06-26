#![no_std]
#![no_main]
#![deny(warnings)]

//use cortex_m_semihosting::hprintln;

extern crate panic_semihosting;

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [CEC])]
mod app {
    use stm32g0xx_hal as hal;
    use cortex_m::asm;
    //use hal::hal::adc::Channel;
    use hal::{prelude::*, stm32, stm32::TIM14, exti,
              timer::{pwm, Channel3, delay::Delay},
              gpio::{ SignalEdge,gpiob::PB7},//,

              analog::adc::{ OversamplingRatio, Precision, SampleTime, Adc, }};
    use rtt_target::{rprintln, rtt_init_print};
    use stm32g0xx_hal::gpio::Analog;
    use systick_monotonic::{ fugit::Duration, Systick};


    #[shared]
    struct Shared {
        pir_status: bool,
        //trigger: Option<SignalEdge>,
        //tim_ower: Timer<stm32::TIM17>,
        led: pwm::PwmPin<stm32::TIM3, Channel3>,//PA12<Output<PushPull>>,
        pwm_max: u32,
        delay: Delay<TIM14>,
        led_status: bool,
        fade_out_handle: Option<fade_out::SpawnHandle>,
        fade_order: bool,
        adc: Adc,
        opto_pin: PB7<Analog>,//Channel<Adc>,
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

        let mono = Systick::new(ctx.core.SYST, 16_000_000);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let pir = gpioa.pa12.into_pull_down_input();
        let mut tim_ower = ctx.device.TIM17.timer(&mut rcc);
        let pwm = ctx.device.TIM3.pwm(1.kHz(), &mut rcc);
        //let trigger = None;
        let mut pwm_ch2 = pwm.bind_pin(gpiob.pb0);
        let mut delay = ctx.device.TIM14.delay(&mut rcc);
        rtt_init_print!();
        let mut adc = ctx.device.ADC.constrain(&mut rcc);

        adc.set_sample_time(SampleTime::T_80);
        adc.set_precision(Precision::B_12);
        adc.set_oversampling_ratio(OversamplingRatio::X_16);
        adc.set_oversampling_shift(16);
        adc.oversampling_enable(true);
        delay.delay(20.millis());
        adc.calibrate();


        tim_ower.listen();
        rprintln!("start");
        let mut exti = ctx.device.EXTI;
        pwm_ch2.enable();
        pwm_ch2.set_duty(0);
        pir.listen(SignalEdge::All, &mut exti);
        let pwm_max = pwm_ch2.get_max_duty();
        let opto_pin = gpiob.pb7.into_analog();
        //let handler = fade_out::spawn_after(Duration::<u64, 1, 1000>::from_ticks(10)).unwrap();

        (
            Shared {
                pir_status: false,
                //trigger,
                led: pwm_ch2,
                pwm_max,
                delay,
                led_status: false,
                fade_out_handle: None,
                fade_order: false,
                adc,
                opto_pin,
            },
            Local {
                exti,
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = EXTI4_15, priority = 2, local = [exti], shared=[pir_status, fade_out_handle, fade_order, adc, opto_pin, led_status])]
    fn pir_signal(ctx: pir_signal::Context) {
        //let mut trigger = ctx.shared.trigger;
        let mut pir_status = ctx.shared.pir_status;
        let mut led_status = ctx.shared.led_status;
        let mut fade_out_handle = ctx.shared.fade_out_handle;
        let mut fade_order = ctx.shared.fade_order;
        let opto_pin = ctx.shared.opto_pin;
        let adc = ctx.shared.adc;
        let fade_order_status = fade_order.lock(|fade_order| *fade_order);
        let status_led  = led_status.lock(|led_status| *led_status);
        let exti = ctx.local.exti;
        use SignalEdge::*;

        let status = exti.is_pending(exti::Event::GPIO12, Rising);
        let status_fall = exti.is_pending(exti::Event::GPIO12, Falling);

        pir_status.lock(|pir_status|{
            *pir_status = status;
        });

        let opto_data = (adc, opto_pin).lock(|adc, opto_pin| {
            let data = adc.read_voltage(opto_pin).unwrap();
            return data
        });

        rprintln!("status led-{} opto:{} stat up-{} down-{}", status_led, opto_data, status, status_fall);
        if status && opto_data<100 && status_led == false {
            fade_in::spawn().unwrap();
            if fade_order_status {
                fade_out_handle.lock(|fade_out_h| {
                    rprintln!("fade_out_handle ");
                    //fade_out_h.take();
                    let taked = fade_out_h.take();
                    rprintln!("tacked {:?}", taked);
                    if let Some(handler_task) = taked{
                        //handler_task.cancel();
                        let resp = handler_task.cancel();
                        match resp {
                            Ok(respon) => rprintln!("handler result {:?}", respon),
                            Err(_) => rprintln!("cancel false"),
                        };
                    }
                    // match taked {
                    //     Some(handler_task) => {
                    //         // if let Ok(resp_on) = handler.cancel() {
                    //         //     rprintln!("handler result {:?}", resp_on);
                    //         // }
                    //
                    //
                    //         let resp = handler_task.cancel();
                    //         match resp {
                    //             Ok(respon) => rprintln!("handler result {:?}", respon),
                    //             Err(_) => rprintln!("cancel false"),
                    //         };
                    //     },
                    //     None => rprintln!("empty handler"),
                    // }
                });
            }
            fade_order.lock(|fade_order| { *fade_order = false; } );

        } else if status_fall && status_led {
            rprintln!("fail");
            let handler_new;

            if let Ok(handler) = fade_out::spawn_after(Duration::<u64, 1, 1000>::from_ticks(5000)) {
                handler_new = Some(handler);
            } else {
                handler_new = None;
            }


            (fade_out_handle, fade_order).lock(|fade_out_h, order| {
                *fade_out_h = handler_new;
                *order = true;
            });
        }

        exti.unpend(exti::Event::GPIO12);
    }

    #[task(priority = 3, shared=[led, pwm_max, delay, led_status])]
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
                delay.lock(|delay| delay.delay(10.millis()));
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

    #[task(priority = 3,  shared=[led, pwm_max, delay, led_status,fade_out_handle])]
    fn fade_out(ctx: fade_out::Context) {
        let mut led = ctx.shared.led;
        let mut led_status = ctx.shared.led_status;
        let mut delay = ctx.shared.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);
        let status  = led_status.lock(|led_status| *led_status);
        let mut fade_out_handle = ctx.shared.fade_out_handle;
        rprintln!("led off");
        if status {
            for val in 0..100 {
                delay.lock(|delay| delay.delay(10.millis()));
                let re_pr = 100 - val;
                (led).lock(|led| {
                    led.set_duty(max * re_pr / 100);
                });
            }
            fade_out_handle.lock(|handler|{
                *handler = None;
            });
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