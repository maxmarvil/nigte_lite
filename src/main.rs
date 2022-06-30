#![no_std]
#![no_main]
#![deny(warnings)]

//use cortex_m_semihosting::hprintln;

extern crate panic_semihosting;

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [CEC])]
mod app {
    use stm32g0xx_hal as hal;
    //use cortex_m::asm;
    //use hal::hal::adc::Channel;
    use hal::{prelude::*, stm32, stm32::TIM14, exti,
              timer::{pwm, Channel3, delay::Delay},
              gpio::{ SignalEdge,gpiob::PB7},//,

              analog::adc::{ OversamplingRatio, Precision, SampleTime, Adc, }};
    //use rtt_target::{rprintln, rtt_init_print};
    use stm32g0xx_hal::gpio::Analog;
    use systick_monotonic::{ Systick};//fugit::Duration,fugit::Hertz,
    use hal::power::{LowPowerMode, PowerMode};
    use cortex_m::peripheral::{SCB};
    use hal::timer::Timer;


    #[shared]
    struct Shared {
        pir_status: bool,
        led: pwm::PwmPin<stm32::TIM3, Channel3>,//PA12<Output<PushPull>>,
        pwm_max: u32,
        delay: Delay<TIM14>,
        led_status: bool,
        adc: Adc,
        opto_pin: PB7<Analog>,//Channel<Adc>,
        timer: Timer<stm32::TIM17>,
    }

    #[local]
    struct Local {
        exti: stm32::EXTI,
        scb: SCB
    }

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<100>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.constrain();
        let systick = ctx.core.SYST;
        let mono = Systick::new(systick, 16_000_000);

        let gpioa = ctx.device.GPIOA.split(&mut rcc);
        let gpiob = ctx.device.GPIOB.split(&mut rcc);
        let pir = gpioa.pa12.into_pull_down_input();
        let pwm = ctx.device.TIM3.pwm(1.kHz(), &mut rcc);
        //let trigger = None;
        let mut pwm_ch2 = pwm.bind_pin(gpiob.pb0);
        let mut delay = ctx.device.TIM14.delay(&mut rcc);
        //rtt_init_print!();
        let mut adc = ctx.device.ADC.constrain(&mut rcc);
        let timer = ctx.device.TIM17.timer(&mut rcc);
        //timer.start(1.kHz<1,100>().into_duration());
        //timer.listen();

        let mut power = ctx.device.PWR.constrain(&mut rcc);
        power.set_mode(PowerMode::UltraLowPower(LowPowerMode::StopMode2));
        let  scb = ctx.core.SCB;

        adc.set_sample_time(SampleTime::T_80);
        adc.set_precision(Precision::B_12);
        adc.set_oversampling_ratio(OversamplingRatio::X_16);
        adc.set_oversampling_shift(16);
        adc.oversampling_enable(true);
        delay.delay(20.millis());
        adc.calibrate();


        //rprintln!("start");
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
                adc,
                opto_pin,
                timer
            },
            Local {
                exti,
                scb
            },
            init::Monotonics(mono),
        )
    }

    #[task(binds = TIM17, shared = [ ])]
    fn timer_tick(_ctx: timer_tick::Context) {
        fade_out::spawn().unwrap();
    }

    #[task(binds = EXTI4_15, priority = 3, local = [exti], shared=[pir_status, adc, opto_pin, led_status,timer])]
    fn pir_signal(ctx: pir_signal::Context) {
        let mut pir_status = ctx.shared.pir_status;
        let opto_pin = ctx.shared.opto_pin;
        let adc = ctx.shared.adc;
        let exti = ctx.local.exti;
        use SignalEdge::*;
        let mut timer = ctx.shared.timer;

        timer.lock(|timer| {timer.listen();});

        let status_up = exti.is_pending(exti::Event::GPIO12, Rising);

        pir_status.lock(|pir_status|{
            *pir_status = status_up;
        });
        // if status {
        //     rprintln!("up");
        // }
        // if status_fall {
        //     rprintln!("fail");
        // }

        let opto_data = (adc, opto_pin).lock(|adc, opto_pin| adc.read_voltage(opto_pin).unwrap_or(0));

        //rprintln!("status led-{} opto:{}", status_led, opto_data);
        if status_up  {
            if opto_data < 200 {
                fade_in::spawn().unwrap();
                (timer).lock(|timer| {
                    timer.reset();
                });
            }

        } else  {
            (timer).lock(|timer| {
                timer.start(6000.millis());
            });
        }

        exti.unpend(exti::Event::GPIO12);
    }

    #[task(priority = 2, shared=[led, pwm_max, delay])]
    fn fade_in(ctx: fade_in::Context) {
        let mut led = ctx.shared.led;
        let mut delay = ctx.shared.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);

        for val in 0 .. 100 {
            delay.lock(|delay| delay.delay(10.millis()));
            ( led).lock(|led| {
                led.set_duty(max*val/100);
            });
        }
        ( led).lock(|led| {
            led.set_duty(max);
        });
    }

    #[task(priority = 2, local=[scb], shared=[led, pwm_max, delay, led_status, timer])]
    fn fade_out(ctx: fade_out::Context) {
        let mut led = ctx.shared.led;
        let mut delay = ctx.shared.delay;
        let mut pwm_max = ctx.shared.pwm_max;
        let mut timer = ctx.shared.timer;
        let scb = ctx.local.scb;
        let max  = pwm_max.lock(|pwm_max| *pwm_max);

        for val in 0..100 {
            delay.lock(|delay| delay.delay(5.millis()));
            let re_pr = 100 - val;
            (led).lock(|led| {
                led.set_duty(max * re_pr / 100);
            });
        }
        (timer).lock(|timer| {
            timer.clear_irq();
        });
        (led).lock(|led| {
            led.set_duty(0);
        });
        delay.lock(|delay| delay.delay(50.millis()));
        scb.set_sleepdeep();
        scb.set_sleeponexit();

    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}