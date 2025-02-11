//! This example demonstrates the usage of async GPIO operations on VA108xx.
//!
//! You need to tie the PA0 to the PA1 pin for this example to work. You can optionally tie the PB22 to PB23 pins well
//! and then set the `CHECK_PB22_TO_PB23` to true to also test async operations on Port B.
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::channel::{Receiver, Sender};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal_async::digital::Wait;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
use va108xx_embassy::embassy;
use va108xx_hal::gpio::{handle_interrupt_for_async_gpio, InputDynPinAsync, InputPinAsync, PinsB};
use va108xx_hal::{
    gpio::{DynPin, PinsA},
    pac::{self, interrupt},
    prelude::*,
};

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

const CHECK_PA0_TO_PA1: bool = true;
const CHECK_PB22_TO_PB23: bool = false;

// Can also be set to OC10 and works as well.
const PB22_TO_PB23_IRQ: pac::Interrupt = pac::Interrupt::OC11;

#[derive(Clone, Copy)]
pub struct GpioCmd {
    cmd_type: GpioCmdType,
    after_delay: u32,
}

impl GpioCmd {
    pub fn new(cmd_type: GpioCmdType, after_delay: u32) -> Self {
        Self {
            cmd_type,
            after_delay,
        }
    }
}

#[derive(Clone, Copy)]
pub enum GpioCmdType {
    SetHigh,
    SetLow,
    RisingEdge,
    FallingEdge,
}

// Declare a bounded channel of 3 u32s.
static CHANNEL_PA0_PA1: Channel<ThreadModeRawMutex, GpioCmd, 3> = Channel::new();
static CHANNEL_PB22_TO_PB23: Channel<ThreadModeRawMutex, GpioCmd, 3> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_init_print!();
    rprintln!("-- VA108xx Async GPIO Demo --");

    let mut dp = pac::Peripherals::take().unwrap();

    // Safety: Only called once here.
    unsafe {
        embassy::init(
            &mut dp.sysconfig,
            &dp.irqsel,
            SYSCLK_FREQ,
            dp.tim23,
            dp.tim22,
        )
    };

    let porta = PinsA::new(&mut dp.sysconfig, dp.porta);
    let portb = PinsB::new(&mut dp.sysconfig, dp.portb);
    let mut led0 = porta.pa10.into_readable_push_pull_output();
    let out_pa0 = porta.pa0.into_readable_push_pull_output();
    let in_pa1 = porta.pa1.into_floating_input();
    let out_pb22 = portb.pb22.into_readable_push_pull_output();
    let in_pb23 = portb.pb23.into_floating_input();

    let in_pa1_async = InputPinAsync::new(in_pa1, pac::Interrupt::OC10);
    let out_pa0_dyn = out_pa0.downgrade();
    let in_pb23_async = InputDynPinAsync::new(in_pb23.downgrade(), PB22_TO_PB23_IRQ).unwrap();
    let out_pb22_dyn = out_pb22.downgrade();

    spawner
        .spawn(output_task(
            "PA0 to PA1",
            out_pa0_dyn,
            CHANNEL_PA0_PA1.receiver(),
        ))
        .unwrap();
    spawner
        .spawn(output_task(
            "PB22 to PB23",
            out_pb22_dyn,
            CHANNEL_PB22_TO_PB23.receiver(),
        ))
        .unwrap();

    if CHECK_PA0_TO_PA1 {
        check_pin_to_pin_async_ops("PA0 to PA1", CHANNEL_PA0_PA1.sender(), in_pa1_async).await;
        rprintln!("Example PA0 to PA1 done");
    }
    if CHECK_PB22_TO_PB23 {
        check_pin_to_pin_async_ops("PB22 to PB23", CHANNEL_PB22_TO_PB23.sender(), in_pb23_async)
            .await;
        rprintln!("Example PB22 to PB23 done");
    }

    rprintln!("Example done, toggling LED0");
    loop {
        led0.toggle().unwrap();
        Timer::after(Duration::from_millis(500)).await;
    }
}

async fn check_pin_to_pin_async_ops(
    ctx: &'static str,
    sender: Sender<'static, ThreadModeRawMutex, GpioCmd, 3>,
    mut async_input: impl Wait,
) {
    rprintln!(
        "{}: sending SetHigh command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender.send(GpioCmd::new(GpioCmdType::SetHigh, 20)).await;
    async_input.wait_for_high().await.unwrap();
    rprintln!(
        "{}: Input pin is high now ({} ms)",
        ctx,
        Instant::now().as_millis()
    );

    rprintln!(
        "{}: sending SetLow command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender.send(GpioCmd::new(GpioCmdType::SetLow, 20)).await;
    async_input.wait_for_low().await.unwrap();
    rprintln!(
        "{}: Input pin is low now ({} ms)",
        ctx,
        Instant::now().as_millis()
    );

    rprintln!(
        "{}: sending RisingEdge command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender.send(GpioCmd::new(GpioCmdType::RisingEdge, 20)).await;
    async_input.wait_for_rising_edge().await.unwrap();
    rprintln!(
        "{}: input pin had rising edge ({} ms)",
        ctx,
        Instant::now().as_millis()
    );

    rprintln!(
        "{}: sending Falling command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender
        .send(GpioCmd::new(GpioCmdType::FallingEdge, 20))
        .await;
    async_input.wait_for_falling_edge().await.unwrap();
    rprintln!(
        "{}: input pin had a falling edge ({} ms)",
        ctx,
        Instant::now().as_millis()
    );

    rprintln!(
        "{}: sending Falling command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender
        .send(GpioCmd::new(GpioCmdType::FallingEdge, 20))
        .await;
    async_input.wait_for_any_edge().await.unwrap();
    rprintln!(
        "{}: input pin had a falling (any) edge ({} ms)",
        ctx,
        Instant::now().as_millis()
    );

    rprintln!(
        "{}: sending Falling command ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
    sender.send(GpioCmd::new(GpioCmdType::RisingEdge, 20)).await;
    async_input.wait_for_any_edge().await.unwrap();
    rprintln!(
        "{}: input pin had a rising (any) edge ({} ms)",
        ctx,
        Instant::now().as_millis()
    );
}

#[embassy_executor::task(pool_size = 2)]
async fn output_task(
    ctx: &'static str,
    mut out: DynPin,
    receiver: Receiver<'static, ThreadModeRawMutex, GpioCmd, 3>,
) {
    loop {
        let next_cmd = receiver.receive().await;
        Timer::after(Duration::from_millis(next_cmd.after_delay.into())).await;
        match next_cmd.cmd_type {
            GpioCmdType::SetHigh => {
                rprintln!("{}: Set output high", ctx);
                out.set_high().unwrap();
            }
            GpioCmdType::SetLow => {
                rprintln!("{}: Set output low", ctx);
                out.set_low().unwrap();
            }
            GpioCmdType::RisingEdge => {
                rprintln!("{}: Rising edge", ctx);
                if !out.is_low().unwrap() {
                    out.set_low().unwrap();
                }
                out.set_high().unwrap();
            }
            GpioCmdType::FallingEdge => {
                rprintln!("{}: Falling edge", ctx);
                if !out.is_high().unwrap() {
                    out.set_high().unwrap();
                }
                out.set_low().unwrap();
            }
        }
    }
}

// PB22 to PB23 can be handled by both OC10 and OC11 depending on configuration.

#[interrupt]
#[allow(non_snake_case)]
fn OC10() {
    handle_interrupt_for_async_gpio();
}

#[interrupt]
#[allow(non_snake_case)]
fn OC11() {
    handle_interrupt_for_async_gpio();
}
