//! More complex UART application
//!
//! Uses the IRQ capabilities of the VA10820 peripheral and the RTIC framework to poll the UART in
//! a non-blocking way. All received data will be sent back to the sender.
#![no_main]
#![no_std]

use once_cell::sync::Lazy;
use ringbuf::StaticRb;

// Larger buffer for TC to be able to hold the possibly large memory write packets.
const RX_RING_BUF_SIZE: usize = 1024;

// Ring buffers to handling variable sized telemetry
static mut RINGBUF: Lazy<StaticRb<u8, RX_RING_BUF_SIZE>> =
    Lazy::new(StaticRb::<u8, RX_RING_BUF_SIZE>::default);

#[rtic::app(device = pac, dispatchers = [OC4])]
mod app {
    use super::*;
    use embedded_io::Write;
    use panic_rtt_target as _;
    use ringbuf::{
        traits::{Consumer, Observer, Producer, SplitRef},
        CachingCons, StaticProd,
    };
    use rtic_example::SYSCLK_FREQ;
    use rtic_monotonics::Monotonic;
    use rtt_target::{rprintln, rtt_init_print};
    use va108xx_hal::{
        gpio::PinsA,
        pac,
        prelude::*,
        uart::{self, RxWithIrq, Tx},
    };

    #[local]
    struct Local {
        data_producer: StaticProd<'static, u8, RX_RING_BUF_SIZE>,
        data_consumer: CachingCons<&'static StaticRb<u8, RX_RING_BUF_SIZE>>,
        rx: RxWithIrq<pac::Uarta>,
        tx: Tx<pac::Uarta>,
    }

    #[shared]
    struct Shared {}

    rtic_monotonics::systick_monotonic!(Mono, 1_000);

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("-- VA108xx UART Echo with IRQ example application--");

        Mono::start(cx.core.SYST, SYSCLK_FREQ.raw());

        let mut dp = cx.device;
        let gpioa = PinsA::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.porta);
        let tx = gpioa.pa9.into_funsel_2();
        let rx = gpioa.pa8.into_funsel_2();

        let irq_uart = uart::Uart::new(
            &mut dp.sysconfig,
            SYSCLK_FREQ,
            dp.uarta,
            (tx, rx),
            115200.Hz(),
        );
        let (tx, rx) = irq_uart.split();
        let mut rx = rx.into_rx_with_irq(&mut dp.sysconfig, &mut dp.irqsel, pac::interrupt::OC3);

        rx.start();

        let (data_producer, data_consumer) = unsafe { RINGBUF.split_ref() };
        echo_handler::spawn().unwrap();
        (
            Shared {},
            Local {
                data_producer,
                data_consumer,
                rx,
                tx,
            },
        )
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(
        binds = OC3,
        shared = [],
        local = [
            rx,
            data_producer
        ],
    )]
    fn reception_task(cx: reception_task::Context) {
        let mut buf: [u8; 16] = [0; 16];
        let mut ringbuf_full = false;
        let result = cx.local.rx.irq_handler(&mut buf);
        if result.bytes_read > 0 && result.errors.is_none() {
            if cx.local.data_producer.vacant_len() < result.bytes_read {
                ringbuf_full = true;
            } else {
                cx.local
                    .data_producer
                    .push_slice(&buf[0..result.bytes_read]);
            }
        }
        if ringbuf_full {
            // Could also drop oldest data, but that would require the consumer to be shared.
            rprintln!("buffer full, data was dropped");
        }
    }

    #[task(shared = [], local = [
        buf: [u8; RX_RING_BUF_SIZE] = [0; RX_RING_BUF_SIZE],
        data_consumer,
        tx
    ], priority=1)]
    async fn echo_handler(cx: echo_handler::Context) {
        loop {
            let bytes_to_read = cx.local.data_consumer.occupied_len();
            if bytes_to_read > 0 {
                let actual_read_bytes = cx
                    .local
                    .data_consumer
                    .pop_slice(&mut cx.local.buf[0..bytes_to_read]);
                cx.local
                    .tx
                    .write_all(&cx.local.buf[0..actual_read_bytes])
                    .expect("Failed to write to TX");
            }
            Mono::delay(50.millis()).await;
        }
    }
}