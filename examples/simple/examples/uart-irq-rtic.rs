//! More complex UART application
//!
//! Uses the IRQ capabilities of the VA10820 peripheral and the RTIC framework to poll the UART in
//! a non-blocking way. You can send variably sized strings to the VA10820 which will be echoed
//! back to the sender.
//!
//! This script was tested with an Arduino Due. You can find the test script in the
//! [`/test/DueSerialTest`](https://egit.irs.uni-stuttgart.de/rust/va108xx-hal/src/branch/main/test/DueSerialTest)
//! folder.
#![no_main]
#![no_std]

#[rtic::app(device = pac, dispatchers = [OC4])]
mod app {
    use embedded_io::Write;
    use rtic_monotonics::systick::Systick;
    use rtic_sync::make_channel;
    use panic_rtt_target as _;
    use rtt_target::{rprintln, rtt_init_print};
    use va108xx_hal::{
        time::Hertz,
        gpio::PinsB,
        pac,
        prelude::*,
        uart::{self, IrqCfg, IrqResult, UartWithIrqBase},
    };

    #[local]
    struct Local {
        rx_info_tx: rtic_sync::channel::Sender<'static, RxInfo, 3>,
        rx_info_rx: rtic_sync::channel::Receiver<'static, RxInfo, 3>,
    }

    #[shared]
    struct Shared {
        irq_uart: UartWithIrqBase<pac::Uartb>,
        rx_buf: [u8; 64],
    }

    #[derive(Debug, Copy, Clone)]
    struct RxInfo {
        pub bytes_read: usize,
        pub end_idx: usize,
        pub timeout: bool,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        //set_print_channel(channels.up.0);
        rprintln!("-- VA108xx UART IRQ example application--");

        // Initialize the systick interrupt & obtain the token to prove that we did
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, Hertz::from(50.MHz()).raw(), systick_mono_token);

        let mut dp = cx.device;
        let gpiob = PinsB::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.portb);
        let tx = gpiob.pb21.into_funsel_1();
        let rx = gpiob.pb20.into_funsel_1();

        let irq_cfg = IrqCfg::new(pac::interrupt::OC3, true, true);
        let (mut irq_uart, _) =
            uart::Uart::uartb(dp.uartb, (tx, rx), 115200.Hz(), &mut dp.sysconfig, 50.MHz())
                .into_uart_with_irq(irq_cfg, Some(&mut dp.sysconfig), Some(&mut dp.irqsel))
                .downgrade();
        irq_uart
            .read_fixed_len_using_irq(64, true)
            .expect("Read initialization failed");

        let (rx_info_tx, rx_info_rx) = make_channel!(RxInfo, 3);
        let rx_buf: [u8; 64] = [0; 64];
        //reply_handler::spawn().expect("spawning reply handler failed");
        (
            Shared { irq_uart, rx_buf },
            Local {
                rx_info_tx,
                rx_info_rx,
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
        shared = [irq_uart, rx_buf],
        local = [cnt: u32 = 0, result: IrqResult = IrqResult::new(), rx_info_tx],
    )]
    fn reception_task(cx: reception_task::Context) {
        let result = cx.local.result;
        let cnt: &mut u32 = cx.local.cnt;
        let irq_uart = cx.shared.irq_uart;
        let rx_buf = cx.shared.rx_buf;
        let (completed, end_idx) = (irq_uart, rx_buf).lock(|irq_uart, rx_buf| {
            match irq_uart.irq_handler(result, rx_buf) {
                Ok(_) => {
                    if result.complete() {
                        // Initiate next transfer immediately
                        irq_uart
                            .read_fixed_len_using_irq(64, true)
                            .expect("Read operation init failed");

                        let mut end_idx = 0;
                        for idx in 0..rx_buf.len() {
                            if (rx_buf[idx] as char) == '\n' {
                                end_idx = idx;
                                break;
                            }
                        }
                        (true, end_idx)
                    } else {
                        (false, 0)
                    }
                }
                Err(e) => {
                    rprintln!("reception error {:?}", e);
                    (false, 0)
                }
            }
        });
        if completed {
            rprintln!("counter: {}", cnt);
            cx.local
                .rx_info_tx
                .try_send(RxInfo {
                    bytes_read: result.bytes_read,
                    end_idx,
                    timeout: result.timeout(),
                })
                .expect("RX queue full");
        }
        *cnt += 1;
    }

    #[task(shared = [irq_uart, rx_buf], local = [rx_info_rx], priority=1)]
    async fn reply_handler(cx: reply_handler::Context) {
        let mut irq_uart = cx.shared.irq_uart;
        let mut rx_buf = cx.shared.rx_buf;
        loop {
            match cx.local.rx_info_rx.recv().await {
                Ok(rx_info) => {
                    rprintln!("reception success, {} bytes read", rx_info.bytes_read);
                    if rx_info.timeout {
                        rprintln!("timeout occurred");
                    }
                    rx_buf.lock(|rx_buf| {
                        let string = core::str::from_utf8(&rx_buf[0..rx_info.end_idx])
                            .expect("Invalid string format");
                        rprintln!("read string: {}", string);
                        irq_uart.lock(|uart| {
                            writeln!(uart.uart, "{}", string).expect("Sending reply failed");
                        });
                    });
                }
                Err(e) => {
                    rprintln!("error receiving RX info: {:?}", e);
                }
            }
        }
    }
}
