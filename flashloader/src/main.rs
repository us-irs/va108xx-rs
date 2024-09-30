//! Vorago flashloader which can be used to flash image A and image B via a simple
//! low-level CCSDS memory interface via a UART interface.
#![no_main]
#![no_std]

use once_cell::sync::Lazy;
use panic_rtt_target as _;
use ringbuf::{
    traits::{Consumer, Observer, Producer, SplitRef},
    CachingCons, StaticProd, StaticRb,
};
use va108xx_hal::prelude::*;

const SYSCLK_FREQ: Hertz = Hertz::from_raw(50_000_000);

const MAX_TC_SIZE: usize = 524;
const MAX_TC_FRAME_SIZE: usize = cobs::max_encoding_length(MAX_TC_SIZE);

const MAX_TM_SIZE: usize = 128;
const MAX_TM_FRAME_SIZE: usize = cobs::max_encoding_length(MAX_TM_SIZE);

const UART_BAUDRATE: u32 = 115200;
const BOOT_NVM_MEMORY_ID: u8 = 1;
const RX_DEBUGGING: bool = false;

pub enum ActionId {
    CorruptImageA = 128,
    CorruptImageB = 129,
}

// Larger buffer for TC to be able to hold the possibly large memory write packets.
const BUF_RB_SIZE_TC: usize = 1024;
const SIZES_RB_SIZE_TC: usize = 16;

const BUF_RB_SIZE_TM: usize = 256;
const SIZES_RB_SIZE_TM: usize = 16;

// Ring buffers to handling variable sized telemetry
static mut BUF_RB_TM: Lazy<StaticRb<u8, BUF_RB_SIZE_TM>> =
    Lazy::new(StaticRb::<u8, BUF_RB_SIZE_TM>::default);
static mut SIZES_RB_TM: Lazy<StaticRb<usize, SIZES_RB_SIZE_TM>> =
    Lazy::new(StaticRb::<usize, SIZES_RB_SIZE_TM>::default);

// Ring buffers to handling variable sized telecommands
static mut BUF_RB_TC: Lazy<StaticRb<u8, BUF_RB_SIZE_TC>> =
    Lazy::new(StaticRb::<u8, BUF_RB_SIZE_TC>::default);
static mut SIZES_RB_TC: Lazy<StaticRb<usize, SIZES_RB_SIZE_TC>> =
    Lazy::new(StaticRb::<usize, SIZES_RB_SIZE_TC>::default);

pub struct DataProducer<const BUF_SIZE: usize, const SIZES_LEN: usize> {
    pub buf_prod: StaticProd<'static, u8, BUF_SIZE>,
    pub sizes_prod: StaticProd<'static, usize, SIZES_LEN>,
}

pub struct DataConsumer<const BUF_SIZE: usize, const SIZES_LEN: usize> {
    pub buf_cons: CachingCons<&'static StaticRb<u8, BUF_SIZE>>,
    pub sizes_cons: CachingCons<&'static StaticRb<usize, SIZES_LEN>>,
}

pub const APP_A_START_ADDR: u32 = 0x3000;
pub const APP_A_END_ADDR: u32 = 0x11800;
pub const APP_B_START_ADDR: u32 = APP_A_END_ADDR;
pub const APP_B_END_ADDR: u32 = 0x20000;

#[rtic::app(device = pac, dispatchers = [OC20, OC21, OC22])]
mod app {
    use super::*;
    use cortex_m::asm;
    use embedded_io::Write;
    use panic_rtt_target as _;
    use rtic::Mutex;
    use rtic_monotonics::systick::prelude::*;
    use rtt_target::rprintln;
    use satrs::pus::verification::{FailParams, VerificationReportCreator};
    use spacepackets::ecss::PusServiceId;
    use spacepackets::ecss::{
        tc::PusTcReader, tm::PusTmCreator, EcssEnumU8, PusPacket, WritablePusPacket,
    };
    use va108xx_hal::gpio::PinsA;
    use va108xx_hal::uart::IrqContextTimeoutOrMaxSize;
    use va108xx_hal::{pac, uart};
    use vorago_reb1::m95m01::M95M01;

    #[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
    pub enum CobsReaderStates {
        #[default]
        WaitingForStart,
        WatingForEnd,
        FrameOverflow,
    }

    #[local]
    struct Local {
        uart_rx: uart::RxWithIrq<pac::Uarta>,
        uart_tx: uart::Tx<pac::Uarta>,
        rx_context: IrqContextTimeoutOrMaxSize,
        // We handle all TM in one task.
        tm_cons: DataConsumer<BUF_RB_SIZE_TM, SIZES_RB_SIZE_TM>,
        // We consume all TC in one task.
        tc_cons: DataConsumer<BUF_RB_SIZE_TC, SIZES_RB_SIZE_TC>,
        // We produce all TC in one task.
        tc_prod: DataProducer<BUF_RB_SIZE_TC, SIZES_RB_SIZE_TC>,
        verif_reporter: VerificationReportCreator,
        nvm: M95M01,
    }

    #[shared]
    struct Shared {
        // Having this shared allows multiple tasks to generate telemetry.
        tm_prod: DataProducer<BUF_RB_SIZE_TM, SIZES_RB_SIZE_TM>,
    }

    rtic_monotonics::systick_monotonic!(Mono, 1000);

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_log::init();
        rprintln!("-- Vorago flashloader --");

        Mono::start(cx.core.SYST, SYSCLK_FREQ.raw());

        let mut dp = cx.device;
        let nvm = M95M01::new(&mut dp.sysconfig, SYSCLK_FREQ, dp.spic);

        let gpioa = PinsA::new(&mut dp.sysconfig, Some(dp.ioconfig), dp.porta);
        let tx = gpioa.pa9.into_funsel_2();
        let rx = gpioa.pa8.into_funsel_2();

        let irq_uart = uart::Uart::new(
            &mut dp.sysconfig,
            SYSCLK_FREQ,
            dp.uarta,
            (tx, rx),
            UART_BAUDRATE.Hz(),
        );
        let (tx, rx) = irq_uart.split();
        let mut rx = rx.into_rx_with_irq(&mut dp.sysconfig, &mut dp.irqsel, pac::interrupt::OC0);

        let verif_reporter = VerificationReportCreator::new(0).unwrap();

        let (buf_prod_tm, buf_cons_tm) = unsafe { BUF_RB_TM.split_ref() };
        let (sizes_prod_tm, sizes_cons_tm) = unsafe { SIZES_RB_TM.split_ref() };

        let (buf_prod_tc, buf_cons_tc) = unsafe { BUF_RB_TC.split_ref() };
        let (sizes_prod_tc, sizes_cons_tc) = unsafe { SIZES_RB_TC.split_ref() };

        let mut rx_context = IrqContextTimeoutOrMaxSize::new(MAX_TC_FRAME_SIZE);
        rx.read_fixed_len_or_timeout_based_using_irq(&mut rx_context)
            .expect("initiating UART RX failed");
        pus_tc_handler::spawn().unwrap();
        pus_tm_tx_handler::spawn().unwrap();
        (
            Shared {
                tm_prod: DataProducer {
                    buf_prod: buf_prod_tm,
                    sizes_prod: sizes_prod_tm,
                },
            },
            Local {
                uart_rx: rx,
                uart_tx: tx,
                rx_context,
                tm_cons: DataConsumer {
                    buf_cons: buf_cons_tm,
                    sizes_cons: sizes_cons_tm,
                },
                tc_cons: DataConsumer {
                    buf_cons: buf_cons_tc,
                    sizes_cons: sizes_cons_tc,
                },
                tc_prod: DataProducer {
                    buf_prod: buf_prod_tc,
                    sizes_prod: sizes_prod_tc,
                },
                verif_reporter,
                nvm,
            },
        )
    }

    // `shared` cannot be accessed from this context
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    // This is the interrupt handler to read all bytes received on the UART0.
    #[task(
        binds = OC0,
        local = [
            cnt: u32 = 0,
            rx_buf: [u8; MAX_TC_FRAME_SIZE] = [0; MAX_TC_FRAME_SIZE],
            rx_context,
            uart_rx,
            tc_prod
        ],
    )]
    fn uart_rx_irq(cx: uart_rx_irq::Context) {
        match cx
            .local
            .uart_rx
            .irq_handler_max_size_or_timeout_based(cx.local.rx_context, cx.local.rx_buf)
        {
            Ok(result) => {
                if RX_DEBUGGING {
                    log::debug!("RX Info: {:?}", cx.local.rx_context);
                    log::debug!("RX Result: {:?}", result);
                }
                if result.complete() {
                    // Check frame validity (must have COBS format) and decode the frame.
                    // Currently, we expect a full frame or a frame received through a timeout
                    // to be one COBS frame. We could parse for multiple COBS packets in one
                    // frame, but the additional complexity is not necessary here..
                    if cx.local.rx_buf[0] == 0 && cx.local.rx_buf[result.bytes_read - 1] == 0 {
                        let decoded_size =
                            cobs::decode_in_place(&mut cx.local.rx_buf[1..result.bytes_read]);
                        if decoded_size.is_err() {
                            log::warn!("COBS decoding failed");
                        } else {
                            let decoded_size = decoded_size.unwrap();
                            if cx.local.tc_prod.sizes_prod.vacant_len() >= 1
                                && cx.local.tc_prod.buf_prod.vacant_len() >= decoded_size
                            {
                                // Should never fail, we checked there is enough space.
                                cx.local.tc_prod.sizes_prod.try_push(decoded_size).unwrap();
                                cx.local
                                    .tc_prod
                                    .buf_prod
                                    .push_slice(&cx.local.rx_buf[1..1 + decoded_size]);
                            } else {
                                log::warn!("COBS TC queue full");
                            }
                        }
                    } else {
                        log::warn!("COBS frame with invalid format, start and end bytes are not 0");
                    }

                    // Initiate next transfer.
                    cx.local
                        .uart_rx
                        .read_fixed_len_or_timeout_based_using_irq(cx.local.rx_context)
                        .expect("read operation failed");
                }
                if result.has_errors() {
                    log::warn!("UART error: {:?}", result.errors.unwrap());
                }
            }
            Err(e) => {
                log::warn!("UART error: {:?}", e);
            }
        }
    }

    #[task(
        priority = 2,
        local=[
            tc_buf: [u8; MAX_TC_SIZE] = [0; MAX_TC_SIZE],
            readback_buf: [u8; MAX_TC_SIZE] = [0; MAX_TC_SIZE],
            src_data_buf: [u8; 16] = [0; 16],
            verif_buf: [u8; 32] = [0; 32],
            tc_cons,
            nvm,
            verif_reporter
        ],
        shared=[tm_prod]
    )]
    async fn pus_tc_handler(mut cx: pus_tc_handler::Context) {
        loop {
            // Try to read a TC from the ring buffer.
            let packet_len = cx.local.tc_cons.sizes_cons.try_pop();
            if packet_len.is_none() {
                // Small delay, TCs might arrive very quickly.
                Mono::delay(20.millis()).await;
                continue;
            }
            let packet_len = packet_len.unwrap();
            log::info!(target: "TC Handler", "received packet with length {}", packet_len);
            assert_eq!(
                cx.local
                    .tc_cons
                    .buf_cons
                    .pop_slice(&mut cx.local.tc_buf[0..packet_len]),
                packet_len
            );
            // Read a telecommand, now handle it.
            handle_valid_pus_tc(&mut cx);
        }
    }

    fn handle_valid_pus_tc(cx: &mut pus_tc_handler::Context) {
        let pus_tc = PusTcReader::new(cx.local.tc_buf);
        if pus_tc.is_err() {
            log::warn!(target: "TC Handler", "PUS TC error: {}", pus_tc.unwrap_err());
            return;
        }
        let (pus_tc, _) = pus_tc.unwrap();
        let mut write_and_send = |tm: &PusTmCreator| {
            let written_size = tm.write_to_bytes(cx.local.verif_buf).unwrap();
            cx.shared.tm_prod.lock(|prod| {
                prod.sizes_prod.try_push(tm.len_written()).unwrap();
                prod.buf_prod
                    .push_slice(&cx.local.verif_buf[0..written_size]);
            });
        };
        let token = cx.local.verif_reporter.add_tc(&pus_tc);
        let (tm, accepted_token) = cx
            .local
            .verif_reporter
            .acceptance_success(cx.local.src_data_buf, token, 0, 0, &[])
            .expect("acceptance success failed");
        write_and_send(&tm);

        let (tm, started_token) = cx
            .local
            .verif_reporter
            .start_success(cx.local.src_data_buf, accepted_token, 0, 0, &[])
            .expect("acceptance success failed");
        write_and_send(&tm);

        if pus_tc.service() == PusServiceId::Action as u8 {
            let mut corrupt_image = |base_addr: u32| {
                let mut buf = [0u8; 4];
                cx.local
                    .nvm
                    .read(base_addr as usize + 32, &mut buf)
                    .expect("reading from NVM failed");
                buf[0] += 1;
                cx.local
                    .nvm
                    .write(base_addr as usize + 32, &buf)
                    .expect("writing to NVM failed");
                let tm = cx
                    .local
                    .verif_reporter
                    .completion_success(cx.local.src_data_buf, started_token, 0, 0, &[])
                    .expect("completion success failed");
                write_and_send(&tm);
            };
            if pus_tc.subservice() == ActionId::CorruptImageA as u8 {
                rprintln!("corrupting App Image A");
                corrupt_image(APP_A_START_ADDR);
            }
            if pus_tc.subservice() == ActionId::CorruptImageB as u8 {
                rprintln!("corrupting App Image B");
                corrupt_image(APP_B_START_ADDR);
            }
        }
        if pus_tc.service() == PusServiceId::Test as u8 && pus_tc.subservice() == 1 {
            log::info!(target: "TC Handler", "received ping TC");
            let tm = cx
                .local
                .verif_reporter
                .completion_success(cx.local.src_data_buf, started_token, 0, 0, &[])
                .expect("completion success failed");
            write_and_send(&tm);
        } else if pus_tc.service() == PusServiceId::MemoryManagement as u8 {
            let tm = cx
                .local
                .verif_reporter
                .step_success(
                    cx.local.src_data_buf,
                    &started_token,
                    0,
                    0,
                    &[],
                    EcssEnumU8::new(0),
                )
                .expect("step success failed");
            write_and_send(&tm);
            // Raw memory write TC
            if pus_tc.subservice() == 2 {
                let app_data = pus_tc.app_data();
                if app_data.len() < 10 {
                    log::warn!(
                        target: "TC Handler",
                        "app data for raw memory write is too short: {}",
                        app_data.len()
                    );
                }
                let memory_id = app_data[0];
                if memory_id != BOOT_NVM_MEMORY_ID {
                    log::warn!(target: "TC Handler", "memory ID {} not supported", memory_id);
                    // TODO: Error reporting
                    return;
                }
                let offset = u32::from_be_bytes(app_data[2..6].try_into().unwrap());
                let data_len = u32::from_be_bytes(app_data[6..10].try_into().unwrap());
                if 10 + data_len as usize > app_data.len() {
                    log::warn!(
                        target: "TC Handler",
                        "invalid data length {} for raw mem write detected",
                        data_len
                    );
                    // TODO: Error reporting
                    return;
                }
                let data = &app_data[10..10 + data_len as usize];
                log::info!(
                    target: "TC Handler",
                    "writing {} bytes at offset {} to NVM",
                    data_len,
                    offset
                );
                cx.local
                    .nvm
                    .write(offset as usize, data)
                    .expect("writing to NVM failed");
                let tm = if !cx
                    .local
                    .nvm
                    .verify(offset as usize, data)
                    .expect("NVM verification failed")
                {
                    log::warn!("verification of data written to NVM failed");
                    cx.local
                        .verif_reporter
                        .completion_failure(
                            cx.local.src_data_buf,
                            started_token,
                            0,
                            0,
                            FailParams::new(&[], &EcssEnumU8::new(0), &[]),
                        )
                        .expect("completion success failed")
                } else {
                    cx.local
                        .verif_reporter
                        .completion_success(cx.local.src_data_buf, started_token, 0, 0, &[])
                        .expect("completion success failed")
                };
                write_and_send(&tm);
                log::info!(
                    target: "TC Handler",
                    "NVM operation done");
            }
        }
    }

    #[task(
        priority = 1,
        local=[
            read_buf: [u8;MAX_TM_SIZE] = [0; MAX_TM_SIZE],
            encoded_buf: [u8;MAX_TM_FRAME_SIZE] = [0; MAX_TM_FRAME_SIZE],
            uart_tx,
            tm_cons
        ],
        shared=[]
    )]
    async fn pus_tm_tx_handler(cx: pus_tm_tx_handler::Context) {
        loop {
            while cx.local.tm_cons.sizes_cons.occupied_len() > 0 {
                let next_size = cx.local.tm_cons.sizes_cons.try_pop().unwrap();
                cx.local
                    .tm_cons
                    .buf_cons
                    .pop_slice(&mut cx.local.read_buf[0..next_size]);
                cx.local.encoded_buf[0] = 0;
                let send_size = cobs::encode(
                    &cx.local.read_buf[0..next_size],
                    &mut cx.local.encoded_buf[1..],
                );
                cx.local.encoded_buf[send_size + 1] = 0;
                cx.local
                    .uart_tx
                    .write(&cx.local.encoded_buf[0..send_size + 2])
                    .unwrap();
                Mono::delay(2.millis()).await;
            }
            Mono::delay(50.millis()).await;
        }
    }
}