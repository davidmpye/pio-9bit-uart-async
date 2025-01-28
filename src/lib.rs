//! Pio backed uart drivers
//!
//! Based on:  Embassy-RP Pio uart example (Apache licence)
//! Based on:  Raspberry Pi Foundation PIO uart examples
//! Modified to 9 bit by David Pye davidmpye@gmail.com
//! Licenced under GNU GPL V3.0 or later (at your discretion)
#![no_std]
use core::convert::Infallible;
use embedded_io_async::{ErrorType, Read, Write};
use fixed::traits::ToFixed;

use embassy_rp::bind_interrupts;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::Level;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio;
use embassy_rp::pio::{
    Common, Config, Direction as PioDirection, FifoJoin, Instance, LoadedProgram, Pio, PioPin,
    ShiftDirection, StateMachine,
};
use embassy_time::{Duration, WithTimeout};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

pub struct PioUart<'a, const SM: usize> {
    tx: PioUartTx<'a, PIO0, 0>,
    rx: PioUartRx<'a, PIO0, 1>,
}

impl<'a, const SM: usize> PioUart<'a, SM> {
    pub fn new<T:PioPin, U:PioPin>(
        tx_pin: T,
        rx_pin: U,
        pio: PIO0,
        first_byte_timeout: Duration,
        interbyte_timeout: Duration,
    ) -> Self {
        let pio::Pio {
            mut common,
            sm0,
            sm1,
            ..
        } = pio::Pio::new(pio, Irqs);

        let tx_program = PioUartTxProgram::new(&mut common);
        let tx = PioUartTx::new(9600, &mut common, sm0, tx_pin, &tx_program);
        let rx_program = PioUartRxProgram::new(&mut common);
        let rx = PioUartRx::new(
            9600,
            first_byte_timeout,
            interbyte_timeout,
            &mut common,
            sm1,
            rx_pin,
            &rx_program,
        );

        Self { tx, rx }
    }
}

impl<'a, const SM: usize> Read for PioUart<'a, SM> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        self.rx.read(buf).await
    }
}

impl<'a, const SM: usize> Write for PioUart<'a, SM> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        self.tx.write(buf).await
    }
}

impl<const SM: usize> ErrorType for PioUart<'_, SM> {
    type Error = Infallible;
}

/// This struct represents a uart tx program loaded into pio instruction memory.
pub struct PioUartTxProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioUartTxProgram<'a, PIO> {
    /// Load the uart tx program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
            .side_set 1 opt
                pull       side 1 [7]  ; Assert stop bit, or stall with line in idle state
                set x, 8   side 0 [7]  ; Preload bit counter, assert start bit for 9 clocks
            bitloop:                   ; This loop will run 9 times (9n1 UART)
                out pins, 1            ; Shift 1 bit from OSR to the first OUT pin
                jmp x-- bitloop   [6]  ; Each loop iteration is 8 cycles.
            "#
        );
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// PIO backed Uart transmitter
pub struct PioUartTx<'a, PIO: Instance, const SM: usize> {
    sm_tx: StateMachine<'a, PIO, SM>,
}

impl<'a, PIO: Instance, const SM: usize> PioUartTx<'a, PIO, SM> {
    /// Configure a pio state machine to use the loaded tx program.
    pub fn new(
        baud: u32,
        common: &mut Common<'a, PIO>,
        mut sm_tx: StateMachine<'a, PIO, SM>,
        tx_pin: impl PioPin,
        program: &PioUartTxProgram<'a, PIO>,
    ) -> Self {
        let tx_pin = common.make_pio_pin(tx_pin);
        sm_tx.set_pins(Level::High, &[&tx_pin]);
        sm_tx.set_pin_dirs(PioDirection::Out, &[&tx_pin]);

        let mut cfg = Config::default();

        cfg.set_out_pins(&[&tx_pin]);
        cfg.use_program(&program.prg, &[&tx_pin]);
        cfg.shift_out.auto_fill = false;
        cfg.shift_out.direction = ShiftDirection::Right;
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        sm_tx.set_config(&cfg);
        sm_tx.set_enable(true);

        Self { sm_tx }
    }

    /// Write 9 bit - it's the lowest 9 bits of the 16
    pub async fn write_u16(&mut self, data: u16) {
        self.sm_tx.tx().wait_push(data as u32).await;
    }
}

impl<PIO: Instance, const SM: usize> ErrorType for PioUartTx<'_, PIO, SM> {
    type Error = Infallible;
}

impl<PIO: Instance, const SM: usize> Write for PioUartTx<'_, PIO, SM> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Infallible> {
        for n in 0..buf.len() / 2 {
            self.write_u16(u16::from_le_bytes([buf[(n * 2) + 1], buf[n * 2] & 0x01]))
                .await;
        }
        Ok(buf.len())
    }
}

/// This struct represents a Uart Rx program loaded into pio instruction memory.
pub struct PioUartRxProgram<'a, PIO: Instance> {
    prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> PioUartRxProgram<'a, PIO> {
    /// Load the uart rx program into the given pio
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
            start:
                wait 0 pin 0        ; Stall until start bit is asserted
                set x, 8    [10]    ; Preload bit counter, then delay until halfway through
            bitloop:                ; the first data bit (12 cycles incl wait, set).
                in pins, 1          ; Shift data bit into ISR
                jmp x-- bitloop [6] ; Loop 8 more times, each loop iteration is 8 cycles
                jmp pin good_stop   ; Check stop bit (should be high)
                irq 4 rel           ; Either a framing error or a break. Set a sticky flag,
                wait 1 pin 0        ; and wait for line to return to idle state.
                jmp start           ; Don't push data if we didn't see good framing.
            good_stop:
                in null, 7	    ; Feed in 7 zeros to justify the 9 bits
		push		    ; Push bits bit
             "#
        );

        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

/// PIO backed Uart reciever
pub struct PioUartRx<'a, PIO: Instance, const SM: usize> {
    sm_rx: StateMachine<'a, PIO, SM>,
    timeout_firstbyte: Duration,
    timeout_interbyte: Duration,
}

impl<'a, PIO: Instance, const SM: usize> PioUartRx<'a, PIO, SM> {
    /// Configure a pio state machine to use the loaded rx program.
    pub fn new(
        baud: u32,
        timeout_firstbyte: Duration,
        timeout_interbyte: Duration,
        common: &mut Common<'a, PIO>,
        mut sm_rx: StateMachine<'a, PIO, SM>,
        rx_pin: impl PioPin,
        program: &PioUartRxProgram<'a, PIO>,
    ) -> Self {
        let mut cfg = Config::default();
        cfg.use_program(&program.prg, &[]);

        let rx_pin = common.make_pio_pin(rx_pin);
        sm_rx.set_pins(Level::High, &[&rx_pin]);
        cfg.set_in_pins(&[&rx_pin]);
        cfg.set_jmp_pin(&rx_pin);
        sm_rx.set_pin_dirs(PioDirection::In, &[&rx_pin]);

        cfg.clock_divider = (clk_sys_freq() / (8 * baud)).to_fixed();
        cfg.shift_in.auto_fill = false;
        cfg.shift_in.direction = ShiftDirection::Right;
        cfg.shift_in.threshold = 32;
        cfg.fifo_join = FifoJoin::RxOnly;
        sm_rx.set_config(&cfg);
        sm_rx.set_enable(true);

        Self {
            sm_rx,
            timeout_firstbyte,
            timeout_interbyte,
        }
    }

    /// Wait for a single u16
    pub async fn read_u16(&mut self) -> u16 {
        (self.sm_rx.rx().wait_pull().await >> 16) as u16 & 0x1FF
    }
}

impl<PIO: Instance, const SM: usize> ErrorType for PioUartRx<'_, PIO, SM> {
    type Error = Infallible;
}

impl<PIO: Instance, const SM: usize> Read for PioUartRx<'_, PIO, SM> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Infallible> {
        //We are a 9 bit uart, so we will need to return 2 bytes for every 9 bit 'read', so if the buffer
        //is <2, we cannot operate.
        if buf.len() < 2 {
            return Ok(0);
        }

        let mut i = 0;
        while i < buf.len() / 2 {
            let timeout = if i == 0 {
                self.timeout_firstbyte
            } else {
                self.timeout_interbyte
            };
            match self.read_u16().with_timeout(timeout).await {
                Ok(byte) => {
                    //Little endian
                    buf[2 * i] = (byte >> 8) as u8;
                    buf[2 * i + 1] = (byte & 0xFF) as u8;
                    i += 1;
                }
                Err(_timeout) => {
                    return Ok(i * 2);
                }
            }
        }
        Ok(i * 2)
    }
}
