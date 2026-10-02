//! nRF52833 / micro:bit v2 hardware: clocks, UART (USB serial via the interface chip),
//! DWT cycle counter and the LED matrix. Direct register access only.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, AtomicUsize, Ordering};

#[inline(always)]
fn wr(addr: usize, v: u32) {
    unsafe { write_volatile(addr as *mut u32, v) }
}
#[inline(always)]
fn rd(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

const CLOCK: usize = 0x4000_0000;
const NVMC: usize = 0x4001_E000;
const UART0: usize = 0x4000_2000;
const P0: usize = 0x5000_0000;
const P1: usize = 0x5000_0300;
const DEMCR: usize = 0xE000_EDFC;
const DWT_CTRL: usize = 0xE000_1000;
const DWT_CYCCNT: usize = 0xE000_1004;
const NVIC_ISER0: usize = 0xE000_E100;

// GPIO register offsets
const OUTSET: usize = 0x508;
const OUTCLR: usize = 0x50C;
const DIRSET: usize = 0x518;
const PIN_CNF: usize = 0x700;

// UART (legacy, non-DMA) register offsets
const TASKS_STARTRX: usize = 0x000;
const TASKS_STARTTX: usize = 0x008;
const EVENTS_RXDRDY: usize = 0x108;
const EVENTS_TXDRDY: usize = 0x11C;
const EVENTS_ERROR: usize = 0x124;
const ERRORSRC: usize = 0x480;
const INTENSET: usize = 0x304;
const ENABLE: usize = 0x500;
const PSEL_TXD: usize = 0x50C;
const PSEL_RXD: usize = 0x514;
const RXD: usize = 0x518;
const TXD: usize = 0x51C;
const BAUDRATE: usize = 0x524;
const CONFIG: usize = 0x56C;

// micro:bit v2 pins
const UART_TX_PIN: u32 = 6; // P0.06 -> interface MCU
const UART_RX_PIN: u32 = 32 + 8; // P1.08 <- interface MCU
const ROWS: [(usize, u32); 5] = [(P0, 21), (P0, 22), (P0, 15), (P0, 24), (P0, 19)];
const COLS: [(usize, u32); 5] = [(P0, 28), (P0, 11), (P0, 31), (P1, 5), (P0, 30)];

pub fn init() {
    // Crystal oscillator for an accurate clock and UART baud rate.
    wr(CLOCK + 0x100, 0); // EVENTS_HFCLKSTARTED
    wr(CLOCK, 1); // TASKS_HFCLKSTART
    let mut spins = 0u32;
    while rd(CLOCK + 0x100) == 0 && spins < 2_000_000 {
        spins += 1;
    }
    // Instruction cache for the code that stays in flash.
    wr(NVMC + 0x540, 1);

    // Cycle counter.
    wr(DEMCR, rd(DEMCR) | (1 << 24));
    wr(DWT_CYCCNT, 0);
    wr(DWT_CTRL, rd(DWT_CTRL) | 1);

    // LED matrix pins as outputs, everything off (rows low, columns high).
    for &(port, pin) in ROWS.iter() {
        wr(port + OUTCLR, 1 << pin);
        wr(port + DIRSET, 1 << pin);
    }
    for &(port, pin) in COLS.iter() {
        wr(port + OUTSET, 1 << pin);
        wr(port + DIRSET, 1 << pin);
    }

    // UART: TX pin output high, RX pin input with buffer connected.
    wr(P0 + OUTSET, 1 << UART_TX_PIN);
    wr(P0 + DIRSET, 1 << UART_TX_PIN);
    wr(P1 + PIN_CNF + 4 * 8, 0);
    wr(UART0 + PSEL_TXD, UART_TX_PIN);
    wr(UART0 + PSEL_RXD, UART_RX_PIN);
    wr(UART0 + BAUDRATE, 0x01D7_E000); // 115200
    wr(UART0 + CONFIG, 0);
    wr(UART0 + ENABLE, 4);
    wr(UART0 + EVENTS_RXDRDY, 0);
    wr(UART0 + EVENTS_TXDRDY, 0);
    wr(UART0 + TASKS_STARTTX, 1);
    wr(UART0 + TASKS_STARTRX, 1);
    wr(UART0 + INTENSET, (1 << 2) | (1 << 7)); // RXDRDY, TXDRDY
    wr(NVIC_ISER0, 1 << 2); // UARTE0_UART0
}

// ---- transmit ring buffer (producer: main code; consumer: UART interrupt) ----

const TX_RING: usize = 1024;
static mut TX_BUF: [u8; TX_RING] = [0; TX_RING];
static TX_HEAD: AtomicUsize = AtomicUsize::new(0);
static TX_TAIL: AtomicUsize = AtomicUsize::new(0);
static TX_BUSY: AtomicUsize = AtomicUsize::new(0);

/// Queues a byte for transmission; the UART interrupt feeds the hardware, so printing
/// does not stall the search (each character takes 87 us on the wire at 115200 baud).
pub fn uart_tx(b: u8) {
    loop {
        unsafe { core::arch::asm!("cpsid i") };
        compiler_fence(Ordering::SeqCst);
        let h = TX_HEAD.load(Ordering::Relaxed);
        let next = (h + 1) % TX_RING;
        if next != TX_TAIL.load(Ordering::Relaxed) {
            if TX_BUSY.load(Ordering::Relaxed) == 0 {
                TX_BUSY.store(1, Ordering::Relaxed);
                wr(UART0 + TXD, b as u32);
            } else {
                unsafe { write_volatile(core::ptr::addr_of_mut!(TX_BUF[h]), b) };
                TX_HEAD.store(next, Ordering::Relaxed);
            }
            compiler_fence(Ordering::SeqCst);
            unsafe { core::arch::asm!("cpsie i") };
            return;
        }
        // Buffer full: let the interrupt drain it.
        compiler_fence(Ordering::SeqCst);
        unsafe { core::arch::asm!("cpsie i") };
        unsafe { core::arch::asm!("nop", "nop", "nop", "nop") };
    }
}

/// TXDRDY: hand the next queued byte to the UART (interrupt context).
#[link_section = ".boot.uart"]
fn tx_pump() {
    if rd(UART0 + EVENTS_TXDRDY) != 0 {
        wr(UART0 + EVENTS_TXDRDY, 0);
        let t = TX_TAIL.load(Ordering::Relaxed);
        if t != TX_HEAD.load(Ordering::Relaxed) {
            let b = unsafe { read_volatile(core::ptr::addr_of!(TX_BUF[t])) };
            TX_TAIL.store((t + 1) % TX_RING, Ordering::Relaxed);
            wr(UART0 + TXD, b as u32);
        } else {
            TX_BUSY.store(0, Ordering::Relaxed);
        }
    }
}

/// UART interrupt: receive and transmit.
#[link_section = ".boot.uart"]
pub fn uart_irq() {
    rx_pump();
    tx_pump();
}

// ---- receive ring buffer (single producer: rx_pump; single consumer: main) ----

const RING: usize = 512;
static mut RX_BUF: [u8; RING] = [0; RING];
static RX_HEAD: AtomicUsize = AtomicUsize::new(0);
static RX_TAIL: AtomicUsize = AtomicUsize::new(0);

/// Moves received bytes into the ring buffer. Called from the UART interrupt, and from
/// the main loop with interrupts masked.
#[inline(never)]
#[link_section = ".boot.uart"]
pub fn rx_pump() {
    while rd(UART0 + EVENTS_RXDRDY) != 0 {
        wr(UART0 + EVENTS_RXDRDY, 0);
        let b = rd(UART0 + RXD) as u8;
        let h = RX_HEAD.load(Ordering::Relaxed);
        let next = (h + 1) % RING;
        if next != RX_TAIL.load(Ordering::Acquire) {
            unsafe { write_volatile(core::ptr::addr_of_mut!(RX_BUF[h]), b) };
            RX_HEAD.store(next, Ordering::Release);
        }
    }
    if rd(UART0 + EVENTS_ERROR) != 0 {
        wr(UART0 + EVENTS_ERROR, 0);
        wr(UART0 + ERRORSRC, 0xF);
    }
}

pub fn rx_pump_masked() {
    unsafe { core::arch::asm!("cpsid i") };
    compiler_fence(Ordering::SeqCst);
    rx_pump();
    compiler_fence(Ordering::SeqCst);
    unsafe { core::arch::asm!("cpsie i") };
}

/// Sleeps until an interrupt arrives, unless input is already waiting. Interrupts are
/// masked around the check so a byte arriving in between still wakes the core.
pub fn idle_wait() {
    unsafe { core::arch::asm!("cpsid i") };
    compiler_fence(Ordering::SeqCst);
    let empty = RX_TAIL.load(Ordering::Relaxed) == RX_HEAD.load(Ordering::Acquire);
    if empty && rd(UART0 + EVENTS_RXDRDY) == 0 {
        unsafe { core::arch::asm!("wfi") };
    }
    compiler_fence(Ordering::SeqCst);
    unsafe { core::arch::asm!("cpsie i") };
}

pub fn rx_available() -> bool {
    RX_TAIL.load(Ordering::Relaxed) != RX_HEAD.load(Ordering::Acquire)
}

pub fn rx_pop() -> Option<u8> {
    let t = RX_TAIL.load(Ordering::Relaxed);
    if t == RX_HEAD.load(Ordering::Acquire) {
        return None;
    }
    let b = unsafe { read_volatile(core::ptr::addr_of!(RX_BUF[t])) };
    RX_TAIL.store((t + 1) % RING, Ordering::Release);
    Some(b)
}

// ---- clock ----

static mut CYC_LAST: u32 = 0;
static mut CYC_TOTAL: u64 = 0;

/// CPU cycles since boot, extended to 64 bits (callers poll far more often than the
/// 67 s wrap period of the 32-bit counter).
pub fn cycles() -> u64 {
    unsafe {
        let c = rd(DWT_CYCCNT);
        let last = core::ptr::addr_of_mut!(CYC_LAST);
        let total = core::ptr::addr_of_mut!(CYC_TOTAL);
        *total += c.wrapping_sub(*last) as u64;
        *last = c;
        *total
    }
}

// ---- LEDs ----

fn led(row: usize, col: usize, on: bool) {
    let (rp, rpin) = ROWS[row];
    let (cp, cpin) = COLS[col];
    if on {
        wr(rp + OUTSET, 1 << rpin);
        wr(cp + OUTCLR, 1 << cpin);
    } else {
        wr(rp + OUTCLR, 1 << rpin);
        wr(cp + OUTSET, 1 << cpin);
    }
}

fn leds_off() {
    for &(rp, rpin) in ROWS.iter() {
        wr(rp + OUTCLR, 1 << rpin);
    }
    for &(cp, cpin) in COLS.iter() {
        wr(cp + OUTSET, 1 << cpin);
    }
}

static mut TICK: u32 = 0;

/// Centre LED: waiting for a command.
pub fn led_idle() {
    leds_off();
    led(2, 2, true);
}

/// Thinking: a dot that walks around the border, advanced from the search's polling.
pub fn led_busy() {
    leds_off();
    led(0, 0, true);
}

pub fn led_tick() {
    const RING_POS: [(usize, usize); 16] = [
        (0, 0),
        (0, 1),
        (0, 2),
        (0, 3),
        (0, 4),
        (1, 4),
        (2, 4),
        (3, 4),
        (4, 4),
        (4, 3),
        (4, 2),
        (4, 1),
        (4, 0),
        (3, 0),
        (2, 0),
        (1, 0),
    ];
    unsafe {
        let t = core::ptr::addr_of_mut!(TICK);
        *t = (*t).wrapping_add(1);
        if (*t).is_multiple_of(8) {
            leds_off();
            let (r, c) = RING_POS[((*t / 8) % 16) as usize];
            led(r, c, true);
        }
    }
}

/// Every LED on: a fault (panic or hard fault). Never returns.
#[link_section = ".boot"]
pub fn led_fault() -> ! {
    for &(rp, rpin) in ROWS.iter() {
        wr(rp + DIRSET, 1 << rpin);
        wr(rp + OUTSET, 1 << rpin);
    }
    for &(cp, cpin) in COLS.iter() {
        wr(cp + DIRSET, 1 << cpin);
        wr(cp + OUTCLR, 1 << cpin);
    }
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}

/// Bytes of painted RAM below the stack that were never overwritten.
pub fn stack_free() -> usize {
    extern "C" {
        static mut __bss_end: u32;
    }
    let mut p = core::ptr::addr_of_mut!(__bss_end) as *const u32;
    let mut n = 0;
    unsafe {
        while read_volatile(p) == 0xDEAD_BEEF {
            n += 4;
            p = p.add(1);
        }
    }
    n
}
