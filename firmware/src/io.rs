//! Drivers for the add-ons used by the standalone UI: the Kitronik :VIEW 128x64 OLED
//! (SSD1306 on the edge-connector I2C bus, P19/P20, address 0x3C) and the ELECFREAKS
//! joystick:bit (stick on P1/P2, buttons C/D/E/F on P12..P15, active low).

use crate::font::FONT;
use core::ptr::{read_volatile, write_volatile};

#[inline(always)]
fn wr(addr: usize, v: u32) {
    unsafe { write_volatile(addr as *mut u32, v) }
}
#[inline(always)]
fn rd(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

const P0: usize = 0x5000_0000;
const P1: usize = 0x5000_0300;
const TWIM0: usize = 0x4000_3000;
const SAADC: usize = 0x4000_7000;

const PIN_CNF: usize = 0x700;
const OUTSET: usize = 0x508;
const OUTCLR: usize = 0x50C;
const IN: usize = 0x510;
const DIRSET: usize = 0x518;

// micro:bit v2 edge connector -> nRF52833 pins
const PIN_SCL: u32 = 26; // P19 = P0.26
const PIN_SDA: u32 = 32; // P20 = P1.00
const PIN_C: u32 = 12; // P12 = P0.12
const PIN_D: u32 = 17; // P13 = P0.17
const PIN_E: u32 = 1; // P14 = P0.01
const PIN_F: u32 = 13; // P15 = P0.13
const PIN_EDGE0: u32 = 2; // P0 = P0.02 (joystick:bit buzzer, held low)
const PIN_P16: u32 = 2; // P16 = P1.02 (vibration motor, held high = off)
const PIN_BTN_A: u32 = 14; // micro:bit button A = P0.14
const PIN_BTN_B: u32 = 23; // micro:bit button B = P0.23
const AIN_X: u32 = 2; // P1 = P0.03 = AIN1 -> PSELP value 2
const AIN_Y: u32 = 3; // P2 = P0.04 = AIN2 -> PSELP value 3

const OLED_ADDR: u32 = 0x3C;

static mut I2C_BUF: [u8; 132] = [0; 132];
static mut ADC_BUF: [i16; 2] = [0; 2];

fn port_of(pin: u32) -> (usize, u32) {
    if pin >= 32 {
        (P1, pin - 32)
    } else {
        (P0, pin)
    }
}

fn input_pullup(pin: u32) {
    let (p, n) = port_of(pin);
    wr(p + PIN_CNF + 4 * n as usize, 3 << 2); // input, buffer connected, pull-up
}

fn input_plain(pin: u32) {
    let (p, n) = port_of(pin);
    wr(p + PIN_CNF + 4 * n as usize, 0);
}

fn output(pin: u32, high: bool) {
    let (p, n) = port_of(pin);
    wr(p + if high { OUTSET } else { OUTCLR }, 1 << n);
    wr(p + DIRSET, 1 << n);
}

fn pin_low(pin: u32) -> bool {
    let (p, n) = port_of(pin);
    rd(p + IN) & (1 << n) == 0
}

/// Pins for the joystick:bit, as its MakeCode extension's `initJoystickBit` sets them.
pub fn joystick_init() {
    output(PIN_EDGE0, false);
    for pin in [PIN_C, PIN_D, PIN_E, PIN_F] {
        input_pullup(pin);
    }
    output(32 + PIN_P16, true);
    input_plain(PIN_BTN_A);
    input_plain(PIN_BTN_B);
}

pub fn button_c() -> bool {
    pin_low(PIN_C)
}
pub fn button_d() -> bool {
    pin_low(PIN_D)
}
pub fn button_e() -> bool {
    pin_low(PIN_E)
}
pub fn button_b() -> bool {
    pin_low(PIN_BTN_B)
}

/// One 10-bit conversion (0..1023 over 0..VDD, as MakeCode's analogReadPin).
fn adc_read(psel: u32) -> u32 {
    unsafe {
        let buf = core::ptr::addr_of_mut!(ADC_BUF) as *mut i16;
        write_volatile(buf, 0);
        wr(SAADC + 0x510, psel); // CH[0].PSELP
        wr(SAADC + 0x514, 0); // CH[0].PSELN
        wr(SAADC + 0x518, (5 << 8) | (1 << 12) | (2 << 16)); // gain 1/4, ref VDD/4, 10 us
        wr(SAADC + 0x5F0, 1); // 10 bit
        wr(SAADC + 0x5F4, 0);
        wr(SAADC + 0x5F8, 0);
        wr(SAADC + 0x62C, buf as u32);
        wr(SAADC + 0x630, 1);
        wr(SAADC + 0x500, 1);
        wr(SAADC + 0x100, 0);
        wr(SAADC + 0x104, 0);
        wr(SAADC + 0x114, 0);
        wr(SAADC, 1); // START
        let mut n = 0;
        while rd(SAADC + 0x100) == 0 && n < 10_000 {
            n += 1;
        }
        wr(SAADC + 0x004, 1); // SAMPLE
        n = 0;
        while rd(SAADC + 0x104) == 0 && n < 10_000 {
            n += 1;
        }
        wr(SAADC + 0x008, 1); // STOP
        n = 0;
        while rd(SAADC + 0x114) == 0 && n < 10_000 {
            n += 1;
        }
        wr(SAADC + 0x500, 0);
        wr(SAADC + 0x510, 0);
        read_volatile(buf).clamp(0, 1023) as u32
    }
}

pub fn joystick_x() -> u32 {
    adc_read(AIN_X)
}
pub fn joystick_y() -> u32 {
    adc_read(AIN_Y)
}

// ---- I2C (TWIM0) ----

pub fn i2c_init() {
    for pin in [PIN_SCL, PIN_SDA] {
        let (p, n) = port_of(pin);
        // input buffer connected, pull-up, standard-0 / disconnect-1 drive
        wr(p + PIN_CNF + 4 * n as usize, (3 << 2) | (6 << 8));
    }
    wr(TWIM0 + 0x500, 0);
    wr(TWIM0 + 0x508, PIN_SCL); // PSEL.SCL
    wr(TWIM0 + 0x50C, PIN_SDA); // PSEL.SDA
    wr(TWIM0 + 0x524, 0x0640_0000); // 400 kHz
    wr(TWIM0 + 0x500, 6);
}

/// Writes `data` (at most 132 bytes) to `addr`; false if the device did not answer.
fn i2c_write(addr: u32, data: &[u8]) -> bool {
    unsafe {
        let buf = core::ptr::addr_of_mut!(I2C_BUF) as *mut u8;
        for (k, &b) in data.iter().enumerate() {
            write_volatile(buf.add(k), b);
        }
        wr(TWIM0 + 0x588, addr);
        wr(TWIM0 + 0x544, buf as u32);
        wr(TWIM0 + 0x548, data.len() as u32);
        wr(TWIM0 + 0x104, 0); // STOPPED
        wr(TWIM0 + 0x124, 0); // ERROR
        wr(TWIM0 + 0x160, 0); // LASTTX
        wr(TWIM0 + 0x200, 1 << 9); // LASTTX -> STOP
        wr(TWIM0 + 0x008, 1); // STARTTX
        let mut n = 0u32;
        loop {
            if rd(TWIM0 + 0x104) != 0 {
                break;
            }
            if rd(TWIM0 + 0x124) != 0 {
                wr(TWIM0 + 0x014, 1); // STOP
                let mut k = 0;
                while rd(TWIM0 + 0x104) == 0 && k < 100_000 {
                    k += 1;
                }
                wr(TWIM0 + 0x4C4, 0xF); // clear ERRORSRC
                wr(TWIM0 + 0x124, 0);
                return false;
            }
            n += 1;
            if n > 2_000_000 {
                return false;
            }
        }
        wr(TWIM0 + 0x200, 0);
        rd(TWIM0 + 0x4C4) == 0
    }
}

// ---- OLED ----

pub struct Oled {
    pub present: bool,
}

impl Oled {
    /// Probes and initialises the display (same command sequence as Kitronik's driver).
    pub fn init() -> Oled {
        i2c_init();
        if !i2c_write(OLED_ADDR, &[0x00, 0xAF]) {
            return Oled { present: false };
        }
        let cmds: [&[u8]; 19] = [
            &[0xAE],
            &[0xA4],
            &[0xD5, 0xF0],
            &[0xA8, 0x3F],
            &[0xD3, 0x00],
            &[0x40],
            &[0x8D, 0x14],
            &[0x20, 0x00],
            &[0x21, 0, 127],
            &[0x22, 0, 63],
            &[0xA1],
            &[0xC8],
            &[0xDA, 0x12],
            &[0x81, 0xCF],
            &[0xD9, 0xF1],
            &[0xDB, 0x40],
            &[0xA6],
            &[0xD6, 0x00],
            &[0xAF],
        ];
        for c in cmds.iter() {
            let mut b = [0u8; 4];
            b[1..1 + c.len()].copy_from_slice(c);
            i2c_write(OLED_ADDR, &b[..1 + c.len()]);
        }
        let o = Oled { present: true };
        o.clear();
        o
    }

    /// Writes one 25-character text line to page `line` (1..8, like Kitronik's `show`).
    pub fn line(&self, line: u8, text: &[u8]) {
        if !self.present {
            return;
        }
        let page = line.clamp(1, 8) - 1;
        i2c_write(OLED_ADDR, &[0x00, 0x21, 0, 127]);
        i2c_write(OLED_ADDR, &[0x00, 0x22, page, page]);
        let mut buf = [0u8; 129];
        buf[0] = 0x40;
        for c in 0..25 {
            let ch = *text.get(c).unwrap_or(&b' ');
            let glyph = if (32..128).contains(&ch) { FONT[(ch - 32) as usize] } else { 0 };
            for x in 0..5 {
                let mut col = 0u8;
                for y in 0..5 {
                    if glyph & (1 << (5 * x + y)) != 0 {
                        col |= 1 << (y + 1);
                    }
                }
                buf[1 + c * 5 + x] = col;
            }
        }
        i2c_write(OLED_ADDR, &buf);
    }

    pub fn clear(&self) {
        for l in 1..=8 {
            self.line(l, b"");
        }
    }
}
