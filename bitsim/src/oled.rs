//! SSD1306 128x64 OLED controller (as on the Kitronik :VIEW 128x64), attached to the
//! emulated I2C bus at address 0x3C.

pub struct Ssd1306 {
    pub fb: [u8; 1024],
    col_start: u8,
    col_end: u8,
    page_start: u8,
    page_end: u8,
    col: u8,
    page: u8,
    page_mode: bool,
    pub on: bool,
}

impl Default for Ssd1306 {
    fn default() -> Self {
        Ssd1306 {
            fb: [0; 1024],
            col_start: 0,
            col_end: 127,
            page_start: 0,
            page_end: 7,
            col: 0,
            page: 0,
            page_mode: true,
            on: false,
        }
    }
}

impl Ssd1306 {
    /// One I2C write transaction: control byte, then commands or display data.
    pub fn write(&mut self, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        if data[0] & 0x40 != 0 {
            for &b in &data[1..] {
                self.fb[(self.page as usize & 7) * 128 + (self.col as usize & 127)] = b;
                if self.page_mode {
                    self.col = (self.col + 1) & 127;
                } else if self.col >= self.col_end {
                    self.col = self.col_start;
                    self.page = if self.page >= self.page_end { self.page_start } else { self.page + 1 };
                } else {
                    self.col += 1;
                }
            }
            return;
        }
        let c = &data[1..];
        let mut i = 0;
        while i < c.len() {
            let cmd = c[i];
            let arg = |k: usize| *c.get(i + k).unwrap_or(&0);
            match cmd {
                0x00..=0x0F => self.col = (self.col & 0xF0) | cmd,
                0x10..=0x1F => self.col = (self.col & 0x0F) | ((cmd & 0x0F) << 4),
                0xB0..=0xB7 => self.page = cmd & 7,
                0x20 => {
                    self.page_mode = arg(1) & 3 == 2;
                    i += 1;
                }
                0x21 => {
                    self.col_start = arg(1) & 127;
                    self.col_end = arg(2) & 127;
                    self.col = self.col_start;
                    i += 2;
                }
                0x22 => {
                    self.page_start = arg(1) & 7;
                    self.page_end = arg(2) & 7;
                    self.page = self.page_start;
                    i += 2;
                }
                0xAE => self.on = false,
                0xAF => self.on = true,
                0x81 | 0x8D | 0xA8 | 0xD3 | 0xD5 | 0xD9 | 0xDA | 0xDB | 0xD6 => i += 1,
                _ => {}
            }
            i += 1;
        }
    }

    fn px(&self, x: usize, y: usize) -> bool {
        (self.fb[(y / 8) * 128 + x] >> (y % 8)) & 1 != 0
    }

    /// The screen as text, two pixel rows per character cell.
    pub fn render(&self) -> String {
        let mut s = String::new();
        s.push('+');
        s.push_str(&"-".repeat(128));
        s.push_str("+\n");
        for row in 0..32 {
            s.push('|');
            for x in 0..128 {
                let (a, b) = (self.px(x, 2 * row), self.px(x, 2 * row + 1));
                s.push(match (a, b) {
                    (true, true) => '\u{2588}',
                    (true, false) => '\u{2580}',
                    (false, true) => '\u{2584}',
                    _ => ' ',
                });
            }
            s.push_str("|\n");
        }
        s.push('+');
        s.push_str(&"-".repeat(128));
        s.push('+');
        s
    }
}
