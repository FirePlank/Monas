//! Firmware images: ELF (our firmware) and Intel HEX, including micro:bit "universal
//! hex" files, from which the nRF52833 (V2) part is taken.

use crate::machine::{Machine, FLASH_SIZE};
use std::collections::HashMap;

pub struct Image {
    /// (address, bytes) chunks.
    pub chunks: Vec<(u32, Vec<u8>)>,
    pub symbols: HashMap<String, (u32, u32)>,
}

impl Image {
    pub fn load(path: &str) -> Result<Image, String> {
        let data = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
        if data.starts_with(b"\x7fELF") {
            parse_elf(&data)
        } else {
            parse_hex(&String::from_utf8_lossy(&data))
        }
    }

    pub fn install(&self, m: &mut Machine) {
        for (addr, bytes) in &self.chunks {
            for (k, &b) in bytes.iter().enumerate() {
                let a = addr + k as u32;
                if a < FLASH_SIZE {
                    m.flash[a as usize] = b;
                } else if (0x1000_1000..0x1000_2000).contains(&a) {
                    m.uicr[(a - 0x1000_1000) as usize] = b;
                }
            }
        }
        m.invalidate_decode_flash();
    }
}

fn u16le(d: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([d[o], d[o + 1]])
}
fn u32le(d: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([d[o], d[o + 1], d[o + 2], d[o + 3]])
}

fn parse_elf(d: &[u8]) -> Result<Image, String> {
    if d[4] != 1 || d[5] != 1 {
        return Err("not a 32-bit little-endian ELF".into());
    }
    let phoff = u32le(d, 28) as usize;
    let shoff = u32le(d, 32) as usize;
    let phentsize = u16le(d, 42) as usize;
    let phnum = u16le(d, 44) as usize;
    let shentsize = u16le(d, 46) as usize;
    let shnum = u16le(d, 48) as usize;
    let mut chunks = Vec::new();
    for k in 0..phnum {
        let p = phoff + k * phentsize;
        let ptype = u32le(d, p);
        let offset = u32le(d, p + 4) as usize;
        let paddr = u32le(d, p + 12);
        let filesz = u32le(d, p + 16) as usize;
        if ptype == 1 && filesz > 0 {
            chunks.push((paddr, d[offset..offset + filesz].to_vec()));
        }
    }
    let mut symbols = HashMap::new();
    for k in 0..shnum {
        let s = shoff + k * shentsize;
        let stype = u32le(d, s + 4);
        if stype == 2 {
            let off = u32le(d, s + 16) as usize;
            let size = u32le(d, s + 20) as usize;
            let link = u32le(d, s + 24) as usize;
            let ent = u32le(d, s + 36) as usize;
            let strs = shoff + link * shentsize;
            let stroff = u32le(d, strs + 16) as usize;
            for j in 0..size / ent.max(1) {
                let e = off + j * ent;
                let name_off = u32le(d, e) as usize;
                let value = u32le(d, e + 4);
                let sz = u32le(d, e + 8);
                let mut end = stroff + name_off;
                while end < d.len() && d[end] != 0 {
                    end += 1;
                }
                let name = String::from_utf8_lossy(&d[stroff + name_off..end]).into_owned();
                if !name.is_empty() {
                    symbols.insert(name, (value, sz));
                }
            }
        }
    }
    Ok(Image { chunks, symbols })
}

fn parse_hex(text: &str) -> Result<Image, String> {
    let mut chunks: Vec<(u32, Vec<u8>)> = Vec::new();
    let mut upper = 0u32;
    let mut block_board: Option<u16> = None;
    for line in text.lines() {
        let line = line.trim();
        if !line.starts_with(':') || line.len() < 11 {
            continue;
        }
        let bytes: Vec<u8> = (1..line.len())
            .step_by(2)
            .filter_map(|i| line.get(i..i + 2).and_then(|h| u8::from_str_radix(h, 16).ok()))
            .collect();
        let n = bytes[0] as usize;
        let addr = ((bytes[1] as u32) << 8) | bytes[2] as u32;
        let rtype = bytes[3];
        if bytes.len() < 5 + n {
            continue;
        }
        let payload = &bytes[4..4 + n];
        match rtype {
            0x00 | 0x0D => {
                // 0x0D carries V2 data inside a universal hex block.
                if matches!(block_board, Some(0x9900) | Some(0x9901)) {
                    continue;
                }
                let a = upper + addr;
                if let Some(last) = chunks.last_mut() {
                    if last.0 + last.1.len() as u32 == a {
                        last.1.extend_from_slice(payload);
                        continue;
                    }
                }
                chunks.push((a, payload.to_vec()));
            }
            0x02 => upper = (((payload[0] as u32) << 8) | payload[1] as u32) << 4,
            0x04 => upper = (((payload[0] as u32) << 8) | payload[1] as u32) << 16,
            0x0A => block_board = Some(((payload[0] as u16) << 8) | payload[1] as u16),
            0x0B => block_board = None,
            _ => {}
        }
    }
    if chunks.is_empty() {
        return Err("no data in hex file".into());
    }
    Ok(Image { chunks, symbols: HashMap::new() })
}
