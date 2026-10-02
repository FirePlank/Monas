//! Thumb / Thumb-2 (ARMv7E-M with the single-precision FPU) decoder.
//! Encodings follow the ARMv7-M Architecture Reference Manual (DDI 0403E), chapter A5
//! (A5.2 16-bit, A5.3 32-bit) and A6/A7 for the FPU. Each instruction is decoded once
//! into an `Inst` and cached by address.

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[repr(u8)]
pub enum Op {
    #[default]
    Undecoded,
    Undefined,
    // Data processing with a flexible second operand (see `F_*` flags).
    And,
    Eor,
    Orr,
    Orn,
    Bic,
    Mov,
    Mvn,
    Tst,
    Teq,
    Add,
    Adc,
    Sub,
    Sbc,
    Rsb,
    Cmp,
    Cmn,
    // Plain immediates, no flags.
    AddW,
    SubW,
    Movw,
    Movt,
    Adr,
    // Multiplies and divides.
    Mul,
    Mla,
    Mls,
    Umull,
    Smull,
    Umlal,
    Smlal,
    Umaal,
    Sdiv,
    Udiv,
    Smlaxy,
    Smlawy,
    Smlad,
    Smlsd,
    Smmla,
    Smmls,
    Smlald,
    Smlsld,
    Smlalxy,
    Usada8,
    // Parallel add/subtract (sub = kind, see exec).
    Parallel,
    Qadd,
    Qsub,
    Qdadd,
    Qdsub,
    Ssat,
    Usat,
    Ssat16,
    Usat16,
    Sbfx,
    Ubfx,
    Bfi,
    Pkh,
    /// Sign/zero extends with optional accumulate: sub = 0 sxth, 1 uxth, 2 sxtb16,
    /// 3 uxtb16, 4 sxtb, 5 uxtb; rn = 15 means no accumulate.
    Extend,
    Clz,
    Rbit,
    Rev,
    Rev16,
    Revsh,
    Sel,
    // Memory.
    Ldr,
    Str,
    Ldrd,
    Strd,
    Ldrex,
    Strex,
    Clrex,
    Ldm,
    Stm,
    Tbb,
    // Branches and control.
    B,
    Bl,
    Bx,
    Blx,
    Cbz,
    It,
    Mrs,
    Msr,
    Cps,
    Nop,
    Wfi,
    Wfe,
    Sev,
    Barrier,
    Svc,
    Bkpt,
    // FPU.
    Vldr,
    Vstr,
    Vldm,
    Vstm,
    VmovSR,
    VmovRRS,
    VmovRRD,
    Vmrs,
    Vmsr,
    Vfp,
}

/// Data-processing flags.
pub const F_S: u8 = 1; // set flags
pub const F_S_NOIT: u8 = 2; // set flags only outside an IT block (16-bit encodings)
pub const F_IMM: u8 = 4; // operand 2 is `imm`
pub const F_IMMC: u8 = 8; // the expanded immediate sets carry to its bit 31
pub const F_REGSH: u8 = 16; // operand 2 is rm shifted by register ra

/// Memory addressing flags.
pub const M_P: u8 = 1; // index (pre)
pub const M_U: u8 = 2; // add offset
pub const M_W: u8 = 4; // write back
pub const M_REG: u8 = 8; // register offset (rm << sh_n)
pub const M_LIT: u8 = 16; // PC-relative literal

/// Shift types (also in `sub` for operand-2 shifts).
pub const SH_LSL: u8 = 0;
pub const SH_LSR: u8 = 1;
pub const SH_ASR: u8 = 2;
pub const SH_ROR: u8 = 3;
pub const SH_RRX: u8 = 4;

/// FP data-processing kinds (in `sub` for `Op::Vfp`).
pub mod vfp {
    pub const VMLA: u8 = 0;
    pub const VMLS: u8 = 1;
    pub const VNMLS: u8 = 2;
    pub const VNMLA: u8 = 3;
    pub const VMUL: u8 = 4;
    pub const VNMUL: u8 = 5;
    pub const VADD: u8 = 6;
    pub const VSUB: u8 = 7;
    pub const VDIV: u8 = 8;
    pub const VFNMS: u8 = 9;
    pub const VFNMA: u8 = 10;
    pub const VFMA: u8 = 11;
    pub const VFMS: u8 = 12;
    pub const VMOVI: u8 = 13;
    pub const VMOVR: u8 = 14;
    pub const VABS: u8 = 15;
    pub const VNEG: u8 = 16;
    pub const VSQRT: u8 = 17;
    pub const VCMP: u8 = 18;
    pub const VCMPE: u8 = 19;
    pub const VCMP0: u8 = 20;
    pub const VCMPE0: u8 = 21;
    pub const VCVT_F_S: u8 = 22; // signed int -> float
    pub const VCVT_F_U: u8 = 23;
    pub const VCVT_S_F: u8 = 24; // float -> signed int (round toward zero if flags & 1 else FPSCR mode)
    pub const VCVT_U_F: u8 = 25;
    pub const VCVT_FIX: u8 = 26; // fixed point (details in imm)
    pub const VCVTBT: u8 = 27; // half-precision conversions
}

#[derive(Clone, Copy, Default, Debug)]
pub struct Inst {
    pub op: Op,
    pub sub: u8,
    pub rd: u8,
    pub rn: u8,
    pub rm: u8,
    pub ra: u8,
    pub flags: u8,
    pub sh_n: u8,
    pub len: u8,
    pub cyc: u8,
    pub cond: u8,
    pub imm: u32,
}

impl Inst {
    fn new(op: Op, len: u8) -> Inst {
        Inst { op, len, cyc: 1, cond: 14, ..Default::default() }
    }
}

#[inline]
fn bits(x: u32, hi: u32, lo: u32) -> u32 {
    (x >> lo) & ((1 << (hi - lo + 1)) - 1)
}
#[inline]
fn bit(x: u32, b: u32) -> u32 {
    (x >> b) & 1
}
#[inline]
fn sext(x: u32, width: u32) -> u32 {
    let s = 32 - width;
    (((x << s) as i32) >> s) as u32
}

pub fn is_32bit(hw1: u16) -> bool {
    (hw1 >> 11) >= 0b11101
}

/// (value, sets_carry) for ThumbExpandImm.
fn thumb_expand_imm(imm12: u32) -> (u32, bool) {
    if bits(imm12, 11, 10) == 0 {
        let b = imm12 & 0xFF;
        let v = match bits(imm12, 9, 8) {
            0 => b,
            1 => (b << 16) | b,
            2 => (b << 24) | (b << 8),
            _ => (b << 24) | (b << 16) | (b << 8) | b,
        };
        (v, false)
    } else {
        let unrot = 0x80 | (imm12 & 0x7F);
        let rot = bits(imm12, 11, 7);
        (unrot.rotate_right(rot), true)
    }
}

/// DecodeImmShift: (type, amount).
fn decode_imm_shift(t: u32, imm5: u32) -> (u8, u8) {
    match t {
        0 => (SH_LSL, imm5 as u8),
        1 => (SH_LSR, if imm5 == 0 { 32 } else { imm5 as u8 }),
        2 => (SH_ASR, if imm5 == 0 { 32 } else { imm5 as u8 }),
        _ => {
            if imm5 == 0 {
                (SH_RRX, 1)
            } else {
                (SH_ROR, imm5 as u8)
            }
        }
    }
}

fn reglist_count(l: u32) -> u8 {
    l.count_ones() as u8
}

pub fn decode(hw1: u16, hw2: u16) -> Inst {
    if is_32bit(hw1) {
        decode32(hw1 as u32, hw2 as u32)
    } else {
        decode16(hw1 as u32)
    }
}

fn dp16(op: Op, rd: u32, rn: u32, rm: u32) -> Inst {
    let mut i = Inst::new(op, 2);
    i.rd = rd as u8;
    i.rn = rn as u8;
    i.rm = rm as u8;
    i.flags = F_S_NOIT;
    i
}

fn decode16(h: u32) -> Inst {
    let undef = Inst::new(Op::Undefined, 2);
    match bits(h, 15, 10) {
        0b000000..=0b001111 => {
            let opc = bits(h, 13, 9);
            let rd = bits(h, 2, 0);
            let rm = bits(h, 5, 3);
            match opc {
                0b00000..=0b01011 => {
                    // LSL/LSR/ASR immediate (shift kind in bits 12:11)
                    let (t, n) = decode_imm_shift(bits(h, 12, 11), bits(h, 10, 6));
                    let mut i = dp16(Op::Mov, rd, 0, rm);
                    i.sub = t;
                    i.sh_n = n;
                    if bits(h, 12, 11) == 0 && bits(h, 10, 6) == 0 {
                        i.sh_n = 0;
                    }
                    i
                }
                0b01100 | 0b01101 => {
                    let mut i = dp16(if opc == 0b01100 { Op::Add } else { Op::Sub }, rd, rm, bits(h, 8, 6));
                    i.sub = SH_LSL;
                    i
                }
                0b01110 | 0b01111 => {
                    let mut i = dp16(if opc == 0b01110 { Op::Add } else { Op::Sub }, rd, rm, 0);
                    i.flags |= F_IMM;
                    i.imm = bits(h, 8, 6);
                    i
                }
                _ => {
                    let rdn = bits(h, 10, 8);
                    let imm = h & 0xFF;
                    let (op, rn) = match bits(h, 12, 11) {
                        0 => (Op::Mov, 0),
                        1 => (Op::Cmp, rdn),
                        2 => (Op::Add, rdn),
                        _ => (Op::Sub, rdn),
                    };
                    let mut i = dp16(op, rdn, rn, 0);
                    i.flags |= F_IMM;
                    if op == Op::Cmp {
                        i.flags = F_S | F_IMM;
                    }
                    i.imm = imm;
                    i
                }
            }
        }
        0b010000 => {
            let rdn = bits(h, 2, 0);
            let rm = bits(h, 5, 3);
            let opc = bits(h, 9, 6);
            let mut i = match opc {
                0b0000 => dp16(Op::And, rdn, rdn, rm),
                0b0001 => dp16(Op::Eor, rdn, rdn, rm),
                0b0010 | 0b0011 | 0b0100 | 0b0111 => {
                    // shift by register: rd = rdn shifted by rm
                    let mut i = dp16(Op::Mov, rdn, 0, rdn);
                    i.flags |= F_REGSH;
                    i.ra = rm as u8;
                    i.sub = match opc {
                        0b0010 => SH_LSL,
                        0b0011 => SH_LSR,
                        0b0100 => SH_ASR,
                        _ => SH_ROR,
                    };
                    return i;
                }
                0b0101 => dp16(Op::Adc, rdn, rdn, rm),
                0b0110 => dp16(Op::Sbc, rdn, rdn, rm),
                0b1000 => {
                    let mut i = dp16(Op::Tst, 0, rdn, rm);
                    i.flags = F_S;
                    i
                }
                0b1001 => {
                    // RSB rd, rn, #0
                    let mut i = dp16(Op::Rsb, rdn, rm, 0);
                    i.flags |= F_IMM;
                    i.imm = 0;
                    return i;
                }
                0b1010 => {
                    let mut i = dp16(Op::Cmp, 0, rdn, rm);
                    i.flags = F_S;
                    i
                }
                0b1011 => {
                    let mut i = dp16(Op::Cmn, 0, rdn, rm);
                    i.flags = F_S;
                    i
                }
                0b1100 => dp16(Op::Orr, rdn, rdn, rm),
                0b1101 => {
                    let mut i = Inst::new(Op::Mul, 2);
                    i.rd = rdn as u8;
                    i.rn = rm as u8;
                    i.rm = rdn as u8;
                    i.flags = F_S_NOIT;
                    return i;
                }
                0b1110 => dp16(Op::Bic, rdn, rdn, rm),
                _ => dp16(Op::Mvn, rdn, 0, rm),
            };
            i.sub = SH_LSL;
            i.sh_n = 0;
            i
        }
        0b010001 => {
            let opc = bits(h, 9, 6);
            let rdn = (bit(h, 7) << 3) | bits(h, 2, 0);
            let rm = bits(h, 6, 3);
            match opc >> 2 {
                0b00 => {
                    let mut i = Inst::new(Op::Add, 2);
                    i.rd = rdn as u8;
                    i.rn = rdn as u8;
                    i.rm = rm as u8;
                    if rdn == 15 {
                        i.cyc = 3;
                    }
                    i
                }
                0b01 => {
                    let mut i = Inst::new(Op::Cmp, 2);
                    i.rn = rdn as u8;
                    i.rm = rm as u8;
                    i.flags = F_S;
                    i
                }
                0b10 => {
                    let mut i = Inst::new(Op::Mov, 2);
                    i.rd = rdn as u8;
                    i.rm = rm as u8;
                    if rdn == 15 {
                        i.cyc = 3;
                    }
                    i
                }
                _ => {
                    let mut i = Inst::new(if bit(h, 7) == 0 { Op::Bx } else { Op::Blx }, 2);
                    i.rm = rm as u8;
                    i.cyc = 3;
                    i
                }
            }
        }
        0b010010 | 0b010011 => {
            let mut i = Inst::new(Op::Ldr, 2);
            i.rd = bits(h, 10, 8) as u8;
            i.rn = 15;
            i.sub = 2;
            i.imm = (h & 0xFF) << 2;
            i.flags = M_P | M_U | M_LIT;
            i.cyc = 2;
            i
        }
        0b010100..=0b100111 => {
            let opa = bits(h, 15, 12);
            let opb = bits(h, 11, 9);
            let rt = bits(h, 2, 0) as u8;
            let rn = bits(h, 5, 3) as u8;
            let mut i;
            if opa == 0b0101 {
                // register offset
                let (op, sub) = match opb {
                    0 => (Op::Str, 2),
                    1 => (Op::Str, 1),
                    2 => (Op::Str, 0),
                    3 => (Op::Ldr, 4),
                    4 => (Op::Ldr, 2),
                    5 => (Op::Ldr, 1),
                    6 => (Op::Ldr, 0),
                    _ => (Op::Ldr, 5),
                };
                i = Inst::new(op, 2);
                i.sub = sub;
                i.rm = bits(h, 8, 6) as u8;
                i.flags = M_P | M_U | M_REG;
            } else {
                let load = bit(h, 11) == 1;
                let imm5 = bits(h, 10, 6);
                i = Inst::new(if load { Op::Ldr } else { Op::Str }, 2);
                i.flags = M_P | M_U;
                match opa {
                    0b0110 => {
                        i.sub = 2;
                        i.imm = imm5 << 2;
                    }
                    0b0111 => {
                        i.sub = 0;
                        i.imm = imm5;
                    }
                    0b1000 => {
                        i.sub = 1;
                        i.imm = imm5 << 1;
                    }
                    _ => {
                        // SP-relative
                        i.sub = 2;
                        i.imm = (h & 0xFF) << 2;
                        i.rd = bits(h, 10, 8) as u8;
                        i.rn = 13;
                        i.cyc = 2;
                        return i;
                    }
                }
            }
            i.rd = rt;
            i.rn = rn;
            i.cyc = 2;
            i
        }
        0b101000 | 0b101001 => {
            let mut i = Inst::new(Op::Adr, 2);
            i.rd = bits(h, 10, 8) as u8;
            i.imm = (h & 0xFF) << 2;
            i
        }
        0b101010 | 0b101011 => {
            let mut i = Inst::new(Op::AddW, 2);
            i.rd = bits(h, 10, 8) as u8;
            i.rn = 13;
            i.imm = (h & 0xFF) << 2;
            i
        }
        0b101100..=0b101111 => {
            let opc = bits(h, 11, 5);
            if opc >> 2 == 0b00000 {
                let mut i = Inst::new(Op::AddW, 2);
                i.rd = 13;
                i.rn = 13;
                i.imm = (h & 0x7F) << 2;
                return i;
            }
            if opc >> 2 == 0b00001 {
                let mut i = Inst::new(Op::SubW, 2);
                i.rd = 13;
                i.rn = 13;
                i.imm = (h & 0x7F) << 2;
                return i;
            }
            if matches!(opc >> 3, 0b0001 | 0b0011 | 0b1001 | 0b1011) {
                let mut i = Inst::new(Op::Cbz, 2);
                i.rn = bits(h, 2, 0) as u8;
                i.imm = (bit(h, 9) << 6) | (bits(h, 7, 3) << 1);
                i.sub = bit(h, 11) as u8; // 1 = CBNZ
                return i;
            }
            if opc >> 1 == 0b001000 || opc >> 1 == 0b001001 || opc >> 1 == 0b001010 || opc >> 1 == 0b001011 {
                let mut i = Inst::new(Op::Extend, 2);
                i.rd = bits(h, 2, 0) as u8;
                i.rm = bits(h, 5, 3) as u8;
                i.rn = 15;
                i.sub = match opc >> 1 {
                    0b001000 => 0, // SXTH
                    0b001001 => 4, // SXTB
                    0b001010 => 1, // UXTH
                    _ => 5,        // UXTB
                };
                return i;
            }
            if opc >> 4 == 0b010 {
                // PUSH = STMDB sp!, list
                let list = (h & 0xFF) | (bit(h, 8) << 14);
                let mut i = Inst::new(Op::Stm, 2);
                i.rn = 13;
                i.imm = list;
                i.flags = M_W; // DB
                i.cyc = 1 + reglist_count(list);
                return i;
            }
            if opc == 0b0110011 {
                let mut i = Inst::new(Op::Cps, 2);
                i.sub = bit(h, 4) as u8; // 1 = disable
                i.imm = bits(h, 1, 0); // I, F
                return i;
            }
            if opc >> 1 == 0b101000 || opc >> 1 == 0b101001 || opc >> 1 == 0b101011 {
                let mut i = Inst::new(
                    match opc >> 1 {
                        0b101000 => Op::Rev,
                        0b101001 => Op::Rev16,
                        _ => Op::Revsh,
                    },
                    2,
                );
                i.rd = bits(h, 2, 0) as u8;
                i.rm = bits(h, 5, 3) as u8;
                return i;
            }
            if opc >> 4 == 0b110 {
                // POP = LDMIA sp!, list
                let list = (h & 0xFF) | (bit(h, 8) << 15);
                let mut i = Inst::new(Op::Ldm, 2);
                i.rn = 13;
                i.imm = list;
                i.flags = M_U | M_W;
                i.cyc = 1 + reglist_count(list) + if bit(h, 8) == 1 { 2 } else { 0 };
                return i;
            }
            if opc >> 3 == 0b1110 {
                let mut i = Inst::new(Op::Bkpt, 2);
                i.imm = h & 0xFF;
                return i;
            }
            if opc >> 3 == 0b1111 {
                if h & 0xF != 0 {
                    let mut i = Inst::new(Op::It, 2);
                    i.imm = h & 0xFF;
                    return i;
                }
                return Inst::new(
                    match bits(h, 7, 4) {
                        2 => Op::Wfe,
                        3 => Op::Wfi,
                        4 => Op::Sev,
                        _ => Op::Nop,
                    },
                    2,
                );
            }
            undef
        }
        0b110000..=0b110011 => {
            let load = bit(h, 11) == 1;
            let rn = bits(h, 10, 8);
            let list = h & 0xFF;
            let mut i = Inst::new(if load { Op::Ldm } else { Op::Stm }, 2);
            i.rn = rn as u8;
            i.imm = list;
            // Writeback unless (load and rn in list).
            i.flags = M_U | if !load || list & (1 << rn) == 0 { M_W } else { 0 };
            i.cyc = 1 + reglist_count(list);
            i
        }
        0b110100..=0b110111 => {
            let cond = bits(h, 11, 8);
            if cond == 0b1110 {
                return undef;
            }
            if cond == 0b1111 {
                let mut i = Inst::new(Op::Svc, 2);
                i.imm = h & 0xFF;
                return i;
            }
            let mut i = Inst::new(Op::B, 2);
            i.cond = cond as u8;
            i.imm = sext((h & 0xFF) << 1, 9);
            i
        }
        0b111000 | 0b111001 => {
            let mut i = Inst::new(Op::B, 2);
            i.imm = sext((h & 0x7FF) << 1, 12);
            i
        }
        _ => undef,
    }
}

fn dp32(op: Op, rd: u32, rn: u32, s: bool) -> Inst {
    let mut i = Inst::new(op, 4);
    i.rd = rd as u8;
    i.rn = rn as u8;
    i.flags = if s { F_S } else { 0 };
    i
}

/// Opcode table shared by the modified-immediate and shifted-register forms.
/// Returns None for unallocated opcodes.
fn dp_op(op: u32, rn: u32, rd: u32, s: bool) -> Option<(Op, bool)> {
    // (op, writes_rd)
    Some(match op {
        0b0000 => {
            if rd == 15 && s {
                (Op::Tst, false)
            } else {
                (Op::And, true)
            }
        }
        0b0001 => (Op::Bic, true),
        0b0010 => {
            if rn == 15 {
                (Op::Mov, true)
            } else {
                (Op::Orr, true)
            }
        }
        0b0011 => {
            if rn == 15 {
                (Op::Mvn, true)
            } else {
                (Op::Orn, true)
            }
        }
        0b0100 => {
            if rd == 15 && s {
                (Op::Teq, false)
            } else {
                (Op::Eor, true)
            }
        }
        0b1000 => {
            if rd == 15 && s {
                (Op::Cmn, false)
            } else {
                (Op::Add, true)
            }
        }
        0b1010 => (Op::Adc, true),
        0b1011 => (Op::Sbc, true),
        0b1101 => {
            if rd == 15 && s {
                (Op::Cmp, false)
            } else {
                (Op::Sub, true)
            }
        }
        0b1110 => (Op::Rsb, true),
        _ => return None,
    })
}

fn decode32(h1: u32, h2: u32) -> Inst {
    let undef = Inst::new(Op::Undefined, 4);
    let op1 = bits(h1, 12, 11);
    let op2 = bits(h1, 10, 4);
    let op = bit(h2, 15);
    match op1 {
        0b01 => {
            if op2 & 0b1100100 == 0b0000000 {
                // load/store multiple
                let opm = bits(h1, 8, 7);
                let load = bit(h1, 4) == 1;
                let w = bit(h1, 5) == 1;
                let rn = bits(h1, 3, 0);
                let mut i = Inst::new(if load { Op::Ldm } else { Op::Stm }, 4);
                i.rn = rn as u8;
                i.imm = h2 & if load { 0xDFFF } else { 0x5FFF };
                i.flags = if w { M_W } else { 0 } | if opm == 0b01 { M_U } else { 0 };
                if opm != 0b01 && opm != 0b10 {
                    return undef;
                }
                i.cyc = 1 + reglist_count(i.imm) + if load && i.imm & 0x8000 != 0 { 2 } else { 0 };
                return i;
            }
            if op2 & 0b1100100 == 0b0000100 {
                // load/store dual, exclusive, table branch
                let o1 = bits(h1, 8, 7);
                let o2 = bits(h1, 5, 4);
                let o3 = bits(h2, 7, 4);
                let rn = bits(h1, 3, 0) as u8;
                let rt = bits(h2, 15, 12) as u8;
                if o1 == 0b00 && o2 == 0b00 {
                    let mut i = Inst::new(Op::Strex, 4);
                    i.rn = rn;
                    i.rm = rt;
                    i.rd = bits(h2, 11, 8) as u8;
                    i.imm = (h2 & 0xFF) << 2;
                    i.sub = 2;
                    i.cyc = 2;
                    return i;
                }
                if o1 == 0b00 && o2 == 0b01 {
                    let mut i = Inst::new(Op::Ldrex, 4);
                    i.rn = rn;
                    i.rd = rt;
                    i.imm = (h2 & 0xFF) << 2;
                    i.sub = 2;
                    i.cyc = 2;
                    return i;
                }
                // LDRD / STRD: o1 = 0x with o2 = 1x, or o1 = 1x with any o2.
                if o1 & 2 != 0 || o2 & 2 != 0 {
                    let load = bit(h1, 4) == 1;
                    let mut i = Inst::new(if load { Op::Ldrd } else { Op::Strd }, 4);
                    i.rn = rn;
                    i.rd = rt;
                    i.ra = bits(h2, 11, 8) as u8;
                    i.imm = (h2 & 0xFF) << 2;
                    i.flags = if bit(h1, 8) == 1 { M_P } else { 0 }
                        | if bit(h1, 7) == 1 { M_U } else { 0 }
                        | if bit(h1, 5) == 1 { M_W } else { 0 };
                    if rn == 15 {
                        i.flags |= M_LIT;
                    }
                    i.cyc = 3;
                    return i;
                }
                if o1 == 0b01 && o2 == 0b00 {
                    let mut i = Inst::new(Op::Strex, 4);
                    i.rn = rn;
                    i.rm = rt;
                    i.rd = bits(h2, 3, 0) as u8;
                    i.imm = 0;
                    i.sub = if o3 == 0b0100 { 0 } else { 1 };
                    i.cyc = 2;
                    return i;
                }
                if o1 == 0b01 && o2 == 0b01 {
                    if o3 == 0b0000 || o3 == 0b0001 {
                        let mut i = Inst::new(Op::Tbb, 4);
                        i.rn = rn;
                        i.rm = bits(h2, 3, 0) as u8;
                        i.sub = o3 as u8; // 1 = TBH
                        i.cyc = 4;
                        return i;
                    }
                    let mut i = Inst::new(Op::Ldrex, 4);
                    i.rn = rn;
                    i.rd = rt;
                    i.imm = 0;
                    i.sub = if o3 == 0b0100 { 0 } else { 1 };
                    i.cyc = 2;
                    return i;
                }
                return undef;
            }
            if op2 & 0b1100000 == 0b0100000 {
                // data processing (shifted register)
                let opc = bits(h1, 8, 5);
                let s = bit(h1, 4) == 1;
                let rn = bits(h1, 3, 0);
                let rd = bits(h2, 11, 8);
                let rm = bits(h2, 3, 0) as u8;
                let imm5 = (bits(h2, 14, 12) << 2) | bits(h2, 7, 6);
                let t = bits(h2, 5, 4);
                if opc == 0b0110 {
                    let mut i = Inst::new(Op::Pkh, 4);
                    i.rd = rd as u8;
                    i.rn = rn as u8;
                    i.rm = rm;
                    i.sub = bit(h2, 5) as u8; // 1 = PKHTB (ASR)
                    i.sh_n = imm5 as u8;
                    return i;
                }
                let (o, _) = match dp_op(opc, rn, rd, s) {
                    Some(x) => x,
                    None => return undef,
                };
                let mut i = dp32(o, rd, rn, s);
                i.rm = rm;
                let (st, sn) = decode_imm_shift(t, imm5);
                i.sub = st;
                i.sh_n = sn;
                if t == 0 && imm5 == 0 {
                    i.sh_n = 0;
                }
                if rd == 15 && !matches!(o, Op::Tst | Op::Teq | Op::Cmp | Op::Cmn) {
                    i.cyc = 3;
                }
                return i;
            }
            decode_coproc(h1, h2)
        }
        0b10 => {
            if op == 0 {
                if op2 & 0b0100000 == 0 {
                    // data processing (modified immediate)
                    let opc = bits(h1, 8, 5);
                    let s = bit(h1, 4) == 1;
                    let rn = bits(h1, 3, 0);
                    let rd = bits(h2, 11, 8);
                    let imm12 = (bit(h1, 10) << 11) | (bits(h2, 14, 12) << 8) | (h2 & 0xFF);
                    let (o, _) = match dp_op(opc, rn, rd, s) {
                        Some(x) => x,
                        None => return undef,
                    };
                    let mut i = dp32(o, rd, rn, s);
                    let (v, c) = thumb_expand_imm(imm12);
                    i.imm = v;
                    i.flags |= F_IMM | if c { F_IMMC } else { 0 };
                    return i;
                }
                // data processing (plain binary immediate)
                let opc = bits(h1, 8, 4);
                let rn = bits(h1, 3, 0);
                let rd = bits(h2, 11, 8) as u8;
                let imm12 = (bit(h1, 10) << 11) | (bits(h2, 14, 12) << 8) | (h2 & 0xFF);
                let imm16 = (bits(h1, 3, 0) << 12) | imm12;
                let sh = (bits(h2, 14, 12) << 2) | bits(h2, 7, 6);
                let mut i = Inst::new(Op::Undefined, 4);
                i.rd = rd;
                i.rn = rn as u8;
                match opc {
                    0b00000 => {
                        if rn == 15 {
                            i.op = Op::Adr;
                            i.imm = imm12;
                        } else {
                            i.op = Op::AddW;
                            i.imm = imm12;
                        }
                    }
                    0b00100 => {
                        i.op = Op::Movw;
                        i.imm = imm16;
                    }
                    0b01010 => {
                        if rn == 15 {
                            i.op = Op::Adr;
                            i.imm = (imm12 as i32).wrapping_neg() as u32;
                        } else {
                            i.op = Op::SubW;
                            i.imm = imm12;
                        }
                    }
                    0b01100 => {
                        i.op = Op::Movt;
                        i.imm = imm16;
                    }
                    0b10000 | 0b10010 => {
                        if opc == 0b10010 && sh == 0 {
                            i.op = Op::Ssat16;
                            i.imm = bits(h2, 3, 0) + 1;
                        } else {
                            i.op = Op::Ssat;
                            i.imm = bits(h2, 4, 0) + 1;
                            i.sub = if bit(h1, 5) == 1 { SH_ASR } else { SH_LSL };
                            i.sh_n = sh as u8;
                        }
                    }
                    0b10100 => {
                        i.op = Op::Sbfx;
                        i.sh_n = sh as u8;
                        i.imm = bits(h2, 4, 0) + 1;
                    }
                    0b10110 => {
                        i.op = Op::Bfi; // rn == 15 is BFC
                        i.sh_n = sh as u8;
                        i.imm = bits(h2, 4, 0);
                    }
                    0b11000 | 0b11010 => {
                        if opc == 0b11010 && sh == 0 {
                            i.op = Op::Usat16;
                            i.imm = bits(h2, 3, 0);
                        } else {
                            i.op = Op::Usat;
                            i.imm = bits(h2, 4, 0);
                            i.sub = if bit(h1, 5) == 1 { SH_ASR } else { SH_LSL };
                            i.sh_n = sh as u8;
                        }
                    }
                    0b11100 => {
                        i.op = Op::Ubfx;
                        i.sh_n = sh as u8;
                        i.imm = bits(h2, 4, 0) + 1;
                    }
                    _ => {}
                }
                return i;
            }
            // branches and miscellaneous control
            let bop1 = bits(h2, 14, 12);
            let s = bit(h1, 10);
            let j1 = bit(h2, 13);
            let j2 = bit(h2, 11);
            if bop1 & 0b101 == 0b000 {
                if op2 & 0b0111000 != 0b0111000 {
                    let mut i = Inst::new(Op::B, 4);
                    i.cond = bits(h1, 9, 6) as u8;
                    let off = (s << 20) | (j2 << 19) | (j1 << 18) | (bits(h1, 5, 0) << 12) | ((h2 & 0x7FF) << 1);
                    i.imm = sext(off, 21);
                    return i;
                }
                match op2 {
                    0b0111000 | 0b0111001 => {
                        let mut i = Inst::new(Op::Msr, 4);
                        i.rn = bits(h1, 3, 0) as u8;
                        i.imm = h2 & 0xFF;
                        i.sub = bits(h2, 11, 10) as u8;
                        return i;
                    }
                    0b0111010 => {
                        return Inst::new(
                            match h2 & 0xFF {
                                2 => Op::Wfe,
                                3 => Op::Wfi,
                                4 => Op::Sev,
                                _ => Op::Nop,
                            },
                            4,
                        );
                    }
                    0b0111011 => {
                        let mut i = Inst::new(if bits(h2, 7, 4) == 0b0010 { Op::Clrex } else { Op::Barrier }, 4);
                        i.cyc = 2;
                        return i;
                    }
                    0b0111110 | 0b0111111 => {
                        let mut i = Inst::new(Op::Mrs, 4);
                        i.rd = bits(h2, 11, 8) as u8;
                        i.imm = h2 & 0xFF;
                        return i;
                    }
                    _ => return undef,
                }
            }
            if bop1 & 0b101 == 0b001 || bop1 & 0b101 == 0b101 {
                let i1 = 1 ^ (j1 ^ s);
                let i2 = 1 ^ (j2 ^ s);
                let off = (s << 24) | (i1 << 23) | (i2 << 22) | (bits(h1, 9, 0) << 12) | ((h2 & 0x7FF) << 1);
                let mut i = Inst::new(if bop1 & 0b100 != 0 { Op::Bl } else { Op::B }, 4);
                i.imm = sext(off, 25);
                return i;
            }
            undef
        }
        _ => {
            // op1 == 0b11
            if op2 & 0b1110001 == 0b0000000 || op2 & 0b1100001 == 0b0000001 && op2 & 0b0000110 != 0b0000110 {
                return decode_ldst_single(h1, h2);
            }
            if op2 & 0b1110000 == 0b0100000 {
                return decode_dp_reg(h1, h2);
            }
            if op2 & 0b1111000 == 0b0110000 {
                return decode_mul(h1, h2);
            }
            if op2 & 0b1111000 == 0b0111000 {
                return decode_long_mul(h1, h2);
            }
            if op2 & 0b1000000 != 0 {
                return decode_coproc(h1, h2);
            }
            undef
        }
    }
}

fn decode_ldst_single(h1: u32, h2: u32) -> Inst {
    let signed = bit(h1, 8) == 1;
    let pos12 = bit(h1, 7) == 1;
    let size = bits(h1, 6, 5);
    let load = bit(h1, 4) == 1;
    let rn = bits(h1, 3, 0) as u8;
    let rt = bits(h2, 15, 12) as u8;
    if size == 3 {
        return Inst::new(Op::Undefined, 4);
    }
    let mut i = Inst::new(if load { Op::Ldr } else { Op::Str }, 4);
    i.rd = rt;
    i.rn = rn;
    i.sub = size as u8 | if signed { 4 } else { 0 };
    i.cyc = 2;
    if load && rt == 15 && size != 2 {
        // PLD / PLI hints
        return Inst::new(Op::Nop, 4);
    }
    if rn == 15 && load {
        i.flags = M_P | M_LIT | if pos12 { M_U } else { 0 };
        i.imm = h2 & 0xFFF;
    } else if pos12 {
        i.flags = M_P | M_U;
        i.imm = h2 & 0xFFF;
    } else if bit(h2, 11) == 1 {
        i.flags = if bit(h2, 10) == 1 { M_P } else { 0 }
            | if bit(h2, 9) == 1 { M_U } else { 0 }
            | if bit(h2, 8) == 1 { M_W } else { 0 };
        i.imm = h2 & 0xFF;
    } else if bits(h2, 11, 6) == 0 {
        i.flags = M_P | M_U | M_REG;
        i.rm = bits(h2, 3, 0) as u8;
        i.sh_n = bits(h2, 5, 4) as u8;
    } else {
        return Inst::new(Op::Undefined, 4);
    }
    if load && rt == 15 {
        i.cyc = 4;
    }
    i
}

fn decode_dp_reg(h1: u32, h2: u32) -> Inst {
    let op1 = bits(h1, 7, 4);
    let op2 = bits(h2, 7, 4);
    let rn = bits(h1, 3, 0) as u8;
    let rd = bits(h2, 11, 8) as u8;
    let rm = bits(h2, 3, 0) as u8;
    if op2 == 0 && op1 & 0b1000 == 0 {
        // shift by register: rd = rn shifted by rm
        let mut i = dp32(Op::Mov, rd as u32, 0, bit(h1, 4) == 1);
        i.rm = rn;
        i.ra = rm;
        i.flags |= F_REGSH;
        i.sub = bits(h1, 6, 5) as u8;
        return i;
    }
    if op1 & 0b1000 == 0 && op2 & 0b1000 != 0 {
        let mut i = Inst::new(Op::Extend, 4);
        i.rd = rd;
        i.rn = rn;
        i.rm = rm;
        i.sh_n = (bits(h2, 5, 4) * 8) as u8;
        i.sub = match op1 {
            0b0000 => 0,
            0b0001 => 1,
            0b0010 => 2,
            0b0011 => 3,
            0b0100 => 4,
            0b0101 => 5,
            _ => return Inst::new(Op::Undefined, 4),
        };
        return i;
    }
    if op1 & 0b1000 != 0 && op2 & 0b1000 == 0 {
        // parallel add/sub: sub = (unsigned<<5) | (kind<<3) | op1[2:0]; kind 0 plain, 1 saturating, 2 halving
        let unsigned = op2 & 0b0100 != 0;
        let kind = op2 & 0b0011;
        if kind == 3 {
            return Inst::new(Op::Undefined, 4);
        }
        let mut i = Inst::new(Op::Parallel, 4);
        i.rd = rd;
        i.rn = rn;
        i.rm = rm;
        i.sub = ((unsigned as u8) << 5) | ((kind as u8) << 3) | (op1 & 7) as u8;
        return i;
    }
    if op1 & 0b1100 == 0b1000 && op2 & 0b1100 == 0b1000 {
        let o1 = op1 & 3;
        let o2 = op2 & 3;
        let op = match (o1, o2) {
            (0, 0) => Op::Qadd,
            (0, 1) => Op::Qdadd,
            (0, 2) => Op::Qsub,
            (0, 3) => Op::Qdsub,
            (1, 0) => Op::Rev,
            (1, 1) => Op::Rev16,
            (1, 2) => Op::Rbit,
            (1, 3) => Op::Revsh,
            (2, 0) => Op::Sel,
            (3, 0) => Op::Clz,
            _ => return Inst::new(Op::Undefined, 4),
        };
        let mut i = Inst::new(op, 4);
        i.rd = rd;
        i.rn = rn;
        i.rm = rm;
        if matches!(op, Op::Rev | Op::Rev16 | Op::Rbit | Op::Revsh | Op::Clz) {
            i.rm = rm;
        }
        return i;
    }
    Inst::new(Op::Undefined, 4)
}

fn decode_mul(h1: u32, h2: u32) -> Inst {
    let op1 = bits(h1, 6, 4);
    let op2 = bits(h2, 5, 4);
    let rn = bits(h1, 3, 0) as u8;
    let ra = bits(h2, 15, 12) as u8;
    let rd = bits(h2, 11, 8) as u8;
    let rm = bits(h2, 3, 0) as u8;
    let mut i = Inst::new(Op::Undefined, 4);
    i.rn = rn;
    i.ra = ra;
    i.rd = rd;
    i.rm = rm;
    match op1 {
        0b000 => {
            i.op = match op2 {
                0 => {
                    if ra == 15 {
                        Op::Mul
                    } else {
                        Op::Mla
                    }
                }
                1 => Op::Mls,
                _ => Op::Undefined,
            };
        }
        0b001 => {
            i.op = Op::Smlaxy;
            i.sub = op2 as u8; // N = bit1, M = bit0
        }
        0b010 => {
            i.op = Op::Smlad;
            i.sub = bit(h2, 4) as u8;
        }
        0b011 => {
            i.op = Op::Smlawy;
            i.sub = bit(h2, 4) as u8;
        }
        0b100 => {
            i.op = Op::Smlsd;
            i.sub = bit(h2, 4) as u8;
        }
        0b101 => {
            i.op = Op::Smmla;
            i.sub = bit(h2, 4) as u8;
        }
        0b110 => {
            i.op = Op::Smmls;
            i.sub = bit(h2, 4) as u8;
        }
        _ => {
            i.op = if op2 == 0 { Op::Usada8 } else { Op::Undefined };
        }
    }
    i
}

fn decode_long_mul(h1: u32, h2: u32) -> Inst {
    let op1 = bits(h1, 6, 4);
    let op2 = bits(h2, 7, 4);
    let mut i = Inst::new(Op::Undefined, 4);
    i.rn = bits(h1, 3, 0) as u8;
    i.ra = bits(h2, 15, 12) as u8; // RdLo
    i.rd = bits(h2, 11, 8) as u8; // RdHi (or Rd for divides)
    i.rm = bits(h2, 3, 0) as u8;
    i.op = match (op1, op2) {
        (0b000, 0b0000) => Op::Smull,
        (0b001, 0b1111) => Op::Sdiv,
        (0b010, 0b0000) => Op::Umull,
        (0b011, 0b1111) => Op::Udiv,
        (0b100, 0b0000) => Op::Smlal,
        (0b100, 0b1000..=0b1011) => {
            i.sub = (op2 & 3) as u8;
            Op::Smlalxy
        }
        (0b100, 0b1100 | 0b1101) => {
            i.sub = (op2 & 1) as u8;
            Op::Smlald
        }
        (0b101, 0b1100 | 0b1101) => {
            i.sub = (op2 & 1) as u8;
            Op::Smlsld
        }
        (0b110, 0b0000) => Op::Umlal,
        (0b110, 0b0110) => Op::Umaal,
        _ => Op::Undefined,
    };
    i
}

fn decode_coproc(h1: u32, h2: u32) -> Inst {
    let undef = Inst::new(Op::Undefined, 4);
    let coproc = bits(h2, 11, 8);
    if coproc & 0b1110 != 0b1010 {
        return undef;
    }
    let dbl = coproc == 0b1011;
    let top = bits(h1, 15, 8);
    if top == 0xEC || top == 0xED {
        let opc = bits(h1, 8, 4);
        let rn = bits(h1, 3, 0) as u8;
        let vd = bits(h2, 15, 12);
        let d = bit(h1, 6);
        // first S register index (D registers are pairs of S registers)
        let sreg = if dbl { (d << 4 | vd) * 2 } else { vd << 1 | d };
        let imm8 = h2 & 0xFF;
        if opc & 0b11110 == 0b00100 {
            // 64-bit transfer between two core registers and S pair / D register
            let to_core = bit(h1, 4) == 1;
            let vm = bits(h2, 3, 0);
            let m = bit(h2, 5);
            let mut i = Inst::new(if dbl { Op::VmovRRD } else { Op::VmovRRS }, 4);
            i.rd = bits(h2, 15, 12) as u8; // Rt
            i.ra = bits(h1, 3, 0) as u8; // Rt2
            i.rm = if dbl { ((m << 4 | vm) * 2) as u8 } else { (vm << 1 | m) as u8 };
            i.sub = to_core as u8;
            i.cyc = 2;
            return i;
        }
        let p = bit(h1, 8);
        let u = bit(h1, 7);
        let w = bit(h1, 5);
        let l = bit(h1, 4);
        if p == 1 && w == 0 {
            // VLDR / VSTR
            let mut i = Inst::new(if l == 1 { Op::Vldr } else { Op::Vstr }, 4);
            i.rn = rn;
            i.rd = sreg as u8;
            i.imm = imm8 << 2;
            i.flags = if u == 1 { M_U } else { 0 } | if rn == 15 { M_LIT } else { 0 };
            i.sub = dbl as u8;
            i.cyc = if dbl { 3 } else { 2 };
            return i;
        }
        // VLDM / VSTM (incl. VPUSH / VPOP)
        let nregs = imm8; // 32-bit words for both single and double registers
        let mut i = Inst::new(if l == 1 { Op::Vldm } else { Op::Vstm }, 4);
        i.rn = rn;
        i.rd = sreg as u8;
        i.imm = nregs; // number of 32-bit words
        i.flags = if u == 1 { M_U } else { 0 } | if w == 1 { M_W } else { 0 };
        i.cyc = 1 + nregs as u8;
        return i;
    }
    if top == 0xEE {
        if bit(h2, 4) == 0 {
            return decode_vfp_dp(h1, h2, dbl);
        }
        // register transfers
        let l = bit(h1, 4) == 1;
        let a = bits(h1, 7, 5);
        let rt = bits(h2, 15, 12) as u8;
        if dbl {
            return undef;
        }
        if a == 0b000 {
            let mut i = Inst::new(Op::VmovSR, 4);
            i.rd = rt;
            i.rm = ((bits(h1, 3, 0) << 1) | bit(h2, 7)) as u8;
            i.sub = l as u8; // 1 = to core
            return i;
        }
        if a == 0b111 {
            let mut i = Inst::new(if l { Op::Vmrs } else { Op::Vmsr }, 4);
            i.rd = rt;
            i.imm = bits(h1, 3, 0); // reg (1 = FPSCR)
            return i;
        }
        return undef;
    }
    undef
}

fn decode_vfp_dp(h1: u32, h2: u32, dbl: bool) -> Inst {
    let undef = Inst::new(Op::Undefined, 4);
    if dbl {
        // The M4 FPU is single precision; double data processing is undefined.
        return undef;
    }
    let opc1 = (bit(h1, 7) << 2) | bits(h1, 5, 4);
    let opc2 = bits(h1, 3, 0);
    let opc3 = bits(h2, 7, 6);
    let vd = (bits(h2, 15, 12) << 1) | bit(h1, 6);
    let vn = (bits(h1, 3, 0) << 1) | bit(h2, 7);
    let vm = (bits(h2, 3, 0) << 1) | bit(h2, 5);
    let opb = bit(h2, 6);
    let mut i = Inst::new(Op::Vfp, 4);
    i.rd = vd as u8;
    i.rn = vn as u8;
    i.rm = vm as u8;
    i.sub = match opc1 {
        0b000 => {
            i.cyc = 3;
            if opb == 0 {
                vfp::VMLA
            } else {
                vfp::VMLS
            }
        }
        0b001 => {
            i.cyc = 3;
            if opb == 0 {
                vfp::VNMLS
            } else {
                vfp::VNMLA
            }
        }
        0b010 => {
            if opb == 0 {
                vfp::VMUL
            } else {
                vfp::VNMUL
            }
        }
        0b011 => {
            if opb == 0 {
                vfp::VADD
            } else {
                vfp::VSUB
            }
        }
        0b100 => {
            i.cyc = 14;
            vfp::VDIV
        }
        0b101 => {
            i.cyc = 3;
            if opb == 0 {
                vfp::VFNMS
            } else {
                vfp::VFNMA
            }
        }
        0b110 => {
            i.cyc = 3;
            if opb == 0 {
                vfp::VFMA
            } else {
                vfp::VFMS
            }
        }
        _ => {
            if opc3 & 1 == 0 {
                // VMOV immediate: imm8 = imm4H:imm4L
                let imm8 = (bits(h1, 3, 0) << 4) | bits(h2, 3, 0);
                let b6 = bit(imm8, 6);
                let exp = ((1 ^ b6) << 7) | (if b6 == 1 { 0x7C } else { 0 }) | bits(imm8, 5, 4);
                let val = (bit(imm8, 7) << 31) | (exp << 23) | (bits(imm8, 3, 0) << 19);
                i.imm = val;
                vfp::VMOVI
            } else {
                match (opc2, opc3) {
                    (0b0000, 0b01) => vfp::VMOVR,
                    (0b0000, 0b11) => vfp::VABS,
                    (0b0001, 0b01) => vfp::VNEG,
                    (0b0001, 0b11) => {
                        i.cyc = 14;
                        vfp::VSQRT
                    }
                    (0b0010 | 0b0011, _) => {
                        i.imm = (opc2 & 1) | (bit(h2, 7) << 1);
                        vfp::VCVTBT
                    }
                    (0b0100, _) => {
                        if bit(h2, 7) == 0 {
                            vfp::VCMP
                        } else {
                            vfp::VCMPE
                        }
                    }
                    (0b0101, _) => {
                        if bit(h2, 7) == 0 {
                            vfp::VCMP0
                        } else {
                            vfp::VCMPE0
                        }
                    }
                    (0b1000, _) => {
                        if bit(h2, 7) == 1 {
                            vfp::VCVT_F_S
                        } else {
                            vfp::VCVT_F_U
                        }
                    }
                    (0b1100, _) => {
                        i.imm = 1 - bit(h2, 7); // 1 = use FPSCR rounding (VCVTR), 0 = toward zero
                        vfp::VCVT_U_F
                    }
                    (0b1101, _) => {
                        i.imm = 1 - bit(h2, 7);
                        vfp::VCVT_S_F
                    }
                    (0b1010 | 0b1011 | 0b1110 | 0b1111, _) => {
                        // fixed-point: imm = to_fixed<<8 | unsigned<<7 | sx<<6 | frac bits
                        let sx = bit(h2, 7);
                        let imm5 = (bits(h2, 3, 0) << 1) | bit(h2, 5);
                        let size = if sx == 1 { 32 } else { 16 };
                        let frac = size - imm5;
                        i.imm = (bit(opc2, 2) << 8) | (bit(opc2, 0) << 7) | (sx << 6) | frac;
                        vfp::VCVT_FIX
                    }
                    _ => return undef,
                }
            }
        }
    };
    i
}
