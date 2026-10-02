//! Basic chess types. Squares are numbered a1 = 0 .. h8 = 63 (little-endian rank-file).
//! Pieces are encoded as `color << 3 | type`, so the type and colour each cost one
//! instruction to extract on a Cortex-M4.

pub type Bitboard = u64;
pub type Value = i32;

pub const WHITE: usize = 0;
pub const BLACK: usize = 1;

pub const PAWN: usize = 0;
pub const KNIGHT: usize = 1;
pub const BISHOP: usize = 2;
pub const ROOK: usize = 3;
pub const QUEEN: usize = 4;
pub const KING: usize = 5;

pub const NO_PIECE: u8 = 15;
pub const NO_SQ: u8 = 64;

pub const VALUE_ZERO: Value = 0;
pub const VALUE_DRAW: Value = 0;
pub const VALUE_MATE: Value = 32000;
pub const VALUE_INFINITE: Value = 32001;
pub const VALUE_NONE: Value = 32002;
pub const MAX_PLY: usize = 64;
pub const VALUE_MATE_IN_MAX_PLY: Value = VALUE_MATE - MAX_PLY as Value;
pub const VALUE_MATED_IN_MAX_PLY: Value = -VALUE_MATE_IN_MAX_PLY;

#[inline(always)]
pub const fn mate_in(ply: usize) -> Value {
    VALUE_MATE - ply as Value
}
#[inline(always)]
pub const fn mated_in(ply: usize) -> Value {
    -VALUE_MATE + ply as Value
}
#[inline(always)]
pub const fn is_decisive(v: Value) -> bool {
    v >= VALUE_MATE_IN_MAX_PLY || v <= VALUE_MATED_IN_MAX_PLY
}

#[inline(always)]
pub const fn make_piece(color: usize, pt: usize) -> u8 {
    ((color << 3) | pt) as u8
}
#[inline(always)]
pub const fn ptype(p: u8) -> usize {
    (p & 7) as usize
}
#[inline(always)]
pub const fn pcolor(p: u8) -> usize {
    (p >> 3) as usize
}

#[inline(always)]
pub const fn file_of(sq: usize) -> usize {
    sq & 7
}
#[inline(always)]
pub const fn rank_of(sq: usize) -> usize {
    sq >> 3
}
#[inline(always)]
pub const fn relative_rank(c: usize, sq: usize) -> usize {
    (sq >> 3) ^ (c * 7)
}
#[inline(always)]
pub const fn relative_sq(c: usize, sq: usize) -> usize {
    sq ^ (c * 56)
}

#[inline(always)]
pub const fn bb(sq: usize) -> Bitboard {
    1u64 << sq
}

#[inline(always)]
pub fn lsb(b: Bitboard) -> usize {
    b.trailing_zeros() as usize
}
#[inline(always)]
pub fn msb(b: Bitboard) -> usize {
    63 - b.leading_zeros() as usize
}
#[inline(always)]
pub fn pop_lsb(b: &mut Bitboard) -> usize {
    let s = lsb(*b);
    *b &= b.wrapping_sub(1);
    s
}
#[inline(always)]
pub fn popcount(b: Bitboard) -> u32 {
    // SWAR on both 32-bit halves, merged once the counts are per nibble (at most 8 per
    // nibble after the merge), so the last two steps run once: the M4 has no POPCNT.
    let (mut lo, mut hi) = (b as u32, (b >> 32) as u32);
    lo -= (lo >> 1) & 0x5555_5555;
    hi -= (hi >> 1) & 0x5555_5555;
    lo = (lo & 0x3333_3333) + ((lo >> 2) & 0x3333_3333);
    hi = (hi & 0x3333_3333) + ((hi >> 2) & 0x3333_3333);
    let x = lo + hi;
    let x = (x & 0x0F0F_0F0F) + ((x >> 4) & 0x0F0F_0F0F);
    x.wrapping_mul(0x0101_0101) >> 24
}
#[inline(always)]
pub fn more_than_one(b: Bitboard) -> bool {
    b & b.wrapping_sub(1) != 0
}

pub const FILE_A: Bitboard = 0x0101_0101_0101_0101;
pub const FILE_H: Bitboard = FILE_A << 7;
pub const RANK_1: Bitboard = 0xFF;
pub const RANK_2: Bitboard = RANK_1 << 8;
pub const RANK_3: Bitboard = RANK_1 << 16;
pub const RANK_4: Bitboard = RANK_1 << 24;
pub const RANK_5: Bitboard = RANK_1 << 32;
pub const RANK_6: Bitboard = RANK_1 << 40;
pub const RANK_7: Bitboard = RANK_1 << 48;
pub const RANK_8: Bitboard = RANK_1 << 56;
pub const LIGHT_SQUARES: Bitboard = 0x55AA_55AA_55AA_55AA;
pub const DARK_SQUARES: Bitboard = !LIGHT_SQUARES;

#[inline(always)]
pub const fn file_bb(sq: usize) -> Bitboard {
    FILE_A << (sq & 7)
}
#[inline(always)]
pub const fn rank_bb(sq: usize) -> Bitboard {
    RANK_1 << (sq & 56)
}

/// Shift a bitboard one step "forward" for colour `c`.
#[inline(always)]
pub const fn push(b: Bitboard, c: usize) -> Bitboard {
    if c == WHITE {
        b << 8
    } else {
        b >> 8
    }
}

#[inline(always)]
pub const fn pawn_attacks_bb(b: Bitboard, c: usize) -> Bitboard {
    if c == WHITE {
        ((b & !FILE_A) << 7) | ((b & !FILE_H) << 9)
    } else {
        ((b & !FILE_A) >> 9) | ((b & !FILE_H) >> 7)
    }
}

/// A move packed in 16 bits: from (0-5), to (6-11), promotion piece - knight (12-13),
/// kind (14-15). Castling is encoded king-from to king-destination.
#[derive(Clone, Copy, PartialEq, Eq, Default, Debug)]
#[repr(transparent)]
pub struct Move(pub u16);

pub const MK_NORMAL: u16 = 0;
pub const MK_PROMOTION: u16 = 1 << 14;
pub const MK_EN_PASSANT: u16 = 2 << 14;
pub const MK_CASTLING: u16 = 3 << 14;

impl Move {
    pub const NONE: Move = Move(0);
    pub const NULL: Move = Move(65);

    #[inline(always)]
    pub const fn new(from: usize, to: usize) -> Move {
        Move((from | (to << 6)) as u16)
    }
    #[inline(always)]
    pub const fn with_kind(kind: u16, from: usize, to: usize) -> Move {
        Move(kind | (from | (to << 6)) as u16)
    }
    #[inline(always)]
    pub const fn promotion(from: usize, to: usize, pt: usize) -> Move {
        Move(MK_PROMOTION | (((pt - KNIGHT) << 12) | from | (to << 6)) as u16)
    }
    #[inline(always)]
    pub const fn from(self) -> usize {
        (self.0 & 63) as usize
    }
    #[inline(always)]
    pub const fn to(self) -> usize {
        ((self.0 >> 6) & 63) as usize
    }
    #[inline(always)]
    pub const fn from_to(self) -> usize {
        (self.0 & 0xFFF) as usize
    }
    #[inline(always)]
    pub const fn kind(self) -> u16 {
        self.0 & (3 << 14)
    }
    #[inline(always)]
    pub const fn promo_type(self) -> usize {
        (((self.0 >> 12) & 3) as usize) + KNIGHT
    }
    #[inline(always)]
    pub const fn is_promotion(self) -> bool {
        self.kind() == MK_PROMOTION
    }
    #[inline(always)]
    pub const fn is_some(self) -> bool {
        self.0 != 0
    }
    #[inline(always)]
    pub const fn is_none(self) -> bool {
        self.0 == 0
    }
    #[inline(always)]
    pub const fn is_ok(self) -> bool {
        self.0 != 0 && self.0 != 65
    }

    /// Long algebraic notation into `buf`; returns the length (4 or 5).
    pub fn write_uci(self, buf: &mut [u8; 5]) -> usize {
        if self.is_none() {
            buf[..4].copy_from_slice(b"0000");
            return 4;
        }
        let (f, t) = (self.from(), self.to());
        buf[0] = b'a' + file_of(f) as u8;
        buf[1] = b'1' + rank_of(f) as u8;
        buf[2] = b'a' + file_of(t) as u8;
        buf[3] = b'1' + rank_of(t) as u8;
        if self.is_promotion() {
            buf[4] = b"nbrq"[self.promo_type() - KNIGHT];
            5
        } else {
            4
        }
    }
}

impl core::fmt::Display for Move {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut b = [0u8; 5];
        let n = self.write_uci(&mut b);
        f.write_str(core::str::from_utf8(&b[..n]).unwrap_or("????"))
    }
}

/// Places a static in RAM on the micro:bit (the default `.rodata` lives in flash,
/// which costs two or three wait states per data load on the nRF52833).
#[macro_export]
macro_rules! ram_static {
    ($(#[$m:meta])* $vis:vis static $name:ident : $ty:ty = $init:expr;) => {
        $(#[$m])*
        #[cfg_attr(target_os = "none", link_section = ".data.tables")]
        $vis static $name: $ty = $init;
    };
}
