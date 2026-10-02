//! Attack generation. Leapers come from small tables; sliders use Hyperbola Quintessence
//! for files and diagonals (a byte swap is two REV instructions on the M4) and a 512-byte
//! first-rank table for ranks. Everything here is a few kilobytes so it can live in RAM,
//! where the M4 loads it with no wait states, instead of a 800 KB magic table in flash.

use crate::ram_static;
use crate::types::*;

const fn leaper_table(d: &[(i32, i32); 8]) -> [Bitboard; 64] {
    let mut t = [0u64; 64];
    let mut sq = 0;
    while sq < 64 {
        let r = (sq / 8) as i32;
        let f = (sq % 8) as i32;
        let mut b = 0u64;
        let mut i = 0;
        while i < 8 {
            let nr = r + d[i].0;
            let nf = f + d[i].1;
            if nr >= 0 && nr < 8 && nf >= 0 && nf < 8 {
                b |= 1u64 << (nr * 8 + nf);
            }
            i += 1;
        }
        t[sq] = b;
        sq += 1;
    }
    t
}

const KNIGHT_D: [(i32, i32); 8] = [(1, 2), (2, 1), (2, -1), (1, -2), (-1, -2), (-2, -1), (-2, 1), (-1, 2)];
const KING_D: [(i32, i32); 8] = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)];

const fn pawn_table() -> [[Bitboard; 64]; 2] {
    let mut t = [[0u64; 64]; 2];
    let mut sq = 0;
    while sq < 64 {
        let b = 1u64 << sq;
        t[0][sq] = pawn_attacks_bb(b, WHITE);
        t[1][sq] = pawn_attacks_bb(b, BLACK);
        sq += 1;
    }
    t
}

const fn ray(sq: usize, dr: i32, df: i32) -> Bitboard {
    let mut r = (sq / 8) as i32 + dr;
    let mut f = (sq % 8) as i32 + df;
    let mut b = 0u64;
    while r >= 0 && r < 8 && f >= 0 && f < 8 {
        b |= 1u64 << (r * 8 + f);
        r += dr;
        f += df;
    }
    b
}

/// Per-square line masks (excluding the square itself), packed together so one
/// square's data is contiguous.
#[derive(Clone, Copy)]
#[repr(C)]
pub struct Lines {
    pub file: Bitboard,
    pub diag: Bitboard,
    pub anti: Bitboard,
    pub bit: Bitboard,
}

const fn lines_table() -> [Lines; 64] {
    let mut t = [Lines { file: 0, diag: 0, anti: 0, bit: 0 }; 64];
    let mut sq = 0;
    while sq < 64 {
        t[sq] = Lines {
            file: ray(sq, 1, 0) | ray(sq, -1, 0),
            diag: ray(sq, 1, 1) | ray(sq, -1, -1),
            anti: ray(sq, 1, -1) | ray(sq, -1, 1),
            bit: 1u64 << sq,
        };
        sq += 1;
    }
    t
}

/// First-rank attacks, indexed by `file * 64 + inner occupancy (b..g files)`.
const fn rank_table() -> [u8; 512] {
    let mut t = [0u8; 512];
    let mut f = 0;
    while f < 8 {
        let mut occ6 = 0;
        while occ6 < 64 {
            let occ = (occ6 << 1) as u32;
            let mut att = 0u32;
            let mut x = f as i32 + 1;
            while x < 8 {
                att |= 1 << x;
                if occ & (1 << x) != 0 {
                    break;
                }
                x += 1;
            }
            x = f as i32 - 1;
            while x >= 0 {
                att |= 1 << x;
                if occ & (1 << x) != 0 {
                    break;
                }
                x -= 1;
            }
            t[f * 64 + occ6] = att as u8;
            occ6 += 1;
        }
        f += 1;
    }
    t
}

ram_static! { pub static KNIGHT_ATT: [Bitboard; 64] = leaper_table(&KNIGHT_D); }
ram_static! { pub static KING_ATT: [Bitboard; 64] = leaper_table(&KING_D); }
ram_static! { pub static PAWN_ATT: [[Bitboard; 64]; 2] = pawn_table(); }
ram_static! { pub static LINES: [Lines; 64] = lines_table(); }
ram_static! { pub static RANK_ATT: [u8; 512] = rank_table(); }

#[inline(always)]
pub fn knight_attacks(sq: usize) -> Bitboard {
    KNIGHT_ATT[sq]
}
#[inline(always)]
pub fn king_attacks(sq: usize) -> Bitboard {
    KING_ATT[sq]
}
#[inline(always)]
pub fn pawn_attacks(c: usize, sq: usize) -> Bitboard {
    PAWN_ATT[c][sq]
}

/// Attacks along one line through the slider, which must hold at most one square
/// per rank (a file or a diagonal) so that the byte swap reverses it.
#[inline(always)]
fn hq(occ: Bitboard, mask: Bitboard, s: Bitboard) -> Bitboard {
    let fwd = occ & mask;
    let rev = fwd.swap_bytes();
    let fwd = fwd.wrapping_sub(s);
    let rev = rev.wrapping_sub(s.swap_bytes());
    (fwd ^ rev.swap_bytes()) & mask
}

#[inline(always)]
fn rank_attacks(sq: usize, occ: Bitboard) -> Bitboard {
    // The rank lies entirely in one 32-bit half, so work on that half only.
    let half = if sq < 32 { occ as u32 } else { (occ >> 32) as u32 };
    let sh = (sq & 24) as u32;
    let occ6 = ((half >> (sh + 1)) & 63) as usize;
    let a = (RANK_ATT[((sq & 7) << 6) | occ6] as u32) << sh;
    if sq < 32 {
        a as u64
    } else {
        (a as u64) << 32
    }
}

#[inline(always)]
pub fn bishop_attacks(sq: usize, occ: Bitboard) -> Bitboard {
    let l = &LINES[sq];
    hq(occ, l.diag, l.bit) | hq(occ, l.anti, l.bit)
}

#[inline(always)]
pub fn rook_attacks(sq: usize, occ: Bitboard) -> Bitboard {
    let l = &LINES[sq];
    hq(occ, l.file, l.bit) | rank_attacks(sq, occ)
}

#[inline(always)]
pub fn queen_attacks(sq: usize, occ: Bitboard) -> Bitboard {
    bishop_attacks(sq, occ) | rook_attacks(sq, occ)
}

/// Rook rays on an empty board.
#[inline(always)]
pub fn rook_pseudo(sq: usize) -> Bitboard {
    LINES[sq].file | (rank_bb(sq) ^ bb(sq))
}
/// Bishop rays on an empty board.
#[inline(always)]
pub fn bishop_pseudo(sq: usize) -> Bitboard {
    LINES[sq].diag | LINES[sq].anti
}

#[inline(always)]
pub fn attacks_of(pt: usize, sq: usize, occ: Bitboard) -> Bitboard {
    match pt {
        KNIGHT => knight_attacks(sq),
        BISHOP => bishop_attacks(sq, occ),
        ROOK => rook_attacks(sq, occ),
        QUEEN => queen_attacks(sq, occ),
        _ => king_attacks(sq),
    }
}

/// Squares strictly between `a` and `b` if they share a line, else empty.
#[cfg_attr(target_os = "none", link_section = ".ramtext")]
#[inline]
pub fn between(a: usize, b: usize) -> Bitboard {
    let (ra, fa, rb, fb) = (rank_of(a), file_of(a), rank_of(b), file_of(b));
    if ra == rb || fa == fb {
        rook_attacks(a, bb(b)) & rook_attacks(b, bb(a))
    } else if ra.wrapping_sub(fa) == rb.wrapping_sub(fb) || ra + fa == rb + fb {
        bishop_attacks(a, bb(b)) & bishop_attacks(b, bb(a))
    } else {
        0
    }
}

/// True if `c` lies on the line through `a` and `b` (which must share a line).
#[cfg_attr(target_os = "none", link_section = ".ramtext")]
#[inline]
pub fn aligned(a: usize, b: usize, c: usize) -> bool {
    let (ra, fa, rb, fb, rc, fc) = (rank_of(a), file_of(a), rank_of(b), file_of(b), rank_of(c), file_of(c));
    if ra == rb {
        rc == ra
    } else if fa == fb {
        fc == fa
    } else if ra.wrapping_sub(fa) == rb.wrapping_sub(fb) {
        rc.wrapping_sub(fc) == ra.wrapping_sub(fa)
    } else {
        rc + fc == ra + fa
    }
}

#[inline(always)]
pub fn distance(a: usize, b: usize) -> usize {
    let dr = (rank_of(a) as i32 - rank_of(b) as i32).unsigned_abs();
    let df = (file_of(a) as i32 - file_of(b) as i32).unsigned_abs();
    dr.max(df) as usize
}
