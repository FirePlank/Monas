//! Pseudo-legal move generation, split into noisy moves (captures and queen promotions)
//! and quiet moves (everything else, under-promotions included). When in check only
//! evasions are generated; `Position::is_legal` filters pins and king safety.

use crate::attacks::*;
use crate::position::Position;
use crate::types::*;

pub const GEN_NOISY: u8 = 1;
pub const GEN_QUIET: u8 = 2;
pub const GEN_ALL: u8 = 3;

pub trait Sink {
    fn push(&mut self, m: Move);
}

pub struct MoveList {
    pub moves: [Move; 256],
    pub len: usize,
}

impl Default for MoveList {
    fn default() -> Self {
        Self::new()
    }
}

impl MoveList {
    #[inline]
    pub const fn new() -> MoveList {
        MoveList { moves: [Move::NONE; 256], len: 0 }
    }
    #[inline]
    pub fn as_slice(&self) -> &[Move] {
        &self.moves[..self.len]
    }
}

impl Sink for MoveList {
    #[inline(always)]
    fn push(&mut self, m: Move) {
        self.moves[self.len] = m;
        self.len += 1;
    }
}

#[inline(always)]
fn serialize<S: Sink>(from: usize, mut targets: Bitboard, out: &mut S) {
    while targets != 0 {
        let to = pop_lsb(&mut targets);
        out.push(Move::new(from, to));
    }
}

#[inline(always)]
fn push_promos<S: Sink, const MODE: u8>(from: usize, to: usize, out: &mut S) {
    if MODE & GEN_NOISY != 0 {
        out.push(Move::promotion(from, to, QUEEN));
    }
    if MODE & GEN_QUIET != 0 {
        out.push(Move::promotion(from, to, KNIGHT));
        out.push(Move::promotion(from, to, ROOK));
        out.push(Move::promotion(from, to, BISHOP));
    }
}

#[cfg_attr(target_os = "none", link_section = ".ramtext")]
#[inline(never)]
pub fn generate<const MODE: u8, S: Sink>(pos: &Position, out: &mut S) {
    let us = pos.us();
    let them = us ^ 1;
    let occ = pos.occ();
    let enemy = pos.colors[them];
    let empty = !occ;
    let ksq = pos.king_sq(us);
    let noisy = MODE & GEN_NOISY != 0;
    let quiet = MODE & GEN_QUIET != 0;

    let katt = king_attacks(ksq);
    if noisy {
        serialize(ksq, katt & enemy, out);
    }
    if quiet {
        serialize(ksq, katt & empty, out);
    }
    if more_than_one(pos.checkers) {
        return;
    }
    let (cap_t, quiet_t) =
        if pos.checkers != 0 { (pos.checkers, between(ksq, lsb(pos.checkers))) } else { (enemy, empty) };

    // Pawns
    let pawns = pos.pc(us, PAWN);
    let (rank7, rank3) = if us == WHITE { (RANK_7, RANK_3) } else { (RANK_2, RANK_6) };
    let promo = pawns & rank7;
    let norm = pawns & !rank7;
    let up: isize = if us == WHITE { 8 } else { -8 };
    // Capture toward the a-file and toward the h-file, and the from-square offsets.
    let (left_off, right_off): (isize, isize) = if us == WHITE { (7, 9) } else { (-9, -7) };

    if quiet {
        let b1 = push(norm, us) & empty;
        let b2 = push(b1 & rank3, us) & empty & quiet_t;
        let mut b1 = b1 & quiet_t;
        while b1 != 0 {
            let to = pop_lsb(&mut b1);
            out.push(Move::new((to as isize - up) as usize, to));
        }
        let mut b2 = b2;
        while b2 != 0 {
            let to = pop_lsb(&mut b2);
            out.push(Move::new((to as isize - 2 * up) as usize, to));
        }
    }
    if noisy {
        let (l, r) = shifted_captures(norm, us);
        let mut l = l & enemy & cap_t;
        let mut r = r & enemy & cap_t;
        while l != 0 {
            let to = pop_lsb(&mut l);
            out.push(Move::new((to as isize - left_off) as usize, to));
        }
        while r != 0 {
            let to = pop_lsb(&mut r);
            out.push(Move::new((to as isize - right_off) as usize, to));
        }
        if pos.ep != NO_SQ {
            let ep = pos.ep as usize;
            if pos.checkers == 0 || pos.checkers & bb(ep ^ 8) != 0 || quiet_t & bb(ep) != 0 {
                let mut att = pawn_attacks(them, ep) & norm;
                while att != 0 {
                    let from = pop_lsb(&mut att);
                    out.push(Move::with_kind(MK_EN_PASSANT, from, ep));
                }
            }
        }
    }
    if promo != 0 {
        let mut p = push(promo, us) & empty & quiet_t;
        while p != 0 {
            let to = pop_lsb(&mut p);
            push_promos::<S, MODE>((to as isize - up) as usize, to, out);
        }
        let (l, r) = shifted_captures(promo, us);
        let mut l = l & enemy & cap_t;
        let mut r = r & enemy & cap_t;
        while l != 0 {
            let to = pop_lsb(&mut l);
            push_promos::<S, MODE>((to as isize - left_off) as usize, to, out);
        }
        while r != 0 {
            let to = pop_lsb(&mut r);
            push_promos::<S, MODE>((to as isize - right_off) as usize, to, out);
        }
    }

    // Pieces
    let targets_n = if noisy { cap_t } else { 0 } | if quiet { quiet_t } else { 0 };
    let mut b = pos.pc(us, KNIGHT);
    while b != 0 {
        let from = pop_lsb(&mut b);
        serialize(from, knight_attacks(from) & targets_n, out);
    }
    let mut b = pos.pc(us, BISHOP) | pos.pc(us, QUEEN);
    while b != 0 {
        let from = pop_lsb(&mut b);
        serialize(from, bishop_attacks(from, occ) & targets_n, out);
    }
    let mut b = pos.pc(us, ROOK) | pos.pc(us, QUEEN);
    while b != 0 {
        let from = pop_lsb(&mut b);
        serialize(from, rook_attacks(from, occ) & targets_n, out);
    }

    // Castling
    if quiet && pos.checkers == 0 {
        let rights = pos.castling >> (2 * us);
        if rights & 3 != 0 {
            let base = relative_sq(us, 0);
            if rights & 1 != 0 && pos.castling_ok(base + 6) {
                out.push(Move::with_kind(MK_CASTLING, base + 4, base + 6));
            }
            if rights & 2 != 0 && pos.castling_ok(base + 2) {
                out.push(Move::with_kind(MK_CASTLING, base + 4, base + 2));
            }
        }
    }
}

/// Pawn captures toward the a-file and toward the h-file.
#[inline(always)]
fn shifted_captures(p: Bitboard, us: usize) -> (Bitboard, Bitboard) {
    if us == WHITE {
        ((p & !FILE_A) << 7, (p & !FILE_H) << 9)
    } else {
        ((p & !FILE_A) >> 9, (p & !FILE_H) >> 7)
    }
}

/// Writes moves as `u32` into a slice: the one sink type the generators are compiled for,
/// so the firmware keeps a single copy of each generator in RAM.
pub struct SliceSink<'a> {
    pub buf: &'a mut [u32],
    pub end: usize,
}

impl Sink for SliceSink<'_> {
    #[inline(always)]
    fn push(&mut self, m: Move) {
        self.buf[self.end] = m.0 as u32;
        self.end += 1;
    }
}

/// All pseudo-legal moves (noisy then quiet).
pub fn generate_all(pos: &Position, list: &mut MoveList) {
    let mut buf = [0u32; 256];
    let mut sink = SliceSink { buf: &mut buf, end: 0 };
    generate::<GEN_NOISY, _>(pos, &mut sink);
    generate::<GEN_QUIET, _>(pos, &mut sink);
    let n = sink.end;
    for &m in &buf[..n] {
        list.push(Move(m as u16));
    }
}

pub fn generate_legal(pos: &Position, list: &mut MoveList) {
    let mut all = MoveList::new();
    generate_all(pos, &mut all);
    let pinned = pos.pinned(pos.us());
    list.len = 0;
    for &m in all.as_slice() {
        if pos.is_legal(m, pinned) {
            list.push(m);
        }
    }
}

pub fn perft(pos: &Position, depth: u32) -> u64 {
    let mut list = MoveList::new();
    generate_all(pos, &mut list);
    let pinned = pos.pinned(pos.us());
    let mut n = 0;
    for &m in list.as_slice() {
        if !pos.is_legal(m, pinned) {
            continue;
        }
        if depth <= 1 {
            n += 1;
        } else {
            let mut c = *pos;
            c.do_move(m);
            n += perft(&c, depth - 1);
        }
    }
    n
}
