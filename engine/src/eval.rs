//! Hand-crafted evaluation: incrementally kept material + PeSTO tables, plus mobility,
//! pawn structure, king shelter and attacker pressure, rook files, outposts and the
//! bishop pair. Port of Peras' HCE onto packed scores. KX vs K positions use the
//! mop-up evaluation instead.

use crate::attacks::*;
use crate::mopup::kx_vs_k;
use crate::position::Position;
use crate::psqt::{eg, mg, s, Score, MAX_PHASE};
use crate::types::*;

#[rustfmt::skip]
const KNIGHT_MOB: [Score; 9] = [
    s(-62, -81), s(-36, -46), s(-12, -26), s(0, -8), s(8, 4), s(14, 10), s(18, 14), s(20, 16), s(22, 18),
];
#[rustfmt::skip]
const BISHOP_MOB: [Score; 14] = [
    s(-48, -59), s(-20, -23), s(6, -3), s(14, 8), s(20, 16), s(26, 22), s(30, 28),
    s(32, 30), s(32, 34), s(34, 36), s(36, 38), s(36, 40), s(38, 42), s(40, 44),
];
#[rustfmt::skip]
const ROOK_MOB: [Score; 15] = [
    s(-60, -78), s(-20, -17), s(0, 16), s(2, 28), s(4, 48), s(8, 62), s(14, 66), s(18, 74),
    s(22, 78), s(22, 80), s(24, 84), s(26, 86), s(28, 88), s(28, 88), s(30, 90),
];
#[rustfmt::skip]
const QUEEN_MOB: [Score; 28] = [
    s(-30, -48), s(-12, -30), s(-8, -7), s(-8, 14), s(10, 30), s(12, 40), s(12, 44), s(18, 50),
    s(18, 52), s(26, 58), s(30, 60), s(30, 64), s(30, 72), s(30, 76), s(30, 80), s(30, 82),
    s(34, 84), s(34, 86), s(36, 90), s(36, 92), s(40, 94), s(42, 96), s(42, 96), s(42, 100),
    s(44, 102), s(46, 104), s(46, 106), s(48, 112),
];

const ISOLATED: Score = s(-8, -16);
const DOUBLED: Score = s(-8, -16);
const BACKWARD: Score = s(-9, -20);
const CONNECTED: [i32; 8] = [0, 0, 7, 8, 12, 29, 48, 86];
const PASSED_MG: [i32; 8] = [0, 0, 5, 10, 20, 40, 70, 120];
const PASSED_EG: [i32; 8] = [0, 0, 10, 20, 40, 80, 140, 220];
const BISHOP_PAIR: Score = s(30, 55);
const ROOK_OPEN: Score = s(48, 29);
const ROOK_SEMI: Score = s(19, 7);
const OUTPOST_KNIGHT: Score = s(54, 34);
const OUTPOST_BISHOP: Score = s(28, 20);
const MINOR_BEHIND_PAWN: Score = s(18, 0);
const KING_ATTACK_WEIGHT: [i32; 6] = [0, 20, 20, 40, 80, 0];
const SHELTER_BONUS: [[i32; 3]; 4] = [[0, 0, 0], [20, 14, 8], [10, 6, 2], [4, 2, 0]];
const SHELTER_MISSING: [i32; 3] = [22, 14, 6];
const TEMPO: i32 = 10;

/// Files (bits 0-7) holding at least one piece of `b`.
#[inline(always)]
fn file_set(b: Bitboard) -> u32 {
    let x = (b as u32) | ((b >> 32) as u32);
    let x = x | (x >> 16);
    (x | (x >> 8)) & 0xFF
}

#[inline(always)]
fn adjacent_files(f: usize) -> u32 {
    ((1u32 << f) << 1 | (1u32 << f) >> 1) & 0xFF
}

/// Squares in front of `sq` (from `c`'s view) on its file and the adjacent files.
#[inline(always)]
fn passed_span(c: usize, sq: usize) -> Bitboard {
    let f = file_bb(sq);
    let files = f | ((f & !FILE_H) << 1) | ((f & !FILE_A) >> 1);
    files & forward_ranks(c, sq)
}

/// All ranks strictly in front of `sq` from `c`'s view.
#[inline(always)]
fn forward_ranks(c: usize, sq: usize) -> Bitboard {
    let r = rank_of(sq);
    if c == WHITE {
        if r == 7 {
            0
        } else {
            !0u64 << ((r + 1) * 8)
        }
    } else if r == 0 {
        0
    } else {
        !0u64 >> ((8 - r) * 8)
    }
}

/// Pawn-only evaluation terms, cached by pawn key.
#[derive(Clone, Copy, Default)]
pub struct PawnEntry {
    pub key: u64,
    pub passed: [Bitboard; 2],
    /// White minus Black.
    pub score: Score,
}

pub const PAWN_CACHE: usize = 64;

pub struct PawnCache {
    pub e: [PawnEntry; PAWN_CACHE],
}

impl PawnCache {
    pub fn clear(&mut self) {
        for x in self.e.iter_mut() {
            *x = PawnEntry { key: u64::MAX, ..Default::default() };
        }
    }
}

fn pawn_structure(pos: &Position) -> PawnEntry {
    let pawns = [pos.pc(WHITE, PAWN), pos.pc(BLACK, PAWN)];
    let pawn_files = [file_set(pawns[0]), file_set(pawns[1])];
    let mut score = [0 as Score; 2];
    let mut passed = [0 as Bitboard; 2];
    for us in 0..2 {
        let them = us ^ 1;
        let own_pawns = pawns[us];
        let enemy_pawns = pawns[them];
        let own_files = pawn_files[us];
        let enemy_files = pawn_files[them];
        let mut sc: Score = 0;
        let mut b = own_pawns;
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let f = file_of(sq);
            let fbb = file_bb(sq);
            let adj_files_bb = ((fbb & !FILE_H) << 1) | ((fbb & !FILE_A) >> 1);
            let has_neighbor = own_files & adjacent_files(f) != 0;
            if !has_neighbor {
                sc += ISOLATED;
            }
            if own_pawns & file_bb(sq) & !bb(sq) != 0 {
                sc += DOUBLED;
            }
            let phalanx = own_pawns & adj_files_bb & rank_bb(sq) != 0;
            let supported = own_pawns & pawn_attacks(them, sq) != 0;
            let rank_idx = (relative_rank(us, sq) + 1).min(7);
            if phalanx || supported {
                let v = CONNECTED[rank_idx];
                sc += s(v, v * (rank_idx as i32 - 2).max(0) / 4);
            }
            if !supported && !phalanx && has_neighbor {
                let behind = forward_ranks(them, sq);
                let no_support_behind = own_pawns & adj_files_bb & behind == 0;
                let enemy_stop_file = enemy_files & adjacent_files(f) != 0;
                if no_support_behind && enemy_stop_file {
                    sc += BACKWARD;
                }
            }
            if enemy_pawns & passed_span(us, sq) == 0 {
                passed[us] |= bb(sq);
                sc += s(PASSED_MG[rank_idx], PASSED_EG[rank_idx]);
            }
        }
        score[us] = sc;
    }
    PawnEntry { key: pos.pawn_key, passed, score: score[WHITE].wrapping_sub(score[BLACK]) }
}

/// Static evaluation from the side to move's point of view (no pawn cache).
pub fn evaluate(pos: &Position) -> Value {
    if let Some(v) = kx_vs_k(pos) {
        return v;
    }
    let pe = pawn_structure(pos);
    evaluate_inner(pos, &pe)
}

/// Static evaluation using the searcher's pawn cache.
#[cfg_attr(target_os = "none", link_section = ".ramtext")]
pub fn evaluate_cached(pos: &Position, cache: &mut PawnCache) -> Value {
    if let Some(v) = kx_vs_k(pos) {
        return v;
    }
    let slot = (pos.pawn_key as usize) & (PAWN_CACHE - 1);
    if cache.e[slot].key != pos.pawn_key {
        cache.e[slot] = pawn_structure(pos);
    }
    let pe = cache.e[slot];
    evaluate_inner(pos, &pe)
}

#[inline(always)]
fn evaluate_inner(pos: &Position, pe: &PawnEntry) -> Value {
    let occ = pos.occ();
    let kings = [pos.king_sq(WHITE), pos.king_sq(BLACK)];
    let pawns = [pos.pc(WHITE, PAWN), pos.pc(BLACK, PAWN)];
    let pawn_files = [file_set(pawns[0]), file_set(pawns[1])];
    let mut score: [Score; 2] = [0, 0];
    let mut danger = [0i32; 2];

    for us in 0..2 {
        let them = us ^ 1;
        let own = pos.colors[us];
        let enemy_king = kings[them];
        let own_files = pawn_files[us];
        let enemy_files = pawn_files[them];
        let outpost_ranks = if us == WHITE { RANK_4 | RANK_5 | RANK_6 } else { RANK_3 | RANK_4 | RANK_5 };
        let mut sc: Score = 0;

        let mut b = pos.pc(us, KNIGHT);
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let mob = popcount(knight_attacks(sq) & !own) as usize;
            sc += KNIGHT_MOB[mob];
            if distance(sq, enemy_king) <= 3 {
                danger[them] += KING_ATTACK_WEIGHT[KNIGHT];
            }
            if bb(sq) & outpost_ranks != 0 {
                let adj = adjacent_files(file_of(sq));
                if own_files & adj != 0 && enemy_files & adj == 0 {
                    sc += OUTPOST_KNIGHT;
                }
            }
            if pawns[us] & push(bb(sq), us) != 0 {
                sc += MINOR_BEHIND_PAWN;
            }
        }

        let bishops = pos.pc(us, BISHOP);
        let mut b = bishops;
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let mob = popcount(bishop_attacks(sq, occ) & !own).min(13) as usize;
            sc += BISHOP_MOB[mob];
            if distance(sq, enemy_king) <= 4 {
                danger[them] += KING_ATTACK_WEIGHT[BISHOP];
            }
            if bb(sq) & outpost_ranks != 0 {
                let adj = adjacent_files(file_of(sq));
                if own_files & adj != 0 && enemy_files & adj == 0 {
                    sc += OUTPOST_BISHOP;
                }
            }
        }
        if bishops & LIGHT_SQUARES != 0 && bishops & DARK_SQUARES != 0 {
            sc += BISHOP_PAIR;
        }

        let mut b = pos.pc(us, ROOK);
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let mob = popcount(rook_attacks(sq, occ) & !own).min(14) as usize;
            sc += ROOK_MOB[mob];
            if distance(sq, enemy_king) <= 4 {
                danger[them] += KING_ATTACK_WEIGHT[ROOK];
            }
            let fbit = 1u32 << file_of(sq);
            if own_files & fbit == 0 {
                sc += if enemy_files & fbit == 0 { ROOK_OPEN } else { ROOK_SEMI };
            }
        }

        let mut b = pos.pc(us, QUEEN);
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let mob = popcount(queen_attacks(sq, occ) & !own).min(27) as usize;
            sc += QUEEN_MOB[mob];
            if distance(sq, enemy_king) <= 5 {
                danger[them] += KING_ATTACK_WEIGHT[QUEEN];
            }
        }

        let own_pawns = pawns[us];

        // King shelter (middlegame)
        let ksq = kings[us];
        let kf = file_of(ksq) as i32;
        let ahead = forward_ranks(us, ksq);
        let mut shelter = 0;
        for df in -1i32..=1 {
            let f = (kf + df).clamp(0, 7) as usize;
            let rel = df.unsigned_abs().min(2) as usize;
            let on_file = own_pawns & (FILE_A << f) & ahead;
            if on_file != 0 {
                let closest = if us == WHITE { lsb(on_file) } else { msb(on_file) };
                let dist = (rank_of(closest) as i32 - rank_of(ksq) as i32).unsigned_abs() as usize;
                if dist <= 3 {
                    shelter += SHELTER_BONUS[dist][rel];
                    continue;
                }
            }
            shelter -= SHELTER_MISSING[rel];
        }
        sc += s(shelter, 0);
        score[us] = sc;
    }

    for us in 0..2 {
        if danger[us] > 0 {
            score[us] -= s(danger[us] * danger[us] / 256, 0);
        }
    }

    // Passed pawns: king proximity to the promotion square (endgame).
    for us in 0..2 {
        let them = us ^ 1;
        let mut b = pe.passed[us];
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let rank_idx = (relative_rank(us, sq) + 1).min(7);
            if rank_idx >= 3 {
                let f = file_of(sq);
                let promo = if us == WHITE { 56 + f } else { f };
                let own_dist = distance(kings[us], promo).min(5) as i32;
                let opp_dist = distance(kings[them], promo).min(5) as i32;
                score[us] += s(0, (opp_dist - own_dist) * (rank_idx as i32 - 2) * 5);
            }
        }
    }

    let total = pos.psq.wrapping_add(pe.score).wrapping_add(score[WHITE]).wrapping_sub(score[BLACK]);
    let phase = pos.phase.min(MAX_PHASE);
    let v = (mg(total) * phase + eg(total) * (MAX_PHASE - phase)) / MAX_PHASE;
    let v = if pos.side == WHITE as u8 { v } else { -v };
    v + TEMPO
}
