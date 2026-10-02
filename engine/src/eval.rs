//! Hand-crafted evaluation: incrementally kept material + PeSTO tables, plus mobility,
//! pawn structure, king shelter and attacker pressure, rook files, outposts and the
//! bishop pair. Port of Peras' HCE onto packed scores. KX vs K positions use the
//! mop-up evaluation instead.

use crate::attacks::*;
use crate::mopup::kx_vs_k;
use crate::position::Position;
use crate::psqt::{eg, mg, s, Score, MAX_PHASE};
use crate::types::*;

/// A Stockfish Classic score converted to PeSTO units: a pawn is 126/208 there, 82/94 here.
const fn sf(mg: i32, eg: i32) -> Score {
    s(mg * 82 / 126, eg * 94 / 208)
}

// Mobility over Stockfish Classic's mobility area, with its values.
#[rustfmt::skip]
const KNIGHT_MOB: [Score; 9] = [
    sf(-62, -81), sf(-53, -56), sf(-12, -31), sf(-4, -16), sf(3, 5), sf(13, 11), sf(22, 17), sf(28, 20), sf(33, 25),
];
#[rustfmt::skip]
const BISHOP_MOB: [Score; 14] = [
    sf(-48, -59), sf(-20, -23), sf(16, -3), sf(26, 13), sf(38, 24), sf(51, 42), sf(55, 54),
    sf(63, 57), sf(63, 65), sf(68, 73), sf(81, 78), sf(81, 86), sf(91, 88), sf(98, 97),
];
#[rustfmt::skip]
const ROOK_MOB: [Score; 15] = [
    sf(-60, -78), sf(-20, -17), sf(2, 23), sf(3, 39), sf(3, 70), sf(11, 99), sf(22, 103), sf(31, 121),
    sf(40, 134), sf(40, 139), sf(41, 158), sf(48, 164), sf(57, 168), sf(57, 169), sf(62, 172),
];
#[rustfmt::skip]
const QUEEN_MOB: [Score; 28] = [
    sf(-30, -48), sf(-12, -30), sf(-8, -7), sf(-9, 19), sf(20, 40), sf(23, 55), sf(23, 59), sf(35, 75),
    sf(38, 78), sf(53, 96), sf(64, 96), sf(65, 100), sf(65, 121), sf(66, 127), sf(67, 131), sf(67, 133),
    sf(72, 136), sf(72, 141), sf(77, 147), sf(79, 150), sf(93, 151), sf(108, 168), sf(108, 168), sf(108, 171),
    sf(110, 182), sf(114, 182), sf(114, 192), sf(116, 219),
];

// Threats, from Stockfish Classic, indexed by the attacked piece type.
const THREAT_BY_MINOR: [Score; 6] = [sf(5, 32), sf(57, 41), sf(77, 56), sf(88, 119), sf(79, 161), 0];
const THREAT_BY_ROOK: [Score; 6] = [sf(3, 46), sf(37, 68), sf(42, 60), sf(0, 38), sf(58, 41), 0];
const THREAT_BY_KING: Score = sf(24, 89);
const HANGING: Score = sf(69, 36);
const WEAK_QUEEN_PROTECTION: Score = sf(14, 0);
const RESTRICTED_PIECE: Score = sf(7, 7);
const THREAT_BY_SAFE_PAWN: Score = sf(173, 94);
const THREAT_BY_PAWN_PUSH: Score = sf(48, 39);
const KNIGHT_ON_QUEEN: Score = sf(16, 11);
const SLIDER_ON_QUEEN: Score = sf(60, 18);

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

/// Stockfish Classic's threats: pieces attacked by lesser ones or left undefended, safe
/// pawn attacks and pushes, restricted squares and attacks on the queen.
#[inline(never)]
#[cfg_attr(target_os = "none", link_section = ".ramtext")]
fn threats(
    pos: &Position,
    us: usize,
    att: &[[Bitboard; 6]; 2],
    all: &[Bitboard; 2],
    att2: &[Bitboard; 2],
    area: &[Bitboard; 2],
) -> Score {
    let them = us ^ 1;
    let occ = pos.occ();
    let mut sc: Score = 0;
    let non_pawn_enemies = pos.colors[them] & !pos.pieces[PAWN];
    let strongly_protected = att[them][PAWN] | (att2[them] & !att2[us]);
    let defended = non_pawn_enemies & strongly_protected;
    let weak = pos.colors[them] & !strongly_protected & all[us];

    if defended | weak != 0 {
        let mut b = (defended | weak) & (att[us][KNIGHT] | att[us][BISHOP]);
        while b != 0 {
            sc += THREAT_BY_MINOR[ptype(pos.board[pop_lsb(&mut b)])];
        }
        let mut b = weak & att[us][ROOK];
        while b != 0 {
            sc += THREAT_BY_ROOK[ptype(pos.board[pop_lsb(&mut b)])];
        }
        if weak & att[us][KING] != 0 {
            sc += THREAT_BY_KING;
        }
        let b = !all[them] | (non_pawn_enemies & att2[us]);
        sc += HANGING * popcount(weak & b) as i32;
        sc += WEAK_QUEEN_PROTECTION * popcount(weak & att[them][QUEEN]) as i32;
    }

    sc += RESTRICTED_PIECE * popcount(all[them] & !strongly_protected & all[us]) as i32;

    let safe = !all[them] | all[us];
    let b = pawn_attacks_bb(pos.pc(us, PAWN) & safe, us) & non_pawn_enemies;
    sc += THREAT_BY_SAFE_PAWN * popcount(b) as i32;

    let rank3 = if us == WHITE { RANK_3 } else { RANK_6 };
    let mut b = push(pos.pc(us, PAWN), us) & !occ;
    b |= push(b & rank3, us) & !occ;
    b &= !att[them][PAWN] & safe;
    sc += THREAT_BY_PAWN_PUSH * popcount(pawn_attacks_bb(b, us) & non_pawn_enemies) as i32;

    let their_queens = pos.pc(them, QUEEN);
    if their_queens != 0 && !more_than_one(their_queens) {
        // Doubled when the queen is the only one on the board.
        let imbalance = if more_than_one(pos.pieces[QUEEN]) { 1 } else { 2 };
        let s = lsb(their_queens);
        let safe = area[us] & !pos.pc(us, PAWN) & !strongly_protected;
        let b = att[us][KNIGHT] & knight_attacks(s);
        sc += KNIGHT_ON_QUEEN * (popcount(b & safe) as i32 * imbalance);
        let b = (att[us][BISHOP] & bishop_attacks(s, occ)) | (att[us][ROOK] & rook_attacks(s, occ));
        sc += SLIDER_ON_QUEEN * (popcount(b & safe & att2[us]) as i32 * imbalance);
    }
    sc
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

    // Attack maps: by colour and piece type, all pieces, and squares attacked twice.
    let mut att = [[0 as Bitboard; 6]; 2];
    let mut all = [0 as Bitboard; 2];
    let mut att2 = [0 as Bitboard; 2];
    let mut area = [0 as Bitboard; 2];
    for us in 0..2 {
        let them = us ^ 1;
        let p = pawns[us];
        let double_pawn = if us == WHITE {
            ((p & !FILE_A) << 7) & ((p & !FILE_H) << 9)
        } else {
            ((p & !FILE_A) >> 9) & ((p & !FILE_H) >> 7)
        };
        att[us][KING] = king_attacks(kings[us]);
        att[us][PAWN] = pawn_attacks_bb(p, us);
        all[us] = att[us][KING] | att[us][PAWN];
        att2[us] = double_pawn | (att[us][KING] & att[us][PAWN]);
        // Mobility excludes our blocked or unmoved pawns, our king and queen, and squares
        // enemy pawns attack.
        let low_ranks = if us == WHITE { RANK_2 | RANK_3 } else { RANK_7 | RANK_6 };
        let blocked = p & (push(occ, them) | low_ranks);
        area[us] = !(blocked | pos.pc(us, KING) | pos.pc(us, QUEEN) | pawn_attacks_bb(pawns[them], them));
    }
    let queens = pos.pieces[QUEEN];

    for us in 0..2 {
        let them = us ^ 1;
        let enemy_king = kings[them];
        let own_files = pawn_files[us];
        let enemy_files = pawn_files[them];
        let outpost_ranks = if us == WHITE { RANK_4 | RANK_5 | RANK_6 } else { RANK_3 | RANK_4 | RANK_5 };
        let mut sc: Score = 0;

        let mut b = pos.pc(us, KNIGHT);
        while b != 0 {
            let sq = pop_lsb(&mut b);
            let a = knight_attacks(sq);
            att2[us] |= all[us] & a;
            att[us][KNIGHT] |= a;
            all[us] |= a;
            sc += KNIGHT_MOB[popcount(a & area[us]) as usize];
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
            // X-rays through queens.
            let a = bishop_attacks(sq, occ ^ queens);
            att2[us] |= all[us] & a;
            att[us][BISHOP] |= a;
            all[us] |= a;
            sc += BISHOP_MOB[popcount(a & area[us]) as usize];
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

        let rooks = pos.pc(us, ROOK);
        let mut b = rooks;
        while b != 0 {
            let sq = pop_lsb(&mut b);
            // X-rays through queens and our other rooks.
            let a = rook_attacks(sq, occ ^ queens ^ rooks);
            att2[us] |= all[us] & a;
            att[us][ROOK] |= a;
            all[us] |= a;
            sc += ROOK_MOB[popcount(a & area[us]) as usize];
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
            let a = queen_attacks(sq, occ);
            att2[us] |= all[us] & a;
            att[us][QUEEN] |= a;
            all[us] |= a;
            sc += QUEEN_MOB[popcount(a & area[us]) as usize];
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

    for (us, sc) in score.iter_mut().enumerate() {
        *sc += threats(pos, us, &att, &all, &att2, &area);
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
