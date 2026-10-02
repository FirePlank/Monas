//! KX-vs-K evaluation, ported from Peras.
//!
//! The normal evaluation gives no direction once the bare king is loose, so the winning
//! side drifts until the fifty-move rule takes the win away. Instead the evaluation is
//! replaced outright: a constant saying the position is won, plus a small gradient that
//! always points at the mate.

use crate::attacks::distance;
use crate::position::Position;
use crate::types::*;

/// Base score for a position the winning side mates from by force. Far above any shaping
/// below, and far enough below the mate range to still read as an evaluation.
const KNOWN_WIN: Value = 2000;

/// Weight on the corner term for K+B+N, which has to outweigh everything else: the mate
/// exists in only two of the four corners, so steering into a wrong one is a lost game.
const CORNER_WEIGHT: i32 = 560;

/// Piece values for the material term, as in Peras.
const PIECE_VALUE: [Value; 5] = [100, 320, 330, 500, 1000];

/// Distance from the nearer edge, 0 on the rim to 3 in the middle.
#[inline]
fn edge_distance(x: usize) -> i32 {
    let x = x as i32;
    x.min(7 - x)
}

/// Drives the bare king to an edge: 90 on the rim, down to 28 in the centre.
#[inline]
fn push_to_edge(sq: usize) -> i32 {
    let rd = edge_distance(rank_of(sq));
    let fd = edge_distance(file_of(sq));
    90 - (7 * fd * fd / 2 + 7 * rd * rd / 2)
}

/// Brings the kings together: 120 when adjacent, falling to 0 across the board.
#[inline]
fn push_close(a: usize, b: usize) -> i32 {
    140 - 20 * distance(a, b) as i32
}

/// Distance from the a1/h8 corners: 0 along the a8-h1 diagonal, 7 at a1 and h8.
#[inline]
fn push_to_corner(sq: usize) -> i32 {
    (7 - rank_of(sq) as i32 - file_of(sq) as i32).abs()
}

/// True when `c`, having no pawns, cannot force mate against a bare king.
fn side_cannot_mate(pos: &Position, c: usize) -> bool {
    if pos.pc(c, PAWN) != 0 {
        return false;
    }
    if pos.pc(c, ROOK) | pos.pc(c, QUEEN) != 0 {
        return false;
    }
    let knights = pos.pc(c, KNIGHT);
    let bishops = pos.pc(c, BISHOP);
    if bishops == 0 {
        // Two knights cannot force mate.
        return popcount(knights) <= 2;
    }
    if knights == 0 {
        // Bishops on one square colour never mate.
        return bishops & LIGHT_SQUARES == 0 || bishops & DARK_SQUARES == 0;
    }
    false
}

fn non_pawn_value(pos: &Position, c: usize) -> Value {
    (KNIGHT..=QUEEN).map(|pt| PIECE_VALUE[pt] * popcount(pos.pc(c, pt)) as Value).sum()
}

/// A complete evaluation for KX vs K from the side to move's point of view, or `None` when
/// the position is not one.
pub fn kx_vs_k(pos: &Position) -> Option<Value> {
    for winner in [WHITE, BLACK] {
        let loser = winner ^ 1;

        // The defender must be a bare king, and the winner must be able to force mate with
        // pieces alone. With a pawn the win runs through promotion, which is not always a win.
        if pos.colors[loser] != pos.pc(loser, KING) {
            continue;
        }
        if pos.pc(winner, PAWN) != 0 || side_cannot_mate(pos, winner) {
            continue;
        }

        let wk = pos.king_sq(winner);
        let lk = pos.king_sq(loser);
        let bishops = pos.pc(winner, BISHOP);
        let knights = pos.pc(winner, KNIGHT);

        let shaping = if popcount(bishops) == 1 && popcount(knights) == 1 {
            // K+B+N mates only in the two corners the bishop covers, so distance to the
            // nearer of those is the whole objective.
            let dark_bishop = bishops & DARK_SQUARES != 0;
            let target = if dark_bishop { lk } else { lk ^ 7 };
            CORNER_WEIGHT * push_to_corner(target) + push_close(wk, lk)
        } else {
            push_to_edge(lk) + push_close(wk, lk)
        };

        let v = KNOWN_WIN + non_pawn_value(pos, winner) + shaping;
        let v = if pos.side as usize == winner { v } else { -v };
        // Fifty-move damping, so captures and progress keep their pull.
        let clock = (pos.rule50 as i32).min(199);
        let v = v - v * clock / 199;
        return Some(v.clamp(-(VALUE_MATE_IN_MAX_PLY - 1), VALUE_MATE_IN_MAX_PLY - 1));
    }
    None
}
