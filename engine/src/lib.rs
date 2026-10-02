//! Monas: a `no_std` chess engine designed for the BBC micro:bit v2.
#![no_std]

pub mod attacks;
pub mod eval;
pub mod mopup;
pub mod movegen;
pub mod picker;
pub mod position;
pub mod psqt;
pub mod search;
pub mod tt;
pub mod types;
pub mod uci;
pub mod zobrist;

/// Transposition table size used on the micro:bit, in 32-byte buckets. The PC build
/// uses the same size by default so that test results carry over to the device.
pub const TT_BUCKETS: usize = 1024;
