//! Opening book generator: short random but reasonable lines from the start position.
//! At each ply a move is drawn from those within 60 centipawns of the best one by a
//! depth-3 search; the final position must be within 70 centipawns at depth 10.
//!
//! book <count> <plies> <out.epd> [seed]

use monas::position::Position;
use monas_tools::*;
use std::collections::HashSet;
use std::io::Write;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let count: usize = args.get(1).and_then(|x| x.parse().ok()).unwrap_or(2000);
    let plies: usize = args.get(2).and_then(|x| x.parse().ok()).unwrap_or(8);
    let out = args.get(3).cloned().unwrap_or_else(|| "book.epd".into());
    let seed: u64 = args.get(4).and_then(|x| x.parse().ok()).unwrap_or(7);
    let threads = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
    let per = count.div_ceil(threads);
    let results = std::sync::Mutex::new(Vec::<(u64, String)>::new());
    std::thread::scope(|sc| {
        for t in 0..threads {
            let results = &results;
            sc.spawn(move || {
                let mut rng = Rng::new(seed * 1000 + t as u64);
                let mut s = new_searcher(1 << 14);
                let mut made = 0;
                while made < per {
                    let mut pos = Position::startpos();
                    let mut ok = true;
                    for _ in 0..plies {
                        let moves = legal_moves(&pos);
                        if moves.is_empty() {
                            ok = false;
                            break;
                        }
                        let mut scored = Vec::new();
                        for &m in &moves {
                            let mut c = pos;
                            c.do_move(m);
                            if legal_moves(&c).is_empty() {
                                continue;
                            }
                            let (_, v) = search_fixed(&mut s, &c, &[], Some(3), None);
                            scored.push((m, -v));
                        }
                        let best = scored.iter().map(|x| x.1).max().unwrap_or(0);
                        let cands: Vec<Move> = scored.iter().filter(|x| x.1 >= best - 60).map(|x| x.0).collect();
                        if cands.is_empty() {
                            ok = false;
                            break;
                        }
                        let m = cands[rng.below(cands.len())];
                        pos.do_move(m);
                    }
                    if !ok {
                        continue;
                    }
                    s.clear();
                    let (_, v) = search_fixed(&mut s, &pos, &[], Some(10), None);
                    if v.abs() > 70 {
                        continue;
                    }
                    let fen = fen_of(&pos);
                    results.lock().unwrap().push((pos.hash, fen));
                    made += 1;
                }
            });
        }
    });
    let mut seen = HashSet::new();
    let mut f = std::fs::File::create(&out).unwrap();
    let mut n = 0;
    for (k, fen) in results.into_inner().unwrap() {
        if seen.insert(k) {
            let parts: Vec<&str> = fen.split_whitespace().collect();
            writeln!(f, "{}", parts[..4].join(" ")).unwrap();
            n += 1;
        }
    }
    println!("wrote {} openings to {}", n, out);
}
use monas::types::Move;
