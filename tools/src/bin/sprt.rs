//! Match runner and SPRT.
//!
//! An engine is either a firmware image on an emulated micro:bit (`sim=image`, charged
//! device time) or a PC executable (`cmd=exe`), charged wall time scaled by `factor=F`,
//! or `vnps=N` virtual time where N nodes count as one second. Each opening is played
//! twice with colours swapped and the pairs are scored pentanomially.
//!
//! sprt --engine "name=new sim=new.elf" --engine "name=old sim=old.elf"
//!      --tc 10+0.1 --book book.epd --concurrency 14 --sprt 0 5 [--games N] [--pgn out.pgn]

use monas_tools::*;
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[derive(Clone, Copy)]
enum Tc {
    Clock { base_ms: u64, inc_ms: u64 },
    MoveTime(u64),
    Nodes(u64),
}

#[derive(Clone)]
struct Config {
    engines: [EngineSpec; 2],
    tc: Tc,
    adjudicate: bool,
    max_plies: usize,
}

struct GameRecord {
    /// Score of engine 0 in this game.
    score0: f64,
    pgn: String,
    reason: String,
    time_loss: bool,
}

fn play_game(
    cfg: &Config,
    engines: &mut [AnyEngine; 2],
    white: usize,
    fen: &str,
    round: usize,
) -> Result<GameRecord, String> {
    let mut game = Game::new(fen).expect("bad opening fen");
    for e in engines.iter_mut() {
        e.new_game()?;
    }
    // Clocks by colour.
    let mut clock = match cfg.tc {
        Tc::Clock { base_ms, .. } => [base_ms as i64, base_ms as i64],
        _ => [0, 0],
    };
    let mut scores_white: Vec<i32> = Vec::new();
    let mut outcome = None;
    let mut reason = String::new();
    let mut time_loss = false;
    while outcome.is_none() {
        if let Some((o, why)) = game.result() {
            outcome = Some(o);
            reason = why.to_string();
            break;
        }
        if game.moves.len() >= cfg.max_plies {
            outcome = Some(Outcome::Draw);
            reason = "max plies".into();
            break;
        }
        let stm = game.pos.us();
        let idx = if stm == 0 { white } else { 1 - white };
        let go = match cfg.tc {
            Tc::Clock { inc_ms, .. } => {
                format!("go wtime {} btime {} winc {} binc {}", clock[0].max(1), clock[1].max(1), inc_ms, inc_ms)
            }
            Tc::MoveTime(ms) => format!("go movetime {}", ms),
            Tc::Nodes(n) => format!("go nodes {}", n),
        };
        let budget = match cfg.tc {
            Tc::Clock { .. } => clock[stm].max(0) as u64,
            Tc::MoveTime(ms) => ms,
            Tc::Nodes(_) => 60_000,
        };
        let (mv, rep) =
            engines[idx].go(&game, &go, budget).map_err(|e| format!("{e} [{}] [{}]", game.uci_position(), go))?;
        if let Tc::Clock { inc_ms, .. } = cfg.tc {
            clock[stm] -= rep.charged_ms as i64;
            if clock[stm] < -50 {
                outcome = Some(if stm == 0 { Outcome::BlackWins } else { Outcome::WhiteWins });
                reason = format!("{} lost on time", engines[idx].spec().name);
                time_loss = true;
                break;
            }
            clock[stm] += inc_ms as i64;
        }
        let m = game.pos.parse_uci_move(&mv);
        let m = match m {
            Some(m) => m,
            None => {
                outcome = Some(if stm == 0 { Outcome::BlackWins } else { Outcome::WhiteWins });
                reason = format!("{} played illegal move {}", engines[idx].spec().name, mv);
                break;
            }
        };
        game.play(m);
        // Adjudication on the mover's reported score, converted to White's view.
        let sc = match (rep.mate, rep.score_cp) {
            (Some(mt), _) => {
                if mt > 0 {
                    30000
                } else {
                    -30000
                }
            }
            (None, Some(cp)) => cp,
            _ => 0,
        };
        scores_white.push(if stm == 0 { sc } else { -sc });
        if cfg.adjudicate {
            let n = scores_white.len();
            if n >= 8 {
                let last = &scores_white[n - 8..];
                if last.iter().all(|&s| s >= 1000) {
                    outcome = Some(Outcome::WhiteWins);
                    reason = "adjudicated win".into();
                } else if last.iter().all(|&s| s <= -1000) {
                    outcome = Some(Outcome::BlackWins);
                    reason = "adjudicated win".into();
                }
            }
            if outcome.is_none() && n >= 90 && n >= 12 {
                let last = &scores_white[n - 12..];
                if last.iter().all(|&s| s.abs() <= 10) {
                    outcome = Some(Outcome::Draw);
                    reason = "adjudicated draw".into();
                }
            }
        }
    }
    let o = outcome.unwrap();
    let s_white = o.white_score();
    let score0 = if white == 0 { s_white } else { 1.0 - s_white };
    let mut pgn = String::new();
    pgn.push_str(&format!("[Event \"Monas SPRT\"]\n[Round \"{}\"]\n", round));
    pgn.push_str(&format!(
        "[White \"{}\"]\n[Black \"{}\"]\n",
        engines[white].spec().name,
        engines[1 - white].spec().name
    ));
    pgn.push_str(&format!(
        "[Result \"{}\"]\n[FEN \"{}\"]\n[SetUp \"1\"]\n[Termination \"{}\"]\n\n",
        o.pgn(),
        fen,
        reason
    ));
    for (i, m) in game.moves.iter().enumerate() {
        if i % 2 == 0 {
            pgn.push_str(&format!("{}. ", i / 2 + 1));
        }
        pgn.push_str(&format!("{} ", m));
    }
    pgn.push_str(o.pgn());
    pgn.push_str("\n\n");
    Ok(GameRecord { score0, pgn, reason, time_loss })
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut specs = Vec::new();
    let mut tc = Tc::Clock { base_ms: 10_000, inc_ms: 100 };
    let mut book = String::new();
    let mut conc = 8usize;
    let mut sprt: Option<(f64, f64)> = None;
    let mut alpha = 0.05;
    let mut beta = 0.05;
    let mut max_games = 100_000usize;
    let mut pgn_path: Option<String> = None;
    let mut adjudicate = true;
    let mut seed = 1u64;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--engine" => {
                specs.push(parse_engine_spec(&args[i + 1]));
                i += 1;
            }
            "--tc" => {
                let (b, inc) = args[i + 1].split_once('+').unwrap_or((&args[i + 1], "0"));
                tc = Tc::Clock {
                    base_ms: (b.parse::<f64>().unwrap() * 1000.0) as u64,
                    inc_ms: (inc.parse::<f64>().unwrap() * 1000.0) as u64,
                };
                i += 1;
            }
            "--movetime" => {
                tc = Tc::MoveTime(args[i + 1].parse().unwrap());
                i += 1;
            }
            "--nodes" => {
                tc = Tc::Nodes(args[i + 1].parse().unwrap());
                i += 1;
            }
            "--book" => {
                book = args[i + 1].clone();
                i += 1;
            }
            "--concurrency" => {
                conc = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--sprt" => {
                sprt = Some((args[i + 1].parse().unwrap(), args[i + 2].parse().unwrap()));
                i += 2;
            }
            "--alpha" => {
                alpha = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--beta" => {
                beta = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--games" => {
                max_games = args[i + 1].parse().unwrap();
                i += 1;
            }
            "--pgn" => {
                pgn_path = Some(args[i + 1].clone());
                i += 1;
            }
            "--noadj" => adjudicate = false,
            "--seed" => {
                seed = args[i + 1].parse().unwrap();
                i += 1;
            }
            other => panic!("unknown argument {other}"),
        }
        i += 1;
    }
    assert_eq!(specs.len(), 2, "need exactly two --engine specs");
    let mut openings: Vec<String> = if book.is_empty() {
        vec![monas::position::START_FEN.to_string()]
    } else {
        BufReader::new(File::open(&book).expect("book"))
            .lines()
            .map_while(Result::ok)
            .map(|l| {
                // EPD lines: keep the first four fields and add clocks.
                let f: Vec<&str> = l.split_whitespace().collect();
                if f.len() >= 6 && f[4].parse::<u32>().is_ok() {
                    f[..6].join(" ")
                } else if f.len() >= 4 {
                    format!("{} 0 1", f[..4].join(" "))
                } else {
                    String::new()
                }
            })
            .filter(|l| !l.is_empty())
            .collect()
    };
    let mut rng = Rng::new(seed);
    for k in (1..openings.len()).rev() {
        let j = rng.below(k + 1);
        openings.swap(k, j);
    }
    let openings = Arc::new(openings);
    let cfg = Config { engines: [specs[0].clone(), specs[1].clone()], tc, adjudicate, max_plies: 500 };
    let next_pair = Arc::new(AtomicUsize::new(0));
    let stop = Arc::new(AtomicBool::new(false));
    let (tx, rx) = channel::<(usize, GameRecord, GameRecord)>();
    let max_pairs = max_games.div_ceil(2);

    let images: Arc<Vec<Option<bitsim::image::Image>>> = Arc::new(
        cfg.engines
            .iter()
            .map(|s| s.sim.as_ref().map(|p| bitsim::image::Image::load(p).expect("firmware image")))
            .collect(),
    );
    let mut handles = Vec::new();
    for _ in 0..conc {
        let cfg = cfg.clone();
        let openings = openings.clone();
        let next_pair = next_pair.clone();
        let stop = stop.clone();
        let tx = tx.clone();
        let images = images.clone();
        handles.push(std::thread::spawn(move || {
            let mut engines: Option<[AnyEngine; 2]> = None;
            while !stop.load(Ordering::Relaxed) {
                let p = next_pair.fetch_add(1, Ordering::Relaxed);
                if p >= max_pairs {
                    break;
                }
                let fen = &openings[p % openings.len()];
                for _attempt in 0..3 {
                    if engines.is_none() {
                        match (
                            AnyEngine::start(&cfg.engines[0], images[0].as_ref()),
                            AnyEngine::start(&cfg.engines[1], images[1].as_ref()),
                        ) {
                            (Ok(a), Ok(b)) => engines = Some([a, b]),
                            (a, b) => {
                                eprintln!("failed to start engines: {:?} {:?}", a.err(), b.err());
                                std::thread::sleep(std::time::Duration::from_secs(1));
                                continue;
                            }
                        }
                    }
                    let es = engines.as_mut().unwrap();
                    let g1 = play_game(&cfg, es, 0, fen, 2 * p + 1);
                    let g2 = match &g1 {
                        Ok(_) => Some(play_game(&cfg, es, 1, fen, 2 * p + 2)),
                        Err(_) => None,
                    };
                    match (g1, g2) {
                        (Ok(a), Some(Ok(b))) => {
                            let _ = tx.send((p, a, b));
                            break;
                        }
                        (a, b) => {
                            eprintln!("engine failure ({:?} / {:?}), restarting", a.err(), b.map(|x| x.err()));
                            engines = None;
                        }
                    }
                }
            }
        }));
    }
    drop(tx);

    let pgn_file = pgn_path.map(|p| Mutex::new(File::create(p).expect("pgn")));
    let mut penta = Penta::default();
    let (mut w, mut l, mut d) = (0u64, 0u64, 0u64);
    let mut time_losses = [0u64; 2];
    let mut reasons: std::collections::BTreeMap<String, u64> = Default::default();
    let t0 = Instant::now();
    let (lo, hi) = sprt_bounds(alpha, beta);
    let mut verdict = String::new();
    for (_p, a, b) in rx {
        for g in [&a, &b] {
            if g.score0 == 1.0 {
                w += 1;
            } else if g.score0 == 0.0 {
                l += 1;
            } else {
                d += 1;
            }
            *reasons.entry(g.reason.clone()).or_default() += 1;
            if g.time_loss {
                time_losses[if g.score0 == 0.0 { 0 } else { 1 }] += 1;
            }
            if let Some(f) = &pgn_file {
                let _ = f.lock().unwrap().write_all(g.pgn.as_bytes());
            }
        }
        let ps = a.score0 + b.score0;
        penta.n[(ps * 2.0).round() as usize] += 1;
        let games = w + l + d;
        let (elo, err) = penta.elo();
        let mut line = format!(
            "games {:5}  +{} -{} ={}  score {:.1}%  elo {:+.1} +/- {:.1}  penta {:?}",
            games,
            w,
            l,
            d,
            100.0 * (w as f64 + d as f64 / 2.0) / games as f64,
            elo,
            err,
            penta.n
        );
        if let Some((e0, e1)) = sprt {
            let llr = penta.llr(e0, e1);
            line.push_str(&format!("  LLR {:.2} [{:.2}, {:.2}]", llr, lo, hi));
            if llr >= hi {
                verdict = format!("H1 accepted: {} is stronger (elo1={})", cfg.engines[0].name, e1);
            } else if llr <= lo {
                verdict = format!("H0 accepted: {} is not stronger (elo0={})", cfg.engines[0].name, e0);
            }
        }
        if games % 20 == 0 || !verdict.is_empty() {
            println!("{}  [{:.0}s]", line, t0.elapsed().as_secs_f64());
        }
        if !verdict.is_empty() {
            stop.store(true, Ordering::Relaxed);
            break;
        }
    }
    stop.store(true, Ordering::Relaxed);
    for h in handles {
        let _ = h.join();
    }
    let games = w + l + d;
    let (elo, err) = penta.elo();
    println!(
        "\nfinal: {} vs {}: games {} +{} -{} ={} elo {:+.1} +/- {:.1} penta {:?}",
        cfg.engines[0].name, cfg.engines[1].name, games, w, l, d, elo, err, penta.n
    );
    println!("time losses: {} {}, {} {}", cfg.engines[0].name, time_losses[0], cfg.engines[1].name, time_losses[1]);
    println!("terminations: {:?}", reasons);
    if !verdict.is_empty() {
        println!("{}", verdict);
    }
}
