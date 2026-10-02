//! Monas on an emulated micro:bit as a UCI engine, for GUIs that take a program
//! without arguments. Runs `dist/monas-microbit-v2.hex` from the repository this binary
//! was built in (it lives in `target/release`), or the image named by `MONAS_HEX`.
fn main() {
    let path = std::env::var("MONAS_HEX").unwrap_or_else(|_| {
        let exe = std::env::current_exe().expect("own path");
        let repo = exe.parent().and_then(|d| d.parent()).and_then(|d| d.parent()).expect("target/release");
        repo.join("dist").join("monas-microbit-v2.hex").to_string_lossy().into_owned()
    });
    let mut sim = bitsim::Sim::load(&path).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        std::process::exit(1)
    });
    bitsim::uci_loop(&mut sim);
}
