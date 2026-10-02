# Development

How Monas is tested without a micro:bit, and how to rebuild the pieces.

## Layout

| Directory     | Contents                                                          |
|---------------|-------------------------------------------------------------------|
| `engine/`     | The engine as a `no_std` library: board, search, evaluation, UCI  |
| `firmware/`   | micro:bit v2 firmware: startup, UART, LEDs, OLED and joystick UI  |
| `host/`       | PC build with UCI on stdin and stdout, and the perft test         |
| `bitsim/`     | micro:bit v2 emulator                                             |
| `tools/`      | Match runner with SPRT, opening book generator, serial UCI bridge |
| `books/`      | Opening book used for matches                                     |
| `tests_ui/`   | Scripts that drive the OLED and joystick UI in the emulator       |
| `dist/`       | Built firmware (see Building in the README; not tracked by git)   |

## The emulator

`bitsim` runs micro:bit v2 firmware images (ELF or `.hex`) and counts time in CPU cycles
at 64 MHz. It models:

- the Cortex-M4F core: Thumb-2 with the DSP and single-precision FPU instructions,
  exceptions and the NVIC;
- instruction timings from the Cortex-M4 technical reference manual;
- the nRF52833 memory system: flash wait states, the 2 KB instruction cache, and RAM
  with its code-RAM alias. Nordic does not publish the cache's layout, so it is modelled
  as 2-way set associative with 16-byte lines;
- the peripherals micro:bit firmware uses: CLOCK, POWER, NVMC, UART and UARTE (timed at
  the baud rate), TIMER, RTC, SAADC, TWIM, GPIO, GPIOTE, PPI, EGU, RNG and TEMP. An
  SSD1306 OLED can be attached to the I2C bus and the joystick inputs set from a script.

It also boots MakeCode programs, including Nordic's boot record, bootloader and the
CODAL runtime, and runs at about twice the speed of a real micro:bit.

```bash
cargo build --release -p bitsim
# send serial commands, each waiting for its answer
./target/release/bitsim dist/monas-microbit-v2.hex uci "position startpos" "go movetime 5000"
# behave as a UCI engine on stdin and stdout
./target/release/bitsim dist/monas-microbit-v2.hex --uci
# the same for GUIs that take a program without arguments (loads dist/monas-microbit-v2.hex)
./target/release/monas-uci
# play through the OLED and joystick UI and print the screen
./target/release/bitsim dist/monas-microbit-v2.hex --ui tests_ui/play_e4.txt
# show which functions the time goes to (needs the ELF for symbols)
./target/release/bitsim firmware/target/thumbv7em-none-eabihf/release/monas-microbit --profile "bench 8"
```

UI scripts take one command per line: `wait <seconds>`, `press <A|B|C|D|E|F> [ms]`,
`joy <up|down|left|right> [ms]`, `screen` and `serial <line>`.

## Matches and SPRT

`sprt` plays games between two engines. Each engine is a firmware image on its own
emulated micro:bit (`sim=`) or a PC program (`cmd=`). Emulated engines are charged
device time, from the moment the `go` command has arrived over serial until `bestmove`
is sent. Each opening is played twice with colours swapped, and the result is an SPRT on
game pairs.

```bash
cargo build --release -p monas-tools
./target/release/sprt \
  --engine "name=new sim=new.elf" \
  --engine "name=old sim=old.elf" \
  --tc 10+0.1 --book books/book8.epd --concurrency 14 --sprt 0 5 --pgn games.pgn
```

Use `--movetime <ms>` for a fixed time per move instead of a clock.

The opening book holds lines of 8 plies picked from reasonable moves and checked for
balance. Regenerate it with `./target/release/book 3000 8 books/book8.epd`.
