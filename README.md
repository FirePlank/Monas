# Monas

A chess engine for the BBC micro:bit v2. It runs bare metal on the nRF52833 and plays
on the micro:bit itself, with an OLED and a joystick, or as a UCI engine over USB.

Monas is the small sibling of [Peras](https://github.com/FirePlank/Peras) and
[Apeiron](https://github.com/FirePlank/infinite-chess-engine).

## Quick Start

1. Connect the micro:bit v2 to a computer. It shows up as a drive called `MICROBIT`.
2. Download `monas-microbit-v2.hex` from the [latest release](https://github.com/FirePlank/Monas/releases/latest)
   and copy it onto the `MICROBIT` drive.
3. The micro:bit restarts. The centre LED lights up when Monas is ready.

## Playing on the micro:bit

Monas uses the same add-ons as [Napablanca](https://github.com/asmrchess/Napablanca-Chess-Engine):
a Kitronik :VIEW 128x64 OLED and an ELECFREAKS joystick:bit. With the OLED attached it
starts in a menu where you choose Play as White, Play as Black or Self-play, then the
time per move (15 seconds by default).

| Control       | Action                                                          |
|---------------|-----------------------------------------------------------------|
| Joystick      | Move the cursor or the menu highlight                           |
| C             | Pick up a piece, or confirm in a menu                           |
| D             | Put the piece down                                              |
| E             | Show the engine's last move and evaluation                      |
| B (micro:bit) | Cancel or go back; while the engine thinks, make it move now    |

## Using it with a chess GUI

Over USB the micro:bit speaks UCI at 115200 baud. Chess GUIs expect a program, so add
this bridge as the engine (it needs `pip install pyserial`):

```bash
python tools/serial_uci.py COM5
```

## How it differs from a desktop engine

A micro:bit has 128 KB of RAM and a 64 MHz Cortex-M4, and reading flash costs extra
cycles whenever the 2 KB instruction cache misses. Monas is built around that:

- **No runtime.** It runs directly on the hardware instead of on MakeCode and CODAL,
  and searches about 9,500 positions per second.
- **Hot code runs from RAM.** The search, move generation and evaluation (37 KB) are
  copied into RAM at boot and run through the chip's code-RAM address range, which has
  no wait states. From flash they would keep missing the cache.
- **Small attack tables.** Desktop engines look up sliding attacks in tables of hundreds
  of kilobytes. Monas computes them with byte-reversal tricks the Cortex-M4 does in one
  instruction, so its tables take a few kilobytes and sit in RAM too.
- **A fixed memory budget.** The hash table is 32 KB, all positions in the search share
  one move buffer, UCI commands are parsed as they arrive instead of buffered, and a
  stack guard stops the search before it runs out of memory.
- **Serial output never blocks.** Sending a line takes milliseconds at 115200 baud, so
  output is queued and sent from an interrupt while the search continues.

Monas has not been run on a physical micro:bit yet. It was developed and tested on an
emulator of the micro:bit v2 that counts CPU cycles (see [DEVELOPMENT.md](DEVELOPMENT.md)).

## Building

```bash
rustup target add thumbv7em-none-eabihf
cd firmware
cargo build --release
mkdir -p ../dist
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/monas-microbit ../dist/monas-microbit-v2.hex
```

`rust-objcopy` comes with `cargo install cargo-binutils`.

## Credits

The OLED font is from Kitronik's [:VIEW 128x64 extension](https://github.com/KitronikLtd/pxt-kitronik-128x64Display)
(MIT License, Copyright (c) 2021 Kitronik Ltd).

## License

Monas is released under the [GNU General Public License v3.0](LICENSE).
