"""Use a micro:bit running Monas as a UCI engine in any chess GUI.

Point the GUI at:  python serial_uci.py COM5      (or /dev/ttyACM0 on Linux)

It relays stdin to the micro:bit's USB serial port (115200 baud) and the device's
output back to stdout. Requires pyserial (pip install pyserial).
"""
import sys
import threading

import serial


def main():
    if len(sys.argv) < 2:
        print("usage: serial_uci.py <serial port>", file=sys.stderr)
        sys.exit(2)
    port = serial.Serial(sys.argv[1], 115200, timeout=0.1)

    def pump():
        buf = b""
        while True:
            data = port.read(256)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="replace").rstrip("\r")
                # The boot banner is not part of UCI.
                if text.startswith("Monas for micro:bit"):
                    continue
                sys.stdout.write(text + "\n")
                sys.stdout.flush()

    threading.Thread(target=pump, daemon=True).start()
    for line in sys.stdin:
        port.write(line.rstrip("\r\n").encode() + b"\n")
        port.flush()
        if line.strip() == "quit":
            break


if __name__ == "__main__":
    main()
