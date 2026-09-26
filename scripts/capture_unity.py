#!/usr/bin/env python3
"""Read a Unity test run from a USB-Serial/JTAG console without resetting the chip.

Flash with a normal `idf.py -p COM4 flash` (its hard reset boots the app), then run this
within the test app's start-up delay. The port is opened with DTR and RTS held low, so
opening it neither resets the chip nor arms ROM download mode. Exits 0 when the Unity
summary line shows zero failures, 1 when it shows failures, 2 on timeout.

Why not reset from here: on Windows an RTS reset over the USB-Serial/JTAG lands in ROM
download mode whenever the ROM's force-download flag is set, which any esptool session
ending in `--after no-reset` leaves behind (.claude/BACKLOG.md, "Capturing a test run").

usage: python scripts/capture_unity.py [--port COM4] [--timeout 180] [--out run.log]
"""
import argparse
import re
import sys
import time

import serial

SUMMARY = re.compile(r"(\d+) Tests (\d+) Failures (\d+) Ignored")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--timeout", type=float, default=180)
    ap.add_argument("--out")
    a = ap.parse_args()

    ser = serial.Serial()
    ser.port = a.port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()

    out = open(a.out, "w", encoding="utf-8", errors="replace") if a.out else None
    end = time.time() + a.timeout
    buf = b""
    result = None
    while time.time() < end and result is None:
        chunk = ser.read(4096)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", errors="replace").rstrip("\r")
            print(text, flush=True)
            if out:
                out.write(text + "\n")
            m = SUMMARY.search(text)
            if m:
                result = int(m.group(2))
    ser.close()
    if out:
        out.close()
    if result is None:
        print("timeout: no Unity summary line", file=sys.stderr)
        return 2
    return 0 if result == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
