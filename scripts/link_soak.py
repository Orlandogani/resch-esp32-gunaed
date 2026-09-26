#!/usr/bin/env python3
"""Two-board soak of the wireless audio link (SDD §16.3, SAD risk R11).

Tails the dongle's and the headset's consoles at once, timestamps every line, keeps a raw
log per board, parses the periodic statistics lines both firmwares print, writes them to
CSV, and judges the run: is the headset's playback backlog bounded, is it drifting, how
often is audio concealed, does playback underrun.

Both boards must run console-safe images, so their USB-Serial/JTAG ports stay up:
  applications/dongle   default build (DONGLE_ENABLE_USB=n: the PC is emulated)
  applications/headset  default build (HEADSET_ENABLE_USB=n: wlink profile)

The ports are opened without toggling DTR/RTS, so nothing is reset.

usage: python scripts/link_soak.py --dongle COM5 --headset COM4 --hours 4 --out soak/
"""
import argparse
import csv
import os
import re
import statistics
import sys
import threading
import time

import serial

# Printed by applications/dongle backends/wlink_central (central_log).
RE_DONGLE_LINK = re.compile(
    r"wlink: (?P<state>UP|down) \| tx (?P<tx>\d+) busy (?P<busy>\d+) skip (?P<skip>\d+) \| "
    r"rx (?P<rx>\d+) lost (?P<lost>\d+) late (?P<late>\d+) conceal (?P<conceal>\d+) \| "
    r"peer backlog (?P<peer>\d+) fr \((?P<age>\d+) ms old\) \| "
    r"enc (?P<enc_avg>\d+)/(?P<enc_max>\d+) us dec (?P<dec_avg>\d+)/(?P<dec_max>\d+) us \| "
    r"stack free (?P<stack>\d+)")
# Printed by applications/dongle host_port (emulated host).
RE_DONGLE_HOST = re.compile(
    r"emu spk (?P<frames>\d+) frames, trim (?P<trim>[+-]?\d+) \(up (?P<up>\d+) down (?P<down>\d+)\), "
    r"path backlog (?P<backlog>\d+)/(?P<target>\d+) B \| mic (?P<mic>\d+) samples, short (?P<short>\d+), "
    r"RMS (?P<rms>\d+)")
# Printed by applications/headset profiles/wlink (log_stats).
RE_HEADSET = re.compile(
    r"LINK (?P<state>UP|down) rx (?P<rx>\d+) lost (?P<lost>\d+) late (?P<late>\d+) "
    r"conceal (?P<conceal>\d+) resync (?P<resync>\d+) \| tx (?P<tx>\d+) busy (?P<busy>\d+) \| "
    r"dec (?P<dec_avg>\d+)/(?P<dec_max>\d+) us enc (?P<enc_avg>\d+)/(?P<enc_max>\d+) us "
    r"stack (?P<stack>\d+) \| PLAY backlog (?P<backlog>\d+) B underruns (?P<underruns>\d+) "
    r"starv (?P<starv>\d+) prefills (?P<prefills>\d+)")

BYTES_PER_MS = 48 * 2 * 2   # 48 kHz stereo 16-bit: the default speaker format


class Board(threading.Thread):
    def __init__(self, name, port, out_dir, patterns):
        super().__init__(daemon=True)
        self.name = name
        self.patterns = patterns
        self.rows = {key: [] for key in patterns}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.timeout = 0.2
        self.ser.dtr = False          # never reset or force download mode
        self.ser.rts = False
        self.ser.open()
        self.raw = open(os.path.join(out_dir, f"{name}.log"), "w", encoding="utf-8", errors="replace")
        self.csv = {}
        for key, rx in patterns.items():
            f = open(os.path.join(out_dir, f"{name}_{key}.csv"), "w", newline="", encoding="utf-8")
            w = csv.writer(f)
            w.writerow(["t_s"] + list(rx.groupindex))
            self.csv[key] = (f, w)

    def run(self):
        t0 = time.time()
        buf = b""
        while not self.stop.is_set():
            chunk = self.ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                t = time.time() - t0
                self.raw.write(f"{t:10.3f} {text}\n")
                for key, rx in self.patterns.items():
                    m = rx.search(text)
                    if m:
                        row = {k: (v if k == "state" else int(v)) for k, v in m.groupdict().items()}
                        row["t_s"] = t
                        with self.lock:
                            self.rows[key].append(row)
                        f, w = self.csv[key]
                        w.writerow([f"{t:.1f}"] + [m.group(k) for k in rx.groupindex])
                        f.flush()
        self.ser.close()
        self.raw.close()
        for f, _ in self.csv.values():
            f.close()

    def snapshot(self, key):
        with self.lock:
            return list(self.rows[key])


def slope_per_hour(rows, field):
    """Least-squares slope of `field` against time, in units per hour."""
    if len(rows) < 3:
        return 0.0
    xs = [r["t_s"] / 3600.0 for r in rows]
    ys = [r[field] for r in rows]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return 0.0 if den == 0 else sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def report(dongle, headset, final):
    hs = [r for r in headset.snapshot("link") if r["state"] == "UP"]
    host = dongle.snapshot("host")
    link = dongle.snapshot("link")
    lines = []
    ok = True
    if len(hs) < 3:
        lines.append("headset: no UP statistics yet")
        return "\n".join(lines), final and False
    elapsed_h = (hs[-1]["t_s"] - hs[0]["t_s"]) / 3600.0
    backlog_ms = [r["backlog"] / BYTES_PER_MS for r in hs]
    drift = slope_per_hour(hs, "backlog") / BYTES_PER_MS
    conceal = hs[-1]["conceal"] - hs[0]["conceal"]
    under = hs[-1]["underruns"] - hs[0]["underruns"]
    minutes = max(elapsed_h * 60.0, 1e-9)
    lines.append(f"elapsed {elapsed_h:.2f} h over {len(hs)} headset samples")
    lines.append(f"headset playback backlog: min {min(backlog_ms):.1f} ms, mean {statistics.fmean(backlog_ms):.1f} ms, "
                 f"max {max(backlog_ms):.1f} ms, drift {drift:+.2f} ms/h")
    lines.append(f"concealed {conceal} frames ({conceal / minutes:.2f}/min), playback underruns {under}")
    if host:
        lines.append(f"dongle servo trims: up {host[-1]['up']} down {host[-1]['down']}, "
                     f"path backlog {host[-1]['backlog']}/{host[-1]['target']} B")
    if link:
        lines.append(f"dongle link: tx {link[-1]['tx']} busy {link[-1]['busy']} skip {link[-1]['skip']}, "
                     f"enc max {link[-1]['enc_max']} us")
    # Verdict thresholds: a healthy servo holds the backlog within a few ms of its target
    # with no trend; concealment should be rare in a quiet RF environment.
    if abs(drift) > 2.0:
        lines.append("FAIL: playback backlog is drifting (> 2 ms/h): the servo is not holding the rate")
        ok = False
    if min(backlog_ms) < 2.0:
        lines.append("FAIL: playback backlog reached ~0: the headset ran dry")
        ok = False
    if conceal / minutes > 1.0:
        lines.append("WARN: more than one concealed frame per minute")
    return "\n".join(lines), ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dongle", required=True, help="dongle console port, e.g. COM5")
    ap.add_argument("--headset", required=True, help="headset console port, e.g. COM4")
    ap.add_argument("--hours", type=float, default=4.0)
    ap.add_argument("--report-min", type=float, default=10.0, help="interim report period")
    ap.add_argument("--out", default="soak")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    dongle = Board("dongle", a.dongle, a.out, {"link": RE_DONGLE_LINK, "host": RE_DONGLE_HOST})
    headset = Board("headset", a.headset, a.out, {"link": RE_HEADSET})
    dongle.start()
    headset.start()
    end = time.time() + a.hours * 3600.0
    next_report = time.time() + a.report_min * 60.0
    try:
        while time.time() < end:
            time.sleep(1.0)
            if time.time() >= next_report:
                next_report += a.report_min * 60.0
                text, _ = report(dongle, headset, final=False)
                print(f"--- {time.strftime('%H:%M:%S')}\n{text}", flush=True)
    except KeyboardInterrupt:
        pass
    dongle.stop.set()
    headset.stop.set()
    dongle.join(2)
    headset.join(2)
    text, ok = report(dongle, headset, final=True)
    print(f"=== final\n{text}\nVERDICT: {'PASS' if ok else 'FAIL'}")
    with open(os.path.join(a.out, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + f"\nVERDICT: {'PASS' if ok else 'FAIL'}\n")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
