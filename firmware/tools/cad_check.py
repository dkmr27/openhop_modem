#!/usr/bin/env python3
"""cad_check.py — CAD busy/clear test with two boards.

For each SF and CAD symbol count: the CAD board runs a series of
CMD_CAD_REQUESTs with the channel quiet (any "busy" is a false alarm),
then again while the TX board transmits back-to-back long packets (the
share of "busy" is the detection rate). Busy verdicts include the
firmware's passive guard (a frame being received counts as busy without a
scan), which is what the host sees too.

Example:
  python cad_check.py --tx COM19 --cad COM11 --preset subg --sf-list 7,9,12
  python cad_check.py --tx COM11 --cad COM3 --preset 2g4 --sf-list 7,9,11,12 --sym 1,2,3
"""

import argparse
import json
import random
import sys
import threading
import time

import modem_check as mc

SYMBOLS = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16}
CORE_PEAK = {7: 22, 8: 22, 9: 24, 10: 25, 11: 26, 12: 30}   # SX126x-scale peaks; other chips ignore them


class Jammer(threading.Thread):
    """Back-to-back long packets from the TX board until stopped."""

    def __init__(self, modem, length):
        super().__init__(daemon=True)
        self.m = modem
        self.length = length
        self.stop_flag = threading.Event()
        self.sent = 0
        self.airtime_us = 0
        self.started = None

    def run(self):
        self.started = time.time()
        seq = 0
        while not self.stop_flag.is_set():
            try:
                airtime, _ = self.m.transmit(mc.test_payload(seq, self.length), timeout=10)
                self.sent += 1
                self.airtime_us += airtime
            except mc.ModemError:
                time.sleep(0.05)
            seq += 1

    def duty(self):
        return self.airtime_us / 1e6 / max(time.time() - self.started, 1e-6)


def run_cads(m, count):
    busy = fails = 0
    for _ in range(count):
        try:
            busy += m.cad()
        except mc.ModemError:
            fails += 1
        time.sleep(0.03 + random.random() * 0.05)
    return busy, fails


def main():
    ap = argparse.ArgumentParser(description="CAD busy/clear test")
    ap.add_argument("--tx", required=True)
    ap.add_argument("--cad", required=True)
    ap.add_argument("--preset", choices=sorted(mc.PRESETS), required=True)
    ap.add_argument("--sf-list", default="7,9,12", type=lambda s: [int(x) for x in s.split(",")])
    ap.add_argument("--sym", default="1,2", type=lambda s: [int(x) for x in s.split(",")],
                    help="CAD symbol codes (0-4 = 1/2/4/8/16 symbols)")
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--len", type=int, default=200)
    ap.add_argument("--tx-power", type=int, default=mc.BENCH_POWER_DBM)
    ap.add_argument("--save")
    args = ap.parse_args()
    sys.stdout.reconfigure(errors="replace")

    tx = mc.Modem(args.tx)
    cad = mc.Modem(args.cad)
    results = []
    try:
        print(f"TX  {tx.name}  {tx.radio_info()['chip']}")
        print(f"CAD {cad.name}  {cad.radio_info()['chip']}")
        for sf in args.sf_list:
            base = dict(mc.PRESETS[args.preset], sf=sf)
            tx.set_config(dict(base, power_dbm=args.tx_power))
            cad.set_config(base)
            print(f"\n{mc.fmt_config(dict(base, power_dbm=args.tx_power))}")
            for code in args.sym:
                applied = cad.set_cad_params(code, CORE_PEAK.get(sf, 22), 10)
                quiet_busy, quiet_fail = run_cads(cad, args.count)
                jam = Jammer(tx, args.len)
                jam.start()
                time.sleep(0.5)
                busy, fail = run_cads(cad, args.count)
                jam.stop_flag.set()
                jam.join(timeout=15)
                duty = jam.duty()
                time.sleep(0.3)
                r = dict(sf=sf, sym_code=code, symbols=SYMBOLS.get(code), applied=list(applied),
                         quiet_busy=quiet_busy, quiet_fail=quiet_fail, busy=busy, fail=fail,
                         count=args.count, tx_packets=jam.sent, tx_duty=round(duty, 2))
                results.append(r)
                print(f"  {SYMBOLS.get(code, code):>2} sym (applied {list(applied)}): "
                      f"quiet {quiet_busy}/{args.count} busy, "
                      f"TX on {busy}/{args.count} busy ({100 * busy / args.count:.0f} %, "
                      f"TX duty {100 * duty:.0f} %)"
                      + (f"  CAD failures {quiet_fail + fail}" if quiet_fail + fail else ""))
        if args.save:
            with open(args.save, "w", encoding="utf-8") as f:
                json.dump(dict(tx=tx.name, cad=cad.name, preset=args.preset, results=results,
                               timestamp=time.strftime("%Y-%m-%dT%H:%M:%S")), f, indent=2)
            print(f"Saved {args.save}")
    finally:
        tx.close()
        cad.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
