#!/usr/bin/env python3
"""hp_check.py — high-power PA path check into a dummy load or attenuator.

Run only with the transmitting board on a dummy load / attenuator.

Sends a few packets at bench power, then at the board's normal drive,
while a partner board listens on the same preset at bench power. The RSSI
step at the partner (leakage through the load is enough) shows whether the
high-power PA path and RF switch states work; a power meter or SDR on the
load side is the real measurement.

Example:
  python hp_check.py --tx COM11 --rx COM19 --preset subg --power 22
  python hp_check.py --tx COM9 --rx COM11 --preset 2g4 --power 0 --bench-power -18
"""

import argparse
import json
import statistics
import struct
import sys
import time

import modem_check as mc


def send_and_measure(tx, rx, cfg, count, length=16, gap=1.0):
    while not rx.rx_packets.empty():
        rx.rx_packets.get_nowait()
    expected = mc.lora_time_on_air_us(cfg, length)
    rssi, snr, airtimes, fails = [], [], [], 0
    for seq in range(count):
        try:
            airtime, _, _ = tx.transmit_lbt(mc.test_payload(seq, length))
            airtimes.append(airtime)
        except mc.ModemError:
            fails += 1
            continue
        deadline = time.time() + expected / 1e6 + 0.5
        while time.time() < deadline:
            try:
                _, p = rx.rx_packets.get(timeout=0.05)
            except Exception:
                continue
            r = mc.decode_rx(p)
            if r["data"].startswith(mc.TEST_MAGIC) and struct.unpack("<H", r["data"][3:5])[0] == seq:
                rssi.append(r["rssi"])
                snr.append(r["snr"])
                break
        time.sleep(gap)
    return dict(sent=count - fails, tx_fail=fails, received=len(rssi),
                rssi_avg=round(statistics.mean(rssi), 1) if rssi else None,
                snr_avg=round(statistics.mean(snr), 1) if snr else None,
                airtime_ok=all(abs(a - expected) / expected < 0.03 for a in airtimes) if airtimes else False)


def main():
    ap = argparse.ArgumentParser(description="High-power PA path check (dummy load required)")
    ap.add_argument("--tx", required=True)
    ap.add_argument("--rx", required=True)
    ap.add_argument("--preset", choices=sorted(mc.PRESETS), required=True)
    ap.add_argument("--power", type=int, required=True, help="normal drive level to check")
    ap.add_argument("--bench-power", type=int, default=mc.BENCH_POWER_DBM)
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--save")
    args = ap.parse_args()
    sys.stdout.reconfigure(errors="replace")

    tx = mc.Modem(args.tx)
    rx = mc.Modem(args.rx)
    ok = True
    try:
        bench = dict(mc.PRESETS[args.preset], power_dbm=args.bench_power)
        high = dict(mc.PRESETS[args.preset], power_dbm=args.power)
        rx.set_config(bench)
        tx.set_config(bench)
        print(f"TX {tx.name} ({tx.version()}) -> RX {rx.name} ({rx.version()})")
        print(f"Preset {mc.fmt_config(bench)}")

        low = send_and_measure(tx, rx, bench, 3)
        print(f"  bench {args.bench_power:>3} dBm: {low['received']}/{low['sent']} received, "
              f"RSSI {low['rssi_avg']} dBm, SNR {low['snr_avg']} dB")

        tx.set_config(high)
        applied = tx.radio_info()["applied_power_dbm"]
        clamp_ok = applied == args.power
        ok &= clamp_ok
        print(f"  [{'PASS' if clamp_ok else 'FAIL'}] applied power {applied} dBm (requested {args.power})")

        hi = send_and_measure(tx, rx, high, args.count)
        print(f"  high  {applied:>3} dBm: {hi['received']}/{hi['sent']} received, "
              f"RSSI {hi['rssi_avg']} dBm, SNR {hi['snr_avg']} dB")
        tx_ok = hi["tx_fail"] == 0 and hi["airtime_ok"]
        ok &= tx_ok
        print(f"  [{'PASS' if tx_ok else 'FAIL'}] all TX_DONE, airtime as expected")
        rx_ok = hi["received"] > 0
        ok &= rx_ok
        print(f"  [{'PASS' if rx_ok else 'FAIL'}] partner received high-power packets")
        if low["rssi_avg"] is not None and hi["rssi_avg"] is not None:
            step = hi["rssi_avg"] - low["rssi_avg"]
            want = applied - args.bench_power
            print(f"  RSSI step {step:+.1f} dB for a {want} dB drive step "
                  f"(receiver compression can make the step smaller)")
        result = dict(tx=tx.name, rx=rx.name, preset=args.preset, bench=low, high=hi,
                      applied_power_dbm=applied, timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"))
        if args.save:
            with open(args.save, "w", encoding="utf-8") as f:
                json.dump(result, f, indent=2)
    finally:
        try:
            tx.set_config(dict(mc.PRESETS[args.preset], power_dbm=args.bench_power))
            print(f"  restored TX to bench power {args.bench_power} dBm")
        finally:
            tx.close()
            rx.close()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
