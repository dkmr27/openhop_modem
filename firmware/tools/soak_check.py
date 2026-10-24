#!/usr/bin/env python3
"""soak_check.py — multi-board soak test.

All boards take turns transmitting; every other board on the same preset
listens. With several presets the run rotates through them in blocks, so
dual-band chips also get band changes under load; a board that has no band
for the current preset sits that block out. Every --poll seconds each board
is checked for resets (uptime going backwards), heap, CRC errors and config
drift.

Example:
  python soak_check.py COM19 COM11 COM3 COM9 --preset subg,2g4 --minutes 60 \
      --block 10 --power COM9:2g4=-18 --save baselines/soak.json
"""

import argparse
import json
import queue
import random
import statistics
import struct
import sys
import time

import modem_check as mc


def supports(info, cfg):
    return any(b["freq_min_hz"] <= cfg["freq_hz"] <= b["freq_max_hz"] for b in info["bands"])


def soak_payload(sender, seq, length):
    head = mc.TEST_MAGIC + struct.pack("<HB", seq, sender)
    return head + random.randbytes(max(length - len(head), 0))


def parse_power(specs):
    """["COM9:2g4=-18", ...] -> {("COM9", "2g4"): -18}"""
    out = {}
    for spec in specs or []:
        port, _, rest = spec.partition(":")
        preset, _, dbm = rest.partition("=")
        out[(port, preset)] = int(dbm)
    return out


class Board:
    def __init__(self, idx, spec, args):
        self.idx = idx
        self.m = mc.Modem(spec, args.verbose, args.dtr)
        self.name = self.m.name
        self.version = self.m.version()
        self.info = self.m.radio_info()
        self.cfg = None
        self.last_uptime = None
        self.resets = 0
        self.min_free_heap = None
        self.crc_start = None
        self.crc_errors = 0
        self.config_drift = 0
        self.poll_errors = 0

    def poll(self):
        try:
            st = self.m.status()
            dbg = self.m.debug()
            cur = self.m.get_config()
        except mc.ModemError:
            self.poll_errors += 1
            return
        if self.last_uptime is not None and st["uptime_sec"] < self.last_uptime:
            self.resets += 1
            print(f"  !! {self.name} reset (uptime {self.last_uptime} -> {st['uptime_sec']} s, "
                  f"reason {dbg['reset_reason']})")
            self.crc_start = None
        self.last_uptime = st["uptime_sec"]
        if self.crc_start is None:
            self.crc_start = st["crc_errors"] - self.crc_errors
        self.crc_errors = st["crc_errors"] - self.crc_start
        heap = dbg["min_free_heap"]
        self.min_free_heap = heap if self.min_free_heap is None else min(self.min_free_heap, heap)
        if self.cfg and cur != self.cfg:
            self.config_drift += 1
            print(f"  !! {self.name} config drift: {mc.fmt_config(cur)}")


def drain(boards, sent, links, rogue, until):
    """Collect RX frames from every board until `until`."""
    while time.time() < until:
        idle = True
        for rx in boards:
            try:
                _, p = rx.m.rx_packets.get_nowait()
            except queue.Empty:
                continue
            idle = False
            r = mc.decode_rx(p)
            d = r["data"]
            if not d.startswith(mc.TEST_MAGIC) or len(d) < 6:
                rogue[rx.name] = rogue.get(rx.name, 0) + 1
                continue
            seq, sender = struct.unpack("<HB", d[3:6])
            key = (sender, seq)
            link = links.get((sender, rx.idx))
            if link is None or key not in sent:
                rogue[rx.name] = rogue.get(rx.name, 0) + 1
                continue
            if sent[key][0] != d:
                link["corrupt"] += 1
            elif rx.idx in sent[key][1]:
                link["dup"] += 1
            else:
                sent[key][1].add(rx.idx)
                link["received"] += 1
                link["rssi"].append(r["rssi"])
                link["snr"].append(r["snr"])
        if idle:
            time.sleep(0.01)


def main():
    ap = argparse.ArgumentParser(description="Multi-board soak test")
    ap.add_argument("ports", nargs="+")
    ap.add_argument("--preset", default="subg", help="comma list of presets to rotate through")
    ap.add_argument("--minutes", type=float, default=30)
    ap.add_argument("--block", type=float, default=10, help="minutes per preset block")
    ap.add_argument("--power", action="append", help="per-board power, PORT:PRESET=DBM")
    ap.add_argument("--len", type=int, default=32)
    ap.add_argument("--gap", type=float, default=0.2, help="extra seconds between packets (plus jitter)")
    ap.add_argument("--poll", type=float, default=60, help="health poll interval, seconds")
    ap.add_argument("--save")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--dtr", choices=("auto", "on", "off"), default="auto")
    args = ap.parse_args()
    sys.stdout.reconfigure(errors="replace")

    presets = args.preset.split(",")
    powers = parse_power(args.power)
    boards = [Board(i, p, args) for i, p in enumerate(args.ports)]
    for b in boards:
        print(f"[{b.idx}] {b.name}  {b.info['chip']}  {b.version}")

    links = {}                   # (tx idx, rx idx) -> stats
    tx_stats = {b.idx: dict(sent=0, tx_fail=0, busy_retries=0, errors={}) for b in boards}
    sent = {}                    # (tx idx, seq) -> (payload, set of rx idx)
    rogue = {}
    blocks = []
    seq = 0
    start = time.time()
    end = start + args.minutes * 60
    next_poll = start
    block_no = 0
    try:
        while time.time() < end:
            name = presets[block_no % len(presets)]
            block_no += 1
            active = []
            for b in boards:
                cfg = dict(mc.PRESETS[name], power_dbm=powers.get((b.name, name), mc.BENCH_POWER_DBM))
                if supports(b.info, cfg):
                    b.cfg = b.m.set_config(cfg)
                    active.append(b)
            print(f"\n{time.strftime('%H:%M:%S')}  block {block_no}: {name}  "
                  f"boards {[b.idx for b in active]}  {mc.fmt_config(mc.PRESETS[name])}")
            blocks.append(dict(preset=name, boards=[b.idx for b in active], start=time.time() - start))
            if len(active) < 2:
                continue
            for t in active:
                for r in active:
                    if t is not r:
                        links.setdefault((t.idx, r.idx), dict(sent=0, received=0, dup=0, corrupt=0,
                                                               rssi=[], snr=[]))
            toa = mc.lora_time_on_air_us(active[0].cfg, args.len) / 1e6
            block_end = min(end, time.time() + args.block * 60)
            turn = 0
            while time.time() < block_end:
                tx = active[turn % len(active)]
                turn += 1
                seq = (seq + 1) & 0xFFFF
                payload = soak_payload(tx.idx, seq, args.len)
                st = tx_stats[tx.idx]
                try:
                    _, _, busy = tx.m.transmit_lbt(payload)
                    st["busy_retries"] += busy
                except mc.ModemError as exc:
                    st["tx_fail"] += 1
                    st["errors"][str(exc)] = st["errors"].get(str(exc), 0) + 1
                    continue
                st["sent"] += 1
                sent[(tx.idx, seq)] = (payload, set())
                for r in active:
                    if r is not tx:
                        links[(tx.idx, r.idx)]["sent"] += 1
                drain(boards, sent, links, rogue, time.time() + toa + 0.15 + args.gap + random.random() * 0.2)
                if time.time() >= next_poll:
                    for b in boards:
                        b.poll()
                    next_poll = time.time() + args.poll
                    total_s = sum(l["sent"] for l in links.values())
                    total_r = sum(l["received"] for l in links.values())
                    print(f"  {time.strftime('%H:%M:%S')}  {(time.time() - start) / 60:5.1f} min  "
                          f"{sum(s['sent'] for s in tx_stats.values())} TX  "
                          f"{total_r}/{total_s} link deliveries")
            drain(boards, sent, links, rogue, time.time() + 0.5)
        for b in boards:
            b.poll()
    except KeyboardInterrupt:
        print("\nInterrupted; reporting so far")
    finally:
        for b in boards:
            try:
                b.m.set_config(dict(mc.PRESETS[presets[0]], power_dbm=mc.BENCH_POWER_DBM)
                               if supports(b.info, mc.PRESETS[presets[0]]) else b.cfg or b.m.get_config())
            except mc.ModemError:
                pass

    ok = True
    print(f"\nSoak {(time.time() - start) / 60:.1f} min, {len(blocks)} block(s)")
    print("Links (TX -> RX):")
    link_out = []
    for (t, r), l in sorted(links.items()):
        loss = l["sent"] - l["received"]
        good = l["sent"] > 0 and loss <= max(1, l["sent"] // 200) and l["corrupt"] == 0
        ok &= good
        line = (f"  [{'PASS' if good else 'FAIL'}] {t} -> {r}: {l['received']}/{l['sent']}"
                f"  ({100 * l['received'] / l['sent']:.2f} %)" if l["sent"] else f"  [FAIL] {t} -> {r}: nothing sent")
        if l["rssi"]:
            line += (f"  RSSI {min(l['rssi'])}/{statistics.mean(l['rssi']):.1f}/{max(l['rssi'])}"
                     f"  SNR {min(l['snr'])}/{statistics.mean(l['snr']):.1f}/{max(l['snr'])}")
        if l["dup"] or l["corrupt"]:
            line += f"  dup {l['dup']} corrupt {l['corrupt']}"
        print(line)
        link_out.append(dict(tx=t, rx=r, sent=l["sent"], received=l["received"], dup=l["dup"],
                             corrupt=l["corrupt"],
                             rssi_avg=round(statistics.mean(l["rssi"]), 1) if l["rssi"] else None,
                             snr_avg=round(statistics.mean(l["snr"]), 1) if l["snr"] else None))
    print("Boards:")
    board_out = []
    for b in boards:
        st = tx_stats[b.idx]
        good = b.resets == 0 and b.config_drift == 0 and b.poll_errors == 0 and st["tx_fail"] == 0
        ok &= good
        print(f"  [{'PASS' if good else 'FAIL'}] [{b.idx}] {b.name} {b.info['chip']}: "
              f"{st['sent']} TX, {st['tx_fail']} TX fail {st['errors'] or ''}, busy retries {st['busy_retries']}, "
              f"resets {b.resets}, config drift {b.config_drift}, poll errors {b.poll_errors}, "
              f"CRC errors {b.crc_errors}, min free heap {b.min_free_heap}, uptime {b.last_uptime} s")
        board_out.append(dict(idx=b.idx, port=b.name, chip=b.info["chip"], version=b.version, **st,
                              resets=b.resets, config_drift=b.config_drift, poll_errors=b.poll_errors,
                              crc_errors=b.crc_errors, min_free_heap=b.min_free_heap, uptime_s=b.last_uptime))
    if rogue:
        print(f"Other packets heard: {rogue}")
    if args.save:
        with open(args.save, "w", encoding="utf-8") as f:
            json.dump(dict(presets=presets, minutes=round((time.time() - start) / 60, 1), blocks=blocks,
                           links=link_out, boards=board_out, other_packets=rogue,
                           timestamp=time.strftime("%Y-%m-%dT%H:%M:%S")), f, indent=2)
        print(f"Saved {args.save}")
    for b in boards:
        b.m.close()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
