#!/usr/bin/env python3
"""modem_check.py — bench check tool for openHop Modem firmware.

Speaks the binary host protocol from firmware/include/protocol.h over USB
serial or TCP, so a board can be configured, made to transmit/receive and
checked without openHop Core. Debug text the firmware prints on the same
USB port is separated from protocol frames and kept for the report.

Examples:
  python modem_check.py --port COM19 info
  python modem_check.py --port COM19 config --preset subg
  python modem_check.py --port COM19 tx --count 5
  python modem_check.py --port COM19 rx --seconds 30
  python modem_check.py --port COM19 smoke --save baseline_heltec_v3.json
  python modem_check.py --port COM19 smoke --compare baseline_heltec_v3.json
  python modem_check.py pair --a COM19 --b COM7 --preset subg --count 20

Device specs for --port / --a / --b: "COM19", "/dev/ttyACM0", or
"tcp:HOST[:PORT[:TOKEN]]" (default port 5055).

TX power defaults to the bench level of -9 dBm. Anything higher has to be
asked for explicitly with --power.

Requires: pyserial (pip install pyserial).
"""

import argparse
import json
import math
import os
import queue
import socket
import statistics
import struct
import sys
import threading
import time

# ─── Protocol constants (keep in sync with include/protocol.h) ──────────
PROTO_SYNC = 0xAA

CMD_TX_REQUEST = 0x01
CMD_TX_DONE = 0x02
CMD_TX_FAIL = 0x03
CMD_RX_PACKET = 0x04
CMD_SET_CONFIG = 0x10
CMD_GET_CONFIG = 0x11
CMD_CONFIG_RESP = 0x12
CMD_GET_RADIO_INFO = 0x14
CMD_RADIO_INFO_RESP = 0x15
CMD_STATUS_REQ = 0x20
CMD_STATUS_RESP = 0x21
CMD_NOISE_REQ = 0x22
CMD_NOISE_RESP = 0x23
CMD_CAD_REQUEST = 0x30
CMD_RX_START = 0x31
CMD_CAD_RESP = 0x32
CMD_RX_STARTED = 0x33
CMD_RADIO_STANDBY = 0x40
CMD_RADIO_RESUME = 0x42
CMD_RADIO_STANDBY_RESP = 0x44
CMD_RADIO_RESUME_RESP = 0x46
CMD_SET_AUTO_CAD = 0x4A
CMD_SET_AUTO_CAD_RESP = 0x4B
CMD_AUTH = 0x50
CMD_AUTH_OK = 0x51
CMD_GET_VERSION = 0x70
CMD_VERSION_RESP = 0x71
CMD_GET_DEBUG = 0x72
CMD_DEBUG_RESP = 0x73
CMD_LOG_MSG = 0x80
CMD_ERROR = 0xFE
CMD_PING = 0xFF
CMD_PONG = 0xFF

ERRORS = {
    0x01: "CRC_MISMATCH", 0x02: "INVALID_CMD", 0x03: "RADIO_BUSY",
    0x04: "TX_TIMEOUT", 0x05: "PAYLOAD_TOO_BIG", 0x06: "INVALID_CONFIG",
    0x07: "CAD_FAILED", 0x08: "RADIO_INIT", 0x09: "UNAUTHORIZED",
    0x0A: "INVALID_WIFI", 0x0B: "NO_RADIO", 0x0C: "OTA_UNSUPPORTED",
    0x0D: "OTA_NO_BUFFER", 0x0E: "CHANNEL_BUSY",
}

RADIO_CONFIG = struct.Struct("<IIBBbHB")      # RadioConfig, 14 bytes
STATUS_RESP = struct.Struct("<IIIIhhhbBH")    # StatusResp, 26 bytes
DEBUG_RESP = struct.Struct("<BIIII")          # CMD_DEBUG_RESP, 17 bytes

MAX_LORA_PAYLOAD = 255
CHIP_NAMES = {1: "SX1262", 2: "LR1121", 3: "SX1280", 4: "LR2021"}
BAND_NAMES = {0: "subg", 1: "2g4"}
FEATURE_NAMES = ("CAD", "CAD tuning", "RX guard", "AGC reset")
BENCH_POWER_DBM = -9
TEST_MAGIC = b"OHT"                            # marks our own test packets

# Test presets, chosen to stay away from the main mesh (see the HAL plan).
PRESETS = {
    "subg": dict(freq_hz=927_925_000, bandwidth_hz=125_000, sf=9, cr=5,
                 power_dbm=BENCH_POWER_DBM, syncword=0x12, preamble_len=16),
    "2g4": dict(freq_hz=2_400_500_000, bandwidth_hz=406_250, sf=11, cr=5,
                power_dbm=BENCH_POWER_DBM, syncword=0x12, preamble_len=16),
}


def crc16_ccitt(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_frame(cmd, payload=b""):
    body = bytes([cmd]) + struct.pack("<H", len(payload)) + payload
    return bytes([PROTO_SYNC]) + body + struct.pack("<H", crc16_ccitt(body))


def lora_time_on_air_us(cfg, payload_len):
    """Semtech LoRa time on air: explicit header, CRC on, auto LDRO.

    LDRO follows RadioLib's auto rule: on when a symbol lasts 16 ms or more,
    and at 2.4 GHz also for SF > 10 (LR11x0 does this to match the SX128x).
    """
    sf, bw = cfg["sf"], cfg["bandwidth_hz"]
    t_sym = (2 ** sf) / bw
    de = 1 if t_sym >= 0.016 or (cfg["freq_hz"] >= 1_000_000_000 and sf > 10) else 0
    n_pre = cfg["preamble_len"] + (6.25 if sf < 7 else 4.25)
    num = 8 * payload_len - 4 * sf + 28 + 16 - 0
    if sf < 7:
        num += 8
    den = 4 * (sf - 2 * de)
    n_payload = 8 + max(math.ceil(num / den) * cfg["cr"], 0)
    return int((n_pre + n_payload) * t_sym * 1e6)


# ─── Transports ──────────────────────────────────────────────────────────
# USB VIDs whose DTR/RTS lines can reset the board: USB-UART bridges wired
# to ESP32 EN/BOOT (CP210x, CH34x, FTDI, Prolific) and ESP32 native USB.
RESET_ON_DTR_VIDS = {0x10C4, 0x1A86, 0x0403, 0x067B, 0x303A}


def want_dtr(port, mode):
    """DTR high for native-USB boards (nRF52 TinyUSB only sends once the
    host raises DTR); low where DTR/RTS are wired to reset the chip."""
    if mode != "auto":
        return mode == "on"
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.device == port and p.vid is not None:
                return p.vid not in RESET_ON_DTR_VIDS
    except Exception:
        pass
    return False


class SerialTransport:
    def __init__(self, port, baud=921600, dtr="auto"):
        import serial  # imported here so --help works without pyserial
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.05
        # RTS stays low so opening the port doesn't reset ESP32 boards.
        self.ser.dtr = want_dtr(port, dtr)
        self.ser.rts = False
        self.ser.open()
        self.name = port

    def read(self):
        return self.ser.read(self.ser.in_waiting or 1)

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class TcpTransport:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=5)
        self.sock.settimeout(0.05)
        self.name = f"tcp:{host}:{port}"

    def read(self):
        try:
            data = self.sock.recv(4096)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionError("TCP connection closed by modem")
        return data

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()


class ModemError(Exception):
    pass


class Modem:
    """One connected modem. A reader thread splits frames from debug text."""

    def __init__(self, spec, verbose=False, dtr="auto"):
        self.verbose = verbose
        token = None
        if spec.startswith("tcp:"):
            parts = spec.split(":")
            host = parts[1]
            port = int(parts[2]) if len(parts) > 2 and parts[2] else 5055
            token = parts[3] if len(parts) > 3 else None
            self.transport = TcpTransport(host, port)
        else:
            self.transport = SerialTransport(spec, dtr=dtr)
        self.name = self.transport.name
        self.responses = queue.Queue()
        self.rx_packets = queue.Queue()
        self.debug_lines = []
        self._text = bytearray()
        self._buf = bytearray()
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self._wait_ready()
        if token:
            self.request(CMD_AUTH, token.encode(), {CMD_AUTH_OK})

    # -- reader --------------------------------------------------------
    def _reader(self):
        while not self._stop:
            try:
                data = self.transport.read()
            except Exception as exc:  # port unplugged, TCP closed, ...
                self._note_text(f"<reader stopped: {exc}>\n".encode())
                return
            if data:
                self._buf.extend(data)
                self._parse()

    def _parse(self):
        buf = self._buf
        while buf:
            if buf[0] != PROTO_SYNC:
                i = buf.find(PROTO_SYNC)
                cut = len(buf) if i < 0 else i
                self._note_text(bytes(buf[:cut]))
                del buf[:cut]
                continue
            if len(buf) < 4:
                return
            length = buf[2] | (buf[3] << 8)
            if length > 512:                 # not a real frame header
                self._note_text(bytes(buf[:1]))
                del buf[:1]
                continue
            total = 4 + length + 2
            if len(buf) < total:
                return
            body = bytes(buf[1:4 + length])
            crc = buf[4 + length] | (buf[5 + length] << 8)
            if crc16_ccitt(body) != crc:     # 0xAA inside debug text
                self._note_text(bytes(buf[:1]))
                del buf[:1]
                continue
            cmd, payload = body[0], body[3:]
            del buf[:total]
            self._on_frame(cmd, payload)

    def _note_text(self, data):
        self._text.extend(data)
        while b"\n" in self._text:
            line, _, rest = self._text.partition(b"\n")
            self._text = bytearray(rest)
            text = line.decode("utf-8", "replace").rstrip("\r")
            if text:
                self.debug_lines.append(text)
                if self.verbose:
                    print(f"  [{self.name} dbg] {text}")

    def _on_frame(self, cmd, payload):
        if cmd == CMD_RX_PACKET:
            self.rx_packets.put((time.time(), payload))
        elif cmd == CMD_LOG_MSG:
            level = {0: "INFO", 1: "WARN", 2: "ERR"}.get(payload[0], "?")
            self._note_text(f"[{level}] ".encode() + payload[1:] + b"\n")
        else:
            self.responses.put((cmd, payload))

    # -- requests ------------------------------------------------------
    def _wait_ready(self, timeout=6.0):
        """Ping until PONG; covers boards that reset when the port opens."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                self.request(CMD_PING, b"", {CMD_PONG}, timeout=0.5)
                time.sleep(0.2)
                self._drain()
                return
            except ModemError:
                pass
        raise ModemError(f"{self.name}: no PONG within {timeout:.0f} s")

    def _drain(self):
        while not self.responses.empty():
            self.responses.get_nowait()

    def request(self, cmd, payload=b"", expect=(), timeout=2.0):
        """Send a frame; return (cmd, payload) of the first expected reply.

        CMD_ERROR replies raise ModemError with the decoded error name.
        """
        self._drain()
        self.transport.write(build_frame(cmd, payload))
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                raise ModemError(f"{self.name}: timeout waiting for reply to 0x{cmd:02X}")
            try:
                rcmd, rpayload = self.responses.get(timeout=left)
            except queue.Empty:
                continue
            if rcmd == CMD_ERROR and rpayload:
                code = rpayload[0]
                raise ModemError(f"ERR_{ERRORS.get(code, hex(code))}")
            if rcmd in expect:
                return rcmd, rpayload

    def close(self):
        self._stop = True
        self._thread.join(timeout=1)
        self.transport.close()

    # -- typed helpers -------------------------------------------------
    def version(self):
        return self.request(CMD_GET_VERSION, b"", {CMD_VERSION_RESP})[1].decode(errors="replace")

    def get_config(self):
        return unpack_config(self.request(CMD_GET_CONFIG, b"", {CMD_CONFIG_RESP})[1])

    def set_config(self, cfg):
        payload = RADIO_CONFIG.pack(cfg["freq_hz"], cfg["bandwidth_hz"], cfg["sf"],
                                    cfg["cr"], cfg["power_dbm"], cfg["syncword"],
                                    cfg["preamble_len"])
        return unpack_config(self.request(CMD_SET_CONFIG, payload, {CMD_CONFIG_RESP})[1])

    def status(self):
        f = STATUS_RESP.unpack(self.request(CMD_STATUS_REQ, b"", {CMD_STATUS_RESP})[1][:STATUS_RESP.size])
        keys = ("uptime_sec", "rx_count", "tx_count", "crc_errors", "last_rssi",
                "last_snr_x10", "noise_floor_x10", "temp_c", "radio_state", "battery_mv")
        return dict(zip(keys, f))

    def noise_floor(self):
        p = self.request(CMD_NOISE_REQ, b"", {CMD_NOISE_RESP})[1]
        return struct.unpack("<h", p[:2])[0] / 10.0

    def debug(self):
        f = DEBUG_RESP.unpack(self.request(CMD_GET_DEBUG, b"", {CMD_DEBUG_RESP})[1][:DEBUG_RESP.size])
        return dict(zip(("reset_reason", "uptime_ms", "free_heap", "min_free_heap", "last_loop_us"), f))

    def transmit(self, data, timeout=6.0):
        """Returns (airtime_us reported by the modem, wall-clock seconds)."""
        t0 = time.time()
        _, p = self.request(CMD_TX_REQUEST, data, {CMD_TX_DONE, CMD_TX_FAIL}, timeout=timeout)
        if len(p) < 4:
            raise ModemError("TX_FAIL")
        return struct.unpack("<I", p[:4])[0], time.time() - t0

    def standby(self):
        return self.request(CMD_RADIO_STANDBY, b"", {CMD_RADIO_STANDBY_RESP})[1][0] == 0

    def resume(self):
        return self.request(CMD_RADIO_RESUME, b"", {CMD_RADIO_RESUME_RESP}, timeout=3)[1][0] == 0

    def radio_info(self):
        """CMD_GET_RADIO_INFO (protocol v0.8+), decoded into a dict."""
        return decode_radio_info(self.request(CMD_GET_RADIO_INFO, b"", {CMD_RADIO_INFO_RESP})[1])

    def cad(self):
        """Channel activity detection: True = busy, False = clear."""
        return self.request(CMD_CAD_REQUEST, b"", {CMD_CAD_RESP}, timeout=2)[1][0] != 0

    def set_auto_cad(self, on):
        return self.request(CMD_SET_AUTO_CAD, bytes([1 if on else 0]), {CMD_SET_AUTO_CAD_RESP})[1][0] == 0

    def rx_start(self):
        self.request(CMD_RX_START, b"", {CMD_RX_STARTED})


def unpack_config(p):
    f = RADIO_CONFIG.unpack(p[:RADIO_CONFIG.size])
    return dict(zip(("freq_hz", "bandwidth_hz", "sf", "cr", "power_dbm", "syncword", "preamble_len"), f))


def decode_radio_info(p):
    ver, chip, feat, cur_band, applied, nbands = struct.unpack("<BBBBbB", p[:6])
    i = 6
    bands = []
    for _ in range(nbands):
        band, fmin, fmax, pmin, pmax, sfmin, sfmax, crmin, crmax, nbw = struct.unpack("<BIIbbBBBBB", p[i:i + 16])
        i += 16
        bws = list(struct.unpack(f"<{nbw}I", p[i:i + 4 * nbw]))
        i += 4 * nbw
        bands.append(dict(band=BAND_NAMES.get(band, band), freq_min_hz=fmin, freq_max_hz=fmax,
                          power_min_dbm=pmin, power_max_dbm=pmax, sf=[sfmin, sfmax],
                          cr=[crmin, crmax], bandwidths_hz=bws))
    name = p[i + 1:i + 1 + p[i]].decode(errors="replace")
    return dict(version=ver, chip_id=chip, chip=CHIP_NAMES.get(chip, name or str(chip)), chip_name=name,
                features=[n for b, n in enumerate(FEATURE_NAMES) if feat & (1 << b)],
                current_band=BAND_NAMES.get(cur_band, cur_band), applied_power_dbm=applied, bands=bands)


def fmt_config(c):
    return (f"{c['freq_hz'] / 1e6:.4f} MHz  BW {c['bandwidth_hz'] / 1e3:g} kHz  SF{c['sf']}  "
            f"CR 4/{c['cr']}  {c['power_dbm']} dBm  sync 0x{c['syncword']:02X}  pre {c['preamble_len']}")


def decode_rx(payload):
    rssi, snr_x10, sig_rssi = struct.unpack("<hhh", payload[:6])
    return dict(rssi=rssi, snr=snr_x10 / 10.0, signal_rssi=sig_rssi, data=payload[6:])


def config_from_args(args):
    cfg = dict(PRESETS[args.preset])
    for key, attr, scale in (("freq_hz", "freq", 1e6), ("bandwidth_hz", "bw", 1e3)):
        val = getattr(args, attr, None)
        if val is not None:
            cfg[key] = int(round(val * scale))
    for key in ("sf", "cr", "power", "syncword", "preamble"):
        val = getattr(args, key, None)
        if val is not None:
            cfg[{"power": "power_dbm", "preamble": "preamble_len"}.get(key, key)] = val
    return cfg


def test_payload(seq, length):
    length = max(length, len(TEST_MAGIC) + 2)
    return TEST_MAGIC + struct.pack("<H", seq) + os.urandom(length - len(TEST_MAGIC) - 2)


# ─── Report helper ───────────────────────────────────────────────────────
class Report:
    def __init__(self):
        self.checks = []
        self.data = {}

    def check(self, name, ok, detail=""):
        self.checks.append(dict(name=name, ok=bool(ok), detail=detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" - {detail}" if detail else ""))
        return ok

    @property
    def passed(self):
        return all(c["ok"] for c in self.checks)


# ─── Commands ────────────────────────────────────────────────────────────
def cmd_info(m, args):
    print(f"Device:   {m.name}")
    print(f"Firmware: {m.version()}")
    try:
        print(f"Config:   {fmt_config(m.get_config())}")
        st = m.status()
        print(f"Status:   up {st['uptime_sec']} s  rx {st['rx_count']}  tx {st['tx_count']}  "
              f"crc_err {st['crc_errors']}  last RSSI {st['last_rssi']} dBm  "
              f"SNR {st['last_snr_x10'] / 10:g} dB  temp {st['temp_c']} C")
        print(f"Noise:    {m.noise_floor():.1f} dBm")
    except ModemError as exc:
        print(f"Radio:    {exc}")
    try:
        ri = m.radio_info()
        print(f"Chip:     {ri['chip']}  features: {', '.join(ri['features']) or 'none'}  "
              f"current band {ri['current_band']}  applied power {ri['applied_power_dbm']} dBm")
        for b in ri["bands"]:
            print(f"  Band {b['band']}: {b['freq_min_hz'] / 1e6:g}-{b['freq_max_hz'] / 1e6:g} MHz  "
                  f"power {b['power_min_dbm']}..{b['power_max_dbm']} dBm  SF{b['sf'][0]}-{b['sf'][1]}  "
                  f"CR 4/{b['cr'][0]}-4/{b['cr'][1]}  BW {', '.join(f'{x / 1e3:g}' for x in b['bandwidths_hz'])} kHz")
    except ModemError as exc:
        print(f"Chip:     radio info not available ({exc})")
    d = m.debug()
    print(f"Debug:    reset_reason {d['reset_reason']}  heap {d['free_heap']} (min {d['min_free_heap']})  "
          f"last loop {d['last_loop_us']} us")
    return 0


def cmd_config(m, args):
    cfg = config_from_args(args)
    print(f"Applying: {fmt_config(cfg)}")
    back = m.set_config(cfg)
    print(f"Modem:    {fmt_config(back)}")
    return 0


def cmd_tx(m, args):
    cfg = config_from_args(args)
    if not args.no_config:
        m.set_config(cfg)
        print(f"Config:   {fmt_config(cfg)}")
    expected = lora_time_on_air_us(cfg, args.len)
    for seq in range(args.count):
        airtime, wall = m.transmit(test_payload(seq, args.len))
        err = (airtime - expected) / expected * 100
        print(f"  TX #{seq}: {args.len} B  modem airtime {airtime} us  "
              f"(calc {expected} us, {err:+.1f}%)  round trip {wall * 1000:.0f} ms")
        time.sleep(args.gap)
    return 0


def cmd_rx(m, args):
    if not args.no_config:
        cfg = config_from_args(args)
        m.set_config(cfg)
        print(f"Config:   {fmt_config(cfg)}")
    print(f"Listening for {args.seconds} s ...")
    deadline = time.time() + args.seconds
    n = 0
    while time.time() < deadline:
        try:
            ts, p = m.rx_packets.get(timeout=0.2)
        except queue.Empty:
            continue
        r = decode_rx(p)
        n += 1
        tag = "test" if r["data"].startswith(TEST_MAGIC) else "other"
        seq = struct.unpack("<H", r["data"][3:5])[0] if tag == "test" else None
        print(f"  RX {time.strftime('%H:%M:%S', time.localtime(ts))}  {len(r['data'])} B  "
              f"RSSI {r['rssi']} dBm  SNR {r['snr']:g} dB  {tag}" + (f" seq {seq}" if seq is not None else ""))
    print(f"{n} packet(s) received")
    return 0


def run_smoke(m, args, rep):
    preset = PRESETS[args.preset]
    rep.data["device"] = m.name
    rep.data["firmware"] = m.version()
    rep.check("ping / version", rep.data["firmware"], rep.data["firmware"])
    rep.data["debug"] = m.debug()

    # Valid config round trip
    cfg = config_from_args(args)
    try:
        back = m.set_config(cfg)
        rep.data["config"] = back
        same = all(back[k] == cfg[k] for k in ("freq_hz", "bandwidth_hz", "sf", "cr", "syncword", "preamble_len"))
        rep.check("SET_CONFIG test preset", same, fmt_config(back))
        rep.data["power_readback_dbm"] = back["power_dbm"]
        rep.check("power readback", back["power_dbm"] <= cfg["power_dbm"],
                  f"requested {cfg['power_dbm']}, modem reports {back['power_dbm']}")
        got = m.get_config()
        rep.check("GET_CONFIG matches", got == back)
    except ModemError as exc:
        rep.check("SET_CONFIG test preset", False, str(exc))
        return

    # Invalid configs must be rejected and leave the radio usable
    other = "2g4" if args.preset == "subg" else "subg"
    bad_cases = [
        ("bad SF (13)", dict(cfg, sf=13)),
        ("bad CR (9)", dict(cfg, cr=9)),
        ("bad BW (1 kHz)", dict(cfg, bandwidth_hz=1000)),
        (f"other band ({other} preset)", dict(PRESETS[other])),
    ]
    rep.data["rejects"] = {}
    last_ok = back
    for name, bad in bad_cases:
        try:
            last_ok = m.set_config(bad)
            result = "accepted"
        except ModemError as exc:
            result = str(exc)
        rep.data["rejects"][name] = result
        # The other-band case is informative: dual-band chips accept it.
        if name.startswith("other band"):
            print(f"  [INFO] {name}: {result}")
        else:
            rep.check(f"reject {name}", result != "accepted", result)
    # v0.8+: a rejected SET_CONFIG must leave the previous config running,
    # so GET_CONFIG still shows the last accepted one (the other-band preset
    # on dual-band chips) and TX works without reconfiguring.
    try:
        kept = m.get_config() == last_ok
        m.transmit(test_payload(1, 16))
        rep.check("rejected config keeps previous", kept, "GET_CONFIG unchanged, TX ok" if kept else "GET_CONFIG changed")
    except ModemError as exc:
        rep.check("rejected config keeps previous", False, str(exc))
    try:
        m.set_config(cfg)
        rep.check("recovers after rejects", True)
    except ModemError as exc:
        rep.check("recovers after rejects", False, str(exc))
        return

    # Standby / resume
    rep.check("RADIO_STANDBY", m.standby())
    rep.check("RADIO_RESUME", m.resume())

    # CAD on request, then auto-CAD in front of a TX. On a quiet bench
    # channel every scan should finish and report clear. Chips whose radio
    # info lacks the CAD feature must refuse both cleanly instead.
    try:
        has_cad = "CAD" in m.radio_info()["features"]
    except ModemError:
        has_cad = True   # pre-v0.8 firmware: every build had SX1262 CAD
    if not has_cad:
        try:
            m.cad()
            rep.check("CAD_REQUEST refused (no CAD)", False, "scan reported a result")
        except ModemError as exc:
            rep.check("CAD_REQUEST refused (no CAD)", str(exc) == "ERR_CAD_FAILED", str(exc))
        refused = not m.set_auto_cad(True)
        rep.check("auto-CAD refused (no CAD)", refused)
        if not refused:
            m.set_auto_cad(False)
    try:
        if not has_cad:
            raise StopIteration
        verdicts = [m.cad() for _ in range(5)]
        rep.data["cad"] = verdicts
        rep.check("CAD_REQUEST completes", True,
                  f"{verdicts.count(False)} clear / {verdicts.count(True)} busy of 5")
    except StopIteration:
        pass
    except ModemError as exc:
        rep.check("CAD_REQUEST completes", False, str(exc))
    try:
        if not has_cad:
            raise StopIteration
        m.set_auto_cad(True)
        m.transmit(test_payload(0, 16))
        rep.check("TX with auto-CAD", True)
    except StopIteration:
        pass
    except ModemError as exc:
        rep.check("TX with auto-CAD", False, str(exc))
    finally:
        if has_cad:
            m.set_auto_cad(False)

    # TX airtime at bench power
    air = []
    for length in (16, 64, 200):
        expected = lora_time_on_air_us(cfg, length)
        try:
            airtime, wall = m.transmit(test_payload(length, length))
        except ModemError as exc:
            rep.check(f"TX {length} B", False, str(exc))
            continue
        err = (airtime - expected) / expected * 100
        air.append(dict(len=length, airtime_us=airtime, calc_us=expected, round_trip_ms=int(wall * 1000)))
        rep.check(f"TX {length} B airtime", abs(err) < 3.0,
                  f"{airtime} us vs calc {expected} us ({err:+.1f}%), round trip {wall * 1000:.0f} ms")
        time.sleep(0.3)
    rep.data["tx"] = air

    # Noise floor: give the sampler time after the last TX (it skips 500 ms)
    time.sleep(1.5)
    nf = m.noise_floor()
    rep.data["noise_floor_dbm"] = nf
    rep.check("noise floor plausible", -140.0 <= nf <= -60.0, f"{nf:.1f} dBm")

    st = m.status()
    rep.data["status"] = st
    rep.check("TX counter advanced", st["tx_count"] >= len(air), f"tx_count {st['tx_count']}")

    run_power_checks(m, args, rep, cfg)


def run_power_checks(m, args, rep, cfg):
    """Radio info, per-band power clamping and the band switch (protocol v0.8+).

    Only SET_CONFIG and RADIO_INFO are used here - nothing transmits - and
    the test preset at bench power is restored at the end.
    """
    try:
        ri = m.radio_info()
    except ModemError as exc:
        rep.check("GET_RADIO_INFO", False, str(exc))
        return
    rep.data["radio_info"] = ri
    rep.check("GET_RADIO_INFO", ri["version"] >= 1 and ri["bands"],
              f"{ri['chip']}, bands {[b['band'] for b in ri['bands']]}, features {ri['features']}")
    try:
        # Each band in turn at 22 dBm requested: the applied level must be
        # that band's ceiling or 22, whichever is lower. Going sub-GHz then
        # 2.4 GHz is the band switch that must never carry a sub-GHz drive
        # level onto the 2.4 GHz path.
        for b in sorted(ri["bands"], key=lambda b: b["band"] != "subg"):
            want = min(22, b["power_max_dbm"])
            m.set_config(dict(PRESETS[b["band"]], power_dbm=22))
            got = m.radio_info()["applied_power_dbm"]
            rep.check(f"{b['band']} 22 dBm request applies {want}", got == want, f"applied {got} dBm")
            over = b["power_max_dbm"] + 8
            if over <= 127:
                m.set_config(dict(PRESETS[b["band"]], power_dbm=over))
                got = m.radio_info()["applied_power_dbm"]
                rep.check(f"{b['band']} over-ceiling request clamps", got == b["power_max_dbm"],
                          f"requested {over}, applied {got} dBm")
            low = b["power_min_dbm"] - 1
            try:
                m.set_config(dict(PRESETS[b["band"]], power_dbm=low))
                result = "accepted"
            except ModemError as exc:
                result = str(exc)
            rep.check(f"{b['band']} below chip minimum rejected", result != "accepted", f"{low} dBm: {result}")
    except ModemError as exc:
        rep.check("power checks", False, str(exc))
    finally:
        m.set_config(cfg)


def cmd_smoke(m, args):
    rep = Report()
    print(f"Smoke test on {m.name} ({args.preset} preset, {config_from_args(args)['power_dbm']} dBm)")
    run_smoke(m, args, rep)
    rep.data["checks"] = rep.checks
    rep.data["debug_lines"] = m.debug_lines[-200:]
    rep.data["timestamp"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    if args.save:
        with open(args.save, "w", encoding="utf-8") as f:
            json.dump(rep.data, f, indent=2)
        print(f"Saved {args.save}")
    rc = 0 if rep.passed else 1
    if args.compare:
        rc |= compare(rep.data, args.compare)
    print("RESULT:", "PASS" if rc == 0 else "FAIL")
    return rc


def compare(cur, path):
    with open(path, encoding="utf-8") as f:
        base = json.load(f)
    print(f"Comparing with {path} ({base.get('firmware')}, {base.get('timestamp')})")
    diffs = 0
    base_checks = {c["name"]: c["ok"] for c in base.get("checks", [])}
    for c in cur.get("checks", []):
        if base_checks.get(c["name"]) and not c["ok"]:
            print(f"  [REGRESSION] {c['name']} passed in baseline, fails now")
            diffs += 1
    for name, res in base.get("rejects", {}).items():
        now = cur.get("rejects", {}).get(name)
        if now != res:
            print(f"  [CHANGED] {name}: baseline {res}, now {now}")
            diffs += 1
    for b, n in zip(base.get("tx", []), cur.get("tx", [])):
        if b["len"] == n["len"] and b["airtime_us"] != n["airtime_us"]:
            print(f"  [CHANGED] TX {b['len']} B airtime {b['airtime_us']} -> {n['airtime_us']} us")
            diffs += 1
    if "noise_floor_dbm" in base and "noise_floor_dbm" in cur:
        delta = cur["noise_floor_dbm"] - base["noise_floor_dbm"]
        if abs(delta) > 6.0:
            print(f"  [CHANGED] noise floor {base['noise_floor_dbm']:.1f} -> {cur['noise_floor_dbm']:.1f} dBm")
            diffs += 1
    if base.get("power_readback_dbm") != cur.get("power_readback_dbm"):
        print(f"  [CHANGED] power readback {base.get('power_readback_dbm')} -> {cur.get('power_readback_dbm')}")
        diffs += 1
    print(f"  {diffs} difference(s) from baseline")
    return 1 if diffs else 0


def pair_direction(tx, rx, cfg, count, length, gap):
    expected = lora_time_on_air_us(cfg, length)
    while not rx.rx_packets.empty():
        rx.rx_packets.get_nowait()
    got, rssi, snr, tx_fail, tx_errors = {}, [], [], 0, {}
    for seq in range(count):
        try:
            tx.transmit(test_payload(seq, length))
        except ModemError as exc:
            tx_fail += 1
            tx_errors[str(exc)] = tx_errors.get(str(exc), 0) + 1
            continue
        deadline = time.time() + expected / 1e6 + 0.5
        while time.time() < deadline:
            try:
                _, p = rx.rx_packets.get(timeout=0.05)
            except queue.Empty:
                continue
            r = decode_rx(p)
            if r["data"].startswith(TEST_MAGIC):
                s = struct.unpack("<H", r["data"][3:5])[0]
                got[s] = r
                rssi.append(r["rssi"])
                snr.append(r["snr"])
                if s == seq:
                    break
        time.sleep(gap)
    res = dict(sent=count - tx_fail, tx_fail=tx_fail, tx_errors=tx_errors, received=len(got))
    if rssi:
        res.update(rssi_min=min(rssi), rssi_avg=round(statistics.mean(rssi), 1), rssi_max=max(rssi),
                   snr_min=min(snr), snr_avg=round(statistics.mean(snr), 1), snr_max=max(snr))
    return res


def cmd_pair(args):
    a = Modem(args.a, args.verbose, args.dtr)
    b = Modem(args.b, args.verbose, args.dtr)
    try:
        print(f"A: {a.name}  {a.version()}")
        print(f"B: {b.name}  {b.version()}")
        sfs = args.sf_list or [config_from_args(args)["sf"]]
        results = []
        rc = 0
        for sf in sfs:
            cfg = dict(config_from_args(args), sf=sf)
            a.set_config(cfg)
            b.set_config(cfg)
            print(f"\n{fmt_config(cfg)}")
            for name, tx, rx in (("A->B", a, b), ("B->A", b, a)):
                r = pair_direction(tx, rx, cfg, args.count, args.len, args.gap)
                r.update(direction=name, sf=sf)
                results.append(r)
                ok = r["received"] == r["sent"] and r["sent"] > 0
                rc |= 0 if ok else 1
                line = f"  [{'PASS' if ok else 'FAIL'}] {name}: {r['received']}/{r['sent']} received"
                if "rssi_avg" in r:
                    line += (f"  RSSI {r['rssi_min']}/{r['rssi_avg']}/{r['rssi_max']} dBm"
                             f"  SNR {r['snr_min']}/{r['snr_avg']}/{r['snr_max']} dB (min/avg/max)")
                if r["tx_errors"]:
                    line += f"  TX errors {r['tx_errors']}"
                print(line)
        if args.save:
            with open(args.save, "w", encoding="utf-8") as f:
                json.dump(dict(a=a.name, b=b.name, results=results,
                               timestamp=time.strftime("%Y-%m-%dT%H:%M:%S")), f, indent=2)
            print(f"Saved {args.save}")
        print("RESULT:", "PASS" if rc == 0 else "FAIL")
        return rc
    finally:
        a.close()
        b.close()


# ─── CLI ─────────────────────────────────────────────────────────────────
def add_radio_args(p):
    p.add_argument("--preset", choices=sorted(PRESETS), default="subg",
                   help="test preset (default subg: 927.925 MHz / 125 kHz / SF9 / CR5)")
    p.add_argument("--freq", type=float, help="override frequency, MHz")
    p.add_argument("--bw", type=float, help="override bandwidth, kHz")
    p.add_argument("--sf", type=int, help="override spreading factor")
    p.add_argument("--cr", type=int, help="override coding rate denominator (5-8)")
    p.add_argument("--power", type=int, help=f"TX power dBm (default {BENCH_POWER_DBM})")
    p.add_argument("--syncword", type=lambda s: int(s, 0), help="override sync word")
    p.add_argument("--preamble", type=int, help="override preamble length")


def main():
    # Firmware debug text can contain bytes the Windows console codepage
    # can't encode; never let printing it kill a test run.
    sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help='device: "COM19" or "tcp:HOST[:PORT[:TOKEN]]"')
    ap.add_argument("-v", "--verbose", action="store_true", help="show firmware debug text")
    ap.add_argument("--dtr", choices=("auto", "on", "off"), default="auto",
                    help="serial DTR line (auto: high for native USB, low for USB-UART bridges)")
    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("info", help="version, config, status, noise floor")

    p = sub.add_parser("config", help="apply a radio config and read it back")
    add_radio_args(p)

    p = sub.add_parser("tx", help="transmit test packets")
    add_radio_args(p)
    p.add_argument("--count", type=int, default=5)
    p.add_argument("--len", type=int, default=32)
    p.add_argument("--gap", type=float, default=0.5, help="seconds between packets")
    p.add_argument("--no-config", action="store_true", help="keep the modem's current config")

    p = sub.add_parser("rx", help="print received packets")
    add_radio_args(p)
    p.add_argument("--seconds", type=float, default=30)
    p.add_argument("--no-config", action="store_true", help="keep the modem's current config")

    p = sub.add_parser("smoke", help="bring-up checks that need only one board")
    add_radio_args(p)
    p.add_argument("--save", help="write results to JSON")
    p.add_argument("--compare", help="compare with a saved baseline JSON")

    p = sub.add_parser("pair", help="two-board TX/RX test in both directions")
    add_radio_args(p)
    p.add_argument("--a", required=True, help="first device")
    p.add_argument("--b", required=True, help="second device")
    p.add_argument("--count", type=int, default=20)
    p.add_argument("--len", type=int, default=32)
    p.add_argument("--gap", type=float, default=0.3)
    p.add_argument("--sf-list", type=lambda s: [int(x) for x in s.split(",")],
                   help="comma-separated SFs to sweep, e.g. 7,9,12")
    p.add_argument("--save", help="write results to JSON")

    args = ap.parse_args()
    try:
        if args.command == "pair":
            return cmd_pair(args)
        if not args.port:
            ap.error("--port is required for this command")
        m = Modem(args.port, args.verbose, args.dtr)
        try:
            return {"info": cmd_info, "config": cmd_config, "tx": cmd_tx,
                    "rx": cmd_rx, "smoke": cmd_smoke}[args.command](m, args)
        finally:
            m.close()
    except ModemError as exc:
        print(f"ERROR: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
