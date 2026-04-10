#!/usr/bin/env python3
"""
Drone Vertical Estimator Monitor
Reads binary packets from /dev/ttyACM0 and displays live EKF state.

Packet format:
  [0]    SYNC0  = 0xA5
  [1]    SYNC1  = 0x5A
  [2]    MSG_ID = 0x01
  [3]    PAYLOAD_LEN
  [4]    SEQ
  [5..N] payload (VerticalEstimatePayload)
  [N+1]  CRC low
  [N+2]  CRC high

VerticalEstimatePayload (packed, 18 bytes):
  uint32  timestamp_ms
  float   z_world_m
  float   vz_world_mps
  float   agl_m
  uint8   ekf_initialized
  uint8   lidar_accepted
"""

import struct
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
import serial
import sys

# ── Protocol constants ──────────────────────────────────────────────────────
SYNC0          = 0xA5
SYNC1          = 0x5A
MSG_VERTICAL   = 0x01
PAYLOAD_FMT    = "<IfffBB"   # little-endian: uint32 + 3×float + 2×uint8
PAYLOAD_SIZE   = struct.calcsize(PAYLOAD_FMT)   # 18 bytes

SERIAL_PORT    = "/dev/ttyACM0"
BAUD_RATE      = 460800

# ── CRC16-CCITT-FALSE (poly=0x1021, init=0xFFFF) ────────────────────────────
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
            crc &= 0xFFFF
    return crc

# ── Packet parser ───────────────────────────────────────────────────────────
def parse_packets(ser, callback):
    """
    Continuously reads from `ser`, finds sync bytes, validates CRC,
    and calls callback(parsed_dict) for each good packet.
    """
    buf = bytearray()
    stats = {"rx": 0, "ok": 0, "crc_err": 0, "seq_last": -1, "dropped": 0}

    while True:
        chunk = ser.read(64)
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= 2:
            # Search for sync header
            if buf[0] != SYNC0 or buf[1] != SYNC1:
                buf.pop(0)
                continue

            # Need at least header (5 bytes) to know payload length
            if len(buf) < 5:
                break

            msg_id      = buf[2]
            payload_len = buf[3]
            seq         = buf[4]
            frame_len   = 5 + payload_len + 2   # header + payload + 2-byte CRC

            if len(buf) < frame_len:
                break   # wait for more data

            payload_bytes = bytes(buf[5 : 5 + payload_len])
            crc_lo        = buf[5 + payload_len]
            crc_hi        = buf[5 + payload_len + 1]
            received_crc  = crc_lo | (crc_hi << 8)

            # CRC covers: msg_id, payload_len, seq, payload
            crc_data = bytes([msg_id, payload_len, seq]) + payload_bytes
            expected_crc = crc16_ccitt_false(crc_data)

            stats["rx"] += 1

            if received_crc != expected_crc:
                stats["crc_err"] += 1
                buf.pop(0)   # resync: discard SYNC0 and try again
                continue

            # CRC OK – consume frame
            buf = buf[frame_len:]
            stats["ok"] += 1

            # Sequence-number drop detection
            if stats["seq_last"] >= 0:
                expected_seq = (stats["seq_last"] + 1) & 0xFF
                if seq != expected_seq:
                    gap = (seq - expected_seq) & 0xFF
                    stats["dropped"] += gap
            stats["seq_last"] = seq

            if msg_id == MSG_VERTICAL and payload_len == PAYLOAD_SIZE:
                ts_ms, z_m, vz_mps, agl_m, ekf_init, lidar_acc = \
                    struct.unpack(PAYLOAD_FMT, payload_bytes)
                callback({
                    "timestamp_ms":    ts_ms,
                    "z_world_m":       z_m,
                    "vz_world_mps":    vz_mps,
                    "agl_m":           agl_m,
                    "ekf_initialized": bool(ekf_init),
                    "lidar_accepted":  bool(lidar_acc),
                    "seq":             seq,
                    "stats":           dict(stats),
                })

# ── GUI ─────────────────────────────────────────────────────────────────────
BG        = "#0d0f14"
CARD_BG   = "#161a22"
BORDER    = "#2a2f3d"
LABEL_FG  = "#6b7280"
VALUE_FG  = "#e2e8f0"
ACCENT    = "#38bdf8"   # sky-400 for AGL
GREEN     = "#4ade80"
YELLOW    = "#facc15"
RED_SOFT  = "#f87171"
MUTED     = "#374151"

CARDS = [
    ("AGL",         "agl_m",         "m",    ACCENT,   "Height above ground"),
    ("Vz",          "vz_world_mps",  "m/s",  GREEN,    "Vertical velocity"),
    ("Z world",     "z_world_m",     "m",    YELLOW,   "World-frame altitude"),
    ("Lidar acc.",  "lidar_accepted", "",    RED_SOFT, "Last lidar update accepted"),
]

class DroneMonitor(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Vertical Estimator")
        self.configure(bg=BG)
        self.resizable(True, True)
        self.minsize(520, 520)

        self._data = {}
        self._lock = threading.Lock()
        self._connected = False
        self._packet_rate = 0.0
        self._last_ts = None
        self._rate_times = []

        self._build_ui()
        self._start_serial()
        self._poll()

    # ── UI construction ──────────────────────────────────────────────────────
    def _build_ui(self):
        pad = dict(padx=18, pady=12)

        # Title bar
        hdr = tk.Frame(self, bg=BG)
        hdr.pack(fill="x", **pad)
        tk.Label(hdr, text="Drone Vertical Monitor",
                 bg=BG, fg=VALUE_FG,
                 font=("Helvetica Neue", 15, "bold")).pack(side="left")
        self._conn_dot = tk.Label(hdr, text="●", bg=BG, fg=RED_SOFT,
                                  font=("Helvetica Neue", 16))
        self._conn_dot.pack(side="right", padx=(0, 4))
        self._conn_label = tk.Label(hdr, text="disconnected",
                                    bg=BG, fg=LABEL_FG,
                                    font=("Helvetica Neue", 11))
        self._conn_label.pack(side="right")

        # Card grid
        grid = tk.Frame(self, bg=BG)
        grid.pack(fill="both", expand=True, padx=14, pady=0)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        self._value_labels = {}
        self._unit_labels  = {}
        self._card_frames  = {}

        for i, (title, key, unit, color, subtitle) in enumerate(CARDS):
            row, col = divmod(i, 2)
            card = tk.Frame(grid, bg=CARD_BG,
                            highlightbackground=BORDER,
                            highlightthickness=1)
            card.grid(row=row, column=col, sticky="nsew",
                      padx=6, pady=6, ipadx=16, ipady=16)
            grid.rowconfigure(row, weight=1)
            self._card_frames[key] = card

            tk.Label(card, text=title.upper(),
                     bg=CARD_BG, fg=LABEL_FG,
                     font=("Helvetica Neue", 10, "bold"),
                     anchor="w").pack(anchor="w", padx=4, pady=(4, 0))

            tk.Label(card, text=subtitle,
                     bg=CARD_BG, fg=MUTED,
                     font=("Helvetica Neue", 9),
                     anchor="w").pack(anchor="w", padx=4)

            val_frame = tk.Frame(card, bg=CARD_BG)
            val_frame.pack(anchor="w", padx=4, pady=(8, 4))

            val_lbl = tk.Label(val_frame, text="—",
                               bg=CARD_BG, fg=color,
                               font=("Helvetica Neue", 46, "bold"),
                               anchor="w")
            val_lbl.pack(side="left", anchor="s")

            if unit:
                unit_lbl = tk.Label(val_frame, text=unit,
                                    bg=CARD_BG, fg=LABEL_FG,
                                    font=("Helvetica Neue", 18),
                                    anchor="sw")
                unit_lbl.pack(side="left", anchor="s", padx=(4, 0), pady=(0, 6))
                self._unit_labels[key] = unit_lbl

            self._value_labels[key] = val_lbl

        # Status bar
        status_bar = tk.Frame(self, bg=CARD_BG,
                              highlightbackground=BORDER,
                              highlightthickness=1)
        status_bar.pack(fill="x", padx=14, pady=(0, 14), ipady=6)

        self._status_labels = {}
        fields = [
            ("ekf_initialized", "EKF"),
            ("seq",             "SEQ"),
            ("_pkt_rate",       "PKT/s"),
            ("_dropped",        "DROPPED"),
            ("timestamp_ms",    "T (ms)"),
        ]
        for i, (key, label) in enumerate(fields):
            col = tk.Frame(status_bar, bg=CARD_BG)
            col.pack(side="left", expand=True, padx=12)
            tk.Label(col, text=label,
                     bg=CARD_BG, fg=LABEL_FG,
                     font=("Helvetica Neue", 9, "bold")).pack()
            v = tk.Label(col, text="—",
                         bg=CARD_BG, fg=VALUE_FG,
                         font=("Helvetica Neue", 11))
            v.pack()
            self._status_labels[key] = v

    # ── Serial thread ────────────────────────────────────────────────────────
    def _start_serial(self):
        t = threading.Thread(target=self._serial_thread, daemon=True)
        t.start()

    def _serial_thread(self):
        while True:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                self._connected = True
                parse_packets(ser, self._on_packet)
            except serial.SerialException as e:
                self._connected = False
                time.sleep(1.0)

    def _on_packet(self, data: dict):
        now = time.monotonic()
        with self._lock:
            self._data = data
            self._rate_times.append(now)
            # Keep only last 2 s for rate calc
            cutoff = now - 2.0
            self._rate_times = [t for t in self._rate_times if t > cutoff]

    # ── GUI update loop ──────────────────────────────────────────────────────
    def _poll(self):
        with self._lock:
            d = dict(self._data)
            rate = len(self._rate_times) / 2.0 if self._rate_times else 0.0
            connected = self._connected

        if d:
            # Main value cards
            for title, key, unit, color, subtitle in CARDS:
                raw = d.get(key)
                if key == "lidar_accepted":
                    text  = "YES" if raw else "NO"
                    color_now = GREEN if raw else RED_SOFT
                elif raw is None:
                    text = "—"
                    color_now = color
                else:
                    text = f"{raw:+.3f}" if abs(raw) >= 0.001 else f"{raw:.4f}"
                    color_now = color
                self._value_labels[key].config(text=text, fg=color_now)

            # Status bar
            ekf = d.get("ekf_initialized")
            self._status_labels["ekf_initialized"].config(
                text="OK" if ekf else "INIT",
                fg=GREEN if ekf else YELLOW)
            self._status_labels["seq"].config(
                text=str(d.get("seq", "—")))
            self._status_labels["_pkt_rate"].config(
                text=f"{rate:.1f}")
            dropped = d.get("stats", {}).get("dropped", 0)
            self._status_labels["_dropped"].config(
                text=str(dropped),
                fg=RED_SOFT if dropped > 0 else VALUE_FG)
            ts = d.get("timestamp_ms")
            self._status_labels["timestamp_ms"].config(
                text=f"{ts:,}" if ts is not None else "—")

        # Connection indicator
        if connected and d:
            self._conn_dot.config(fg=GREEN)
            self._conn_label.config(text="connected")
        elif connected:
            self._conn_dot.config(fg=YELLOW)
            self._conn_label.config(text="waiting…")
        else:
            self._conn_dot.config(fg=RED_SOFT)
            self._conn_label.config(text=f"no device at {SERIAL_PORT}")

        self.after(50, self._poll)   # 20 Hz GUI refresh


# ── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Allow overriding port via CLI: python drone_monitor.py /dev/ttyUSB0
    if len(sys.argv) > 1:
        SERIAL_PORT = sys.argv[1]

    app = DroneMonitor()
    app.mainloop()