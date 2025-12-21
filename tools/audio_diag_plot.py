#!/usr/bin/env python3
"""
Poll Beyondex audio diagnostics over USB (vendor control request) and plot live.

Device firmware implements:
  bmRequestType = 0xC0 (IN | VENDOR | DEVICE)
  bRequest      = 0x43
  wValue        = 0
  wIndex        = 0
  wLength       = 20 bytes

Payload (little-endian):
  uint32 magic   = 0x44584542 ('BEXD')
  uint32 underrun_count
  uint32 overflow_count
  int32  buf_len
  int32  buf_us

Notes:
  - On macOS, the system audio driver may already have claimed the device, which
    can prevent libusb/pyusb from opening it. If you see permission/busy errors,
    try:
      * unplug/replug and run this before selecting it as an audio output, or
      * run on a host where you can detach/ignore the kernel driver (Linux).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import usb.core
import usb.util
import usb.backend.libusb1


MAGIC = 0x44584542  # 'BEXD'


@dataclass(frozen=True)
class Diag:
    t: float
    underrun: int
    overflow: int
    buf_len: int
    buf_us: int


def parse_diag(buf: bytes) -> tuple[int, int, int, int, int]:
    if len(buf) != 20:
        raise ValueError(f"expected 20 bytes, got {len(buf)}")
    magic, underrun, overflow, buf_len, buf_us = struct.unpack("<IIIii", buf)
    return magic, underrun, overflow, buf_len, buf_us


def find_device(vid: int, pid: Optional[int]) -> usb.core.Device:
    backend = usb.backend.libusb1.get_backend()
    if backend is None:
        # Try common Homebrew locations on macOS
        import os
        from ctypes.util import find_library

        candidates = [
            "/opt/homebrew/lib/libusb-1.0.dylib",  # Apple Silicon Homebrew
            "/usr/local/lib/libusb-1.0.dylib",     # Intel Homebrew
        ]
        lib = find_library("usb-1.0") or find_library("libusb-1.0")
        if lib:
            candidates.insert(0, lib)

        for c in candidates:
            if c and os.path.exists(c):
                backend = usb.backend.libusb1.get_backend(find_library=lambda _: c)
                if backend is not None:
                    break

    if backend is None:
        raise RuntimeError(
            "No backend available (PyUSB could not load libusb).\n"
            "Fix:\n"
            "  - macOS (Homebrew): `brew install libusb`\n"
            "  - Linux: install `libusb-1.0-0`\n"
            "  - Windows: install libusb and a WinUSB/libusb driver for the device (e.g. Zadig)\n"
            "Then re-run this script.\n"
            "If on macOS Apple Silicon, libusb should be at /opt/homebrew/lib/libusb-1.0.dylib."
        )

    dev = usb.core.find(idVendor=vid, idProduct=pid, backend=backend) if pid is not None else usb.core.find(idVendor=vid, backend=backend)
    if dev is None:
        if pid is None:
            raise RuntimeError(f"device not found (vid=0x{vid:04x})")
        raise RuntimeError(f"device not found (vid=0x{vid:04x}, pid=0x{pid:04x})")
    return dev


def try_set_config(dev: usb.core.Device) -> None:
    try:
        dev.set_configuration()
    except usb.core.USBError:
        # Often already configured/claimed; control transfers can still work.
        pass


def read_diag(dev: usb.core.Device, timeout_ms: int) -> Diag:
    # ctrl_transfer returns an array('B') / bytes-like
    data = dev.ctrl_transfer(0xC0, 0x43, 0, 0, 20, timeout=timeout_ms)
    b = bytes(data)
    magic, underrun, overflow, buf_len, buf_us = parse_diag(b)
    if magic != MAGIC:
        raise RuntimeError(f"bad magic 0x{magic:08x} (expected 0x{MAGIC:08x})")
    return Diag(t=time.time(), underrun=underrun, overflow=overflow, buf_len=buf_len, buf_us=buf_us)


def main() -> int:
    ap = argparse.ArgumentParser(description="Live plot Beyondex USB audio diagnostics (vendor request 0x43).")
    ap.add_argument("--vid", default="0xCAFE", help="USB VID (default: 0xCAFE)")
    ap.add_argument("--pid", default=None, help="USB PID (optional, e.g. 0x4010)")
    ap.add_argument("--hz", type=float, default=10.0, help="Poll rate in Hz (default: 10)")
    ap.add_argument("--window", type=float, default=60.0, help="Plot window in seconds (default: 60)")
    ap.add_argument("--timeout-ms", type=int, default=200, help="USB control transfer timeout (default: 200)")
    ap.add_argument("--csv", default=None, help="Optional CSV log path")
    args = ap.parse_args()

    vid = int(args.vid, 0)
    pid = int(args.pid, 0) if args.pid is not None else None

    # Import matplotlib lazily so users can at least try connecting without it.
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    period = 1.0 / max(args.hz, 0.1)
    maxlen = max(10, int(args.window * args.hz))

    ts = deque(maxlen=maxlen)
    buf_us = deque(maxlen=maxlen)
    buf_len = deque(maxlen=maxlen)
    underrun = deque(maxlen=maxlen)
    overflow = deque(maxlen=maxlen)

    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "w", encoding="utf-8")
        csv_f.write("t,buf_us,buf_len,underrun,overflow\n")
        csv_f.flush()

    dev: Optional[usb.core.Device] = None
    last_connect_attempt = 0.0

    fig, (ax0, ax1) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle("Beyondex USB audio diagnostics (vendor req 0x43)")

    (l_buf_us,) = ax0.plot([], [], label="buf_us")
    ax0.set_ylabel("buffered us")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right")

    (l_buf_len,) = ax1.plot([], [], label="buf_len")
    (l_underrun,) = ax1.plot([], [], label="underrun_count")
    (l_overflow,) = ax1.plot([], [], label="overflow_count")
    ax1.set_ylabel("count / buf_len")
    ax1.set_xlabel("seconds (relative)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    t0: Optional[float] = None
    last_poll = 0.0

    def update(_frame: int):
        nonlocal dev, last_connect_attempt, t0, last_poll

        now = time.time()
        if now - last_poll < period:
            return (l_buf_us, l_buf_len, l_underrun, l_overflow)
        last_poll = now

        if dev is None and (now - last_connect_attempt) > 1.0:
            last_connect_attempt = now
            try:
                dev = find_device(vid, pid)
                try_set_config(dev)
                # No interface claim needed for EP0 control transfers.
                print(f"Connected: vid=0x{dev.idVendor:04x} pid=0x{dev.idProduct:04x}")
            except Exception as e:
                dev = None
                print(f"Waiting for device... ({e})")
                return (l_buf_us, l_buf_len, l_underrun, l_overflow)

        if dev is None:
            return (l_buf_us, l_buf_len, l_underrun, l_overflow)

        try:
            d = read_diag(dev, timeout_ms=args.timeout_ms)
        except usb.core.USBError as e:
            print(f"USB error: {e}; will retry connect")
            dev = None
            return (l_buf_us, l_buf_len, l_underrun, l_overflow)
        except Exception as e:
            print(f"Read error: {e}; will retry connect")
            dev = None
            return (l_buf_us, l_buf_len, l_underrun, l_overflow)

        if t0 is None:
            t0 = d.t
        tr = d.t - t0

        ts.append(tr)
        buf_us.append(d.buf_us)
        buf_len.append(d.buf_len)
        underrun.append(d.underrun)
        overflow.append(d.overflow)

        if csv_f:
            csv_f.write(f"{tr:.6f},{d.buf_us},{d.buf_len},{d.underrun},{d.overflow}\n")
            csv_f.flush()

        l_buf_us.set_data(ts, buf_us)
        l_buf_len.set_data(ts, buf_len)
        l_underrun.set_data(ts, underrun)
        l_overflow.set_data(ts, overflow)

        # keep a rolling view
        if ts:
            ax1.set_xlim(max(0.0, ts[-1] - args.window), ts[-1] + 1e-6)

        # autoscale Y with a bit of headroom
        ax0.relim()
        ax0.autoscale_view(scalex=False, scaley=True)
        ax1.relim()
        ax1.autoscale_view(scalex=False, scaley=True)

        return (l_buf_us, l_buf_len, l_underrun, l_overflow)

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    try:
        plt.show()
    finally:
        if csv_f:
            csv_f.close()
        try:
            if dev is not None:
                usb.util.dispose_resources(dev)
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


