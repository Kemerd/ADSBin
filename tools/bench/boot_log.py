# -*- coding: utf-8 -*-
"""
boot_log.py — capture a clean ESP32 boot log to a file over the CDC/UART console.

Resets the board (DTR/RTS toggle), then streams everything the device prints for a
fixed window straight into a log file. Unlike a timed in-memory grab, this writes
continuously so nothing is lost to a capture-window race — perfect for catching the
exact line where boot stops.

Usage:
    py -3 boot_log.py [COMx] [seconds] [outfile]
    (defaults: COM6, 12 s, boot_capture.log)
"""
from __future__ import annotations

import sys
import time

import serial  # pyserial


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    out  = sys.argv[3] if len(sys.argv) > 3 else "boot_capture.log"

    # Open at the console baud. We assert reset via RTS, release, then read.
    sp = serial.Serial()
    sp.port = port
    sp.baudrate = 115200
    sp.timeout = 0.1
    # Drive a hardware reset: RTS controls EN on these boards. DTR low avoids
    # accidentally entering download mode (which needs BOTH in a specific order).
    sp.dtr = False
    sp.rts = True   # hold reset asserted
    sp.open()
    time.sleep(0.2)
    sp.rts = False  # release reset -> board boots fresh from here
    sp.reset_input_buffer()

    deadline = time.monotonic() + secs
    n = 0
    with open(out, "wb") as f:
        while time.monotonic() < deadline:
            chunk = sp.read(4096)
            if chunk:
                f.write(chunk)
                f.flush()
                n += len(chunk)
            else:
                time.sleep(0.01)
    sp.close()
    print(f"wrote {n} bytes of boot log to {out} over {secs:.0f}s on {port}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
