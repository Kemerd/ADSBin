# -*- coding: utf-8 -*-
"""
cdc_link.py — USB-CDC serial transport for the ADSBin bench harness.

The ESP32-P4 exposes a single USB Serial/JTAG endpoint on its USB-C port. That
one link carries BOTH the firmware's UTF-8 console/debug text (the ``sink_debug``
traffic table, ``+OK``/``+ERR`` injection replies) AND the binary GDL90 byte
stream (framed by ``0x7E`` flags). This module owns the physical link: it opens
the port, auto-detects the device by USB VID:PID, and hands the bench a clean,
thread-safe stream of raw bytes plus a convenience line reader for the text side.

Design notes
------------
* Pure I/O. No protocol knowledge lives here — GDL90 deframing is gdl90.py's job
  and token parsing is sink_debug_parse.py's. Keeping this layer dumb means the
  same link object drives every subcommand without special-casing.
* A background reader thread continuously drains the OS serial buffer into an
  in-memory ring of bytes. This matters because the P4 streams GDL90 + text at a
  steady clip; if we only read on demand the kernel buffer overruns and we lose
  frames mid-burst. The reader never blocks the CLI thread.
* We expose two views over the same captured bytes: ``read_bytes()`` for the
  GDL90 deframer and ``read_line()`` for the text protocols. They are consistent
  because both are fed from the one reader thread under a single lock.

Only pyserial is required (see requirements.txt). Windows-console / UTF-8 safe.
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import serial                       # pyserial
import serial.tools.list_ports      # cross-platform port enumeration


# ─────────────────────────────────────────────────────────────────────────────
#  Known USB identifiers for the ESP32-P4 USB Serial/JTAG device.
#
#  The P4's built-in USB-Serial-JTAG presents Espressif's VID 0x303A. The native
#  Serial/JTAG PID is 0x1001; we also accept the classic ESP USB-CDC PID 0x4001
#  in case the firmware exposes TinyUSB CDC instead. The bench auto-detects on
#  any of these, but a user can always force a specific port by name.
# ─────────────────────────────────────────────────────────────────────────────
ESP32P4_USB_IDS = (
    (0x303A, 0x1001),   # Espressif USB Serial/JTAG (built-in).
    (0x303A, 0x4001),   # Espressif TinyUSB CDC-ACM.
    (0x303A, 0x0002),   # Espressif generic CDC (some IDF configs).
)

# The console baud is irrelevant for a true USB-CDC/JTAG endpoint (it ignores the
# line rate), but pyserial still wants a number. 115200 matches the IDF console
# default so a real UART bridge would also work unchanged.
DEFAULT_BAUD = 115200


@dataclass
class PortInfo:
    """One enumerated serial port and the bits the bench cares about."""
    device: str             # OS port name, e.g. "COM7" or "/dev/ttyACM0".
    description: str        # Human label from the OS, may be empty.
    vid: Optional[int]      # USB vendor id, or None for non-USB ports.
    pid: Optional[int]      # USB product id, or None.
    serial_number: Optional[str]
    is_adsbin: bool         # True if (vid,pid) matches a known ESP32-P4 id.


def list_ports() -> List[PortInfo]:
    """
    Enumerate every serial port the OS sees, flagging likely ADSBin devices.

    @return  All ports, ADSBin-matching ones flagged via ``is_adsbin``. The list
             is returned in the OS's natural order so the caller can present it.
    """
    out: List[PortInfo] = []

    # serial.tools.list_ports.comports() is the portable enumerator across
    # Windows (SetupAPI), Linux (/sys) and macOS (IOKit).
    for p in serial.tools.list_ports.comports():
        vid = p.vid
        pid = p.pid

        # A device is "ADSBin" if its USB id pair is in our known table.
        is_adsbin = (vid is not None and pid is not None
                     and (vid, pid) in ESP32P4_USB_IDS)

        out.append(PortInfo(
            device=p.device,
            description=p.description or "",
            vid=vid,
            pid=pid,
            serial_number=p.serial_number,
            is_adsbin=is_adsbin,
        ))

    return out


def auto_detect_port() -> Optional[str]:
    """
    Pick the single best ADSBin port by VID:PID, or None if ambiguous/absent.

    @return  The device name of the lone matching port, else None. We refuse to
             guess when zero or more-than-one candidate matches so the caller can
             prompt the user instead of opening the wrong device.
    """
    candidates = [p for p in list_ports() if p.is_adsbin]

    # Exactly one match is the only unambiguous case worth auto-opening.
    if len(candidates) == 1:
        return candidates[0].device
    return None


class CdcLink:
    """
    A thread-safe USB-CDC link to the ADSBin device.

    Opens the serial port and spins a daemon reader thread that drains incoming
    bytes into an internal buffer. The bench pulls bytes for GDL90 deframing or
    reads newline-delimited UTF-8 lines for the text protocols, both fed from the
    same captured stream so the two views never disagree.

    Use as a context manager so the port and reader thread are always released::

        with CdcLink(port) as link:
            link.write_line("+INJECT 8D...")
            reply = link.read_line(timeout=1.0)
    """

    def __init__(self, port: str, baud: int = DEFAULT_BAUD,
                 read_timeout: float = 0.1) -> None:
        """
        Open @p port and start the background reader.

        @param port          OS device name (COMx / /dev/ttyACMx).
        @param baud          Line rate (ignored by true USB-CDC, set for UARTs).
        @param read_timeout  Per-read timeout the reader thread uses, seconds.
        """
        # Open the underlying pyserial port WITHOUT asserting the modem control
        # lines. On boards whose USB-C jack routes through a USB-UART bridge (e.g.
        # a CH343), DTR/RTS are wired to the P4's EN/IO0 auto-reset circuit, so a
        # default open() would reboot the device — losing any command we send and
        # resetting the traffic table mid-test. We construct the port unopened,
        # clear DTR/RTS, then open, so the board keeps running across connect.
        # (A true USB-Serial/JTAG endpoint ignores these lines, so this is safe on
        # every supported transport.)
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = baud
        self._serial.timeout = read_timeout
        self._serial.write_timeout = 2.0
        self._serial.dtr = False
        self._serial.rts = False
        self._serial.open()

        # Captured-but-unconsumed bytes for the GDL90 deframer view.
        self._byte_buf = bytearray()
        # Captured text awaiting a newline for the line-reader view. We keep this
        # separate from the byte view because the text reader strips framing it
        # has already consumed, whereas the byte view wants everything raw.
        self._line_buf = bytearray()
        # Completed (newline-terminated) lines, decoded to str, FIFO order.
        self._lines: List[str] = []
        # Persistent GDL90-frame state for the TEXT filter. Frames (0x7E..0x7E)
        # interleave with text on the shared link and can split across read
        # chunks, so we track "are we currently inside a frame?" ACROSS chunks and
        # route in-frame bytes away from the text accumulator. Without persistence
        # a frame split over two reads would leak its second half into the text.
        self._in_gdl90_frame = False

        # One lock guards all three buffers above; held only briefly.
        self._lock = threading.Lock()
        self._stop = threading.Event()

        # Daemon so a stuck reader can never wedge interpreter shutdown.
        self._reader = threading.Thread(
            target=self._read_loop, name="cdc-reader", daemon=True)
        self._reader.start()

        # Brief settle after opening the port. Even with DTR/RTS cleared, the act
        # of opening a USB-UART bridge can glitch the line for a few tens of ms;
        # a short pause lets the device's TX settle so the FIRST command we send
        # isn't lost to a transient. Without it, the very first inject after a
        # fresh connect can intermittently time out while later ones succeed.
        time.sleep(0.3)

    # ── lifecycle ────────────────────────────────────────────────────────────

    def __enter__(self) -> "CdcLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        """Stop the reader thread and close the port (idempotent)."""
        # Signal the loop to exit, then wait briefly for it to unwind.
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)

        # Closing the port may raise if already gone; swallow on teardown.
        try:
            if self._serial.is_open:
                self._serial.close()
        except Exception:
            pass

    # ── background reader ─────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        """
        Drain the serial port into our buffers until asked to stop.

        Runs on the daemon thread. Every chunk is appended to BOTH the raw byte
        view and the text-line accumulator, so a single physical read feeds both
        consumers without re-reading the device.
        """
        while not self._stop.is_set():
            try:
                # in_waiting lets us grab a whole burst in one syscall; fall back
                # to a single byte (which honours the read timeout) when idle so
                # we never spin the CPU.
                n = self._serial.in_waiting
                chunk = self._serial.read(n if n > 0 else 1)
            except (serial.SerialException, OSError):
                # Device unplugged or port died — stop cleanly; the CLI will see
                # empty reads and report it rather than crashing the thread.
                break

            if not chunk:
                continue

            # Publish the chunk to both views under the single lock.
            with self._lock:
                self._byte_buf.extend(chunk)
                self._consume_lines_locked(chunk)

    # GDL90 flag byte. A frame is FLAG ... FLAG; on the shared link these frames
    # interleave with the sink_debug text and would otherwise splice into a text
    # line and fracture a KEY=VALUE token. The byte view (read_bytes) keeps the
    # raw stream for the GDL90 deframer; the TEXT view filters frames out below.
    _GDL90_FLAG = 0x7E

    def _filter_text_bytes(self, chunk: bytes) -> bytes:
        """
        Strip interleaved GDL90 frames from @p chunk for the TEXT view.

        Walks the chunk byte-by-byte through a persistent in-frame state machine
        (``self._in_gdl90_frame``), so a frame split across read chunks is handled
        correctly: every byte from an opening 0x7E up to and including the closing
        0x7E is dropped (it belongs to the binary GDL90 view, captured separately).
        Each complete frame is replaced by a single newline so it acts as a line
        boundary instead of gluing the surrounding text lines together (a blank
        line is harmless — the parser keys only on ICAO/===/MSG lines). Bytes
        outside any frame pass through verbatim.
        """
        out = bytearray()
        for b in chunk:
            if self._in_gdl90_frame:
                # Inside a frame: discard until the closing flag, then emit one
                # newline to preserve the text line boundary the frame sat on.
                if b == self._GDL90_FLAG:
                    self._in_gdl90_frame = False
                    out.append(0x0A)   # '\n'
                # else: drop the in-frame byte.
            elif b == self._GDL90_FLAG:
                # Opening flag — enter frame state; nothing emitted yet.
                self._in_gdl90_frame = True
            else:
                # Ordinary text byte.
                out.append(b)
        return bytes(out)

    def _consume_lines_locked(self, chunk: bytes) -> None:
        """
        Append @p chunk to the line accumulator and split out completed lines.

        Must be called with ``self._lock`` held. Lines are split on ``\\n``; a
        trailing ``\\r`` (CRLF from the IDF console) is stripped. Bytes after the
        last newline stay buffered until their newline arrives.
        """
        # Filter interleaved binary GDL90 frames out of the text stream BEFORE
        # accumulating, tracking frame state across chunks (the GDL90 byte view
        # already captured them independently). This is the root fix for the
        # shared-link interleave: text lines stay pure grammar.
        self._line_buf.extend(self._filter_text_bytes(chunk))

        # Pull out every complete, newline-terminated line.
        while True:
            nl = self._line_buf.find(b"\n")
            if nl < 0:
                break

            # Slice off the line (without its newline) and advance the buffer.
            raw = bytes(self._line_buf[:nl])
            del self._line_buf[:nl + 1]

            # Decode leniently so any residual stray byte never throws; strip CR
            # and surrounding whitespace for clean token parsing.
            line = raw.decode("utf-8", errors="replace").rstrip("\r")
            self._lines.append(line)

    # ── byte view (GDL90) ─────────────────────────────────────────────────────

    def read_bytes(self, max_bytes: int = 65536) -> bytes:
        """
        Pull and clear up to @p max_bytes of captured raw bytes.

        @return  Bytes captured since the last call (FIFO), up to the cap. Use
                 this to feed gdl90.deframe_stream(); call it repeatedly while
                 accumulating across a capture window.
        """
        with self._lock:
            out = bytes(self._byte_buf[:max_bytes])
            del self._byte_buf[:max_bytes]
            return out

    def collect_bytes(self, duration_s: float) -> bytes:
        """
        Capture raw bytes for a fixed wall-clock window.

        @param duration_s  How long to accumulate, seconds.
        @return            All bytes the reader saw during the window. Handy for
                           ``dump-debug``/``validate-gdl90`` which want a steady
                           slice of the stream rather than a single read.
        """
        deadline = time.monotonic() + duration_s
        out = bytearray()

        # Poll the byte view until the window closes. The reader thread does the
        # actual blocking I/O; this loop just harvests at a gentle cadence.
        while time.monotonic() < deadline:
            out.extend(self.read_bytes())
            time.sleep(0.02)

        # Sweep up anything that arrived in the final tick.
        out.extend(self.read_bytes())
        return bytes(out)

    # ── text view (sink_debug / +INJECT) ──────────────────────────────────────

    def read_line(self, timeout: float = 1.0) -> Optional[str]:
        """
        Return the next complete text line, or None if none arrives in time.

        @param timeout  Max seconds to wait for a newline-terminated line.
        @return         The line (no trailing newline/CR), or None on timeout.
        """
        deadline = time.monotonic() + timeout

        # Spin on the queue the reader thread fills; sleep briefly between checks
        # so we yield the GIL and don't peg a core while waiting on the device.
        while time.monotonic() < deadline:
            with self._lock:
                if self._lines:
                    return self._lines.pop(0)
            time.sleep(0.01)

        return None

    def read_lines(self, duration_s: float) -> List[str]:
        """
        Collect every text line seen during a fixed wall-clock window.

        @param duration_s  Capture window, seconds.
        @return            Lines in arrival order. Used by ``dump-debug`` to grab
                           a full ``=== ADSBIN TRAFFIC ... ===`` block.
        """
        deadline = time.monotonic() + duration_s
        out: List[str] = []

        # Drain the line queue repeatedly until the window elapses.
        while time.monotonic() < deadline:
            with self._lock:
                while self._lines:
                    out.append(self._lines.pop(0))
            time.sleep(0.02)

        # Final sweep for lines completed on the last tick.
        with self._lock:
            while self._lines:
                out.append(self._lines.pop(0))
        return out

    def write_line(self, text: str) -> None:
        """
        Send one UTF-8 line to the device, appending a newline if absent.

        @param text  The command line (e.g. ``"+INJECT 8D..."``). A trailing
                     ``\\n`` is added so the firmware console sees a full line.
        """
        if not text.endswith("\n"):
            text += "\n"
        # Encode explicitly so the wire is always UTF-8 regardless of host locale.
        self._serial.write(text.encode("utf-8"))
        self._serial.flush()

    def drain(self) -> None:
        """Discard any buffered/captured input (both views). Use before a probe."""
        with self._lock:
            self._byte_buf.clear()
            self._line_buf.clear()
            self._lines.clear()
        # Also flush whatever the OS still holds so stale frames can't leak in.
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass


def open_link(port: Optional[str] = None, baud: int = DEFAULT_BAUD) -> CdcLink:
    """
    Open a CdcLink, auto-detecting the port when @p port is None.

    @param port  Explicit device name, or None to auto-detect by VID:PID.
    @param baud  Line rate (ignored by USB-CDC; passed through for UART bridges).
    @return      A live CdcLink.
    @raises RuntimeError  When auto-detect finds zero or multiple candidates.
    """
    # Resolve the port: explicit name wins; otherwise insist on a unique match.
    if port is None:
        port = auto_detect_port()
        if port is None:
            raise RuntimeError(
                "Could not auto-detect an ADSBin device (zero or multiple "
                "matching USB ports). Pass --port COMx explicitly, or run "
                "'list-ports' to see what's connected.")

    return CdcLink(port, baud=baud)


# A tiny self-check so 'python cdc_link.py' just lists ports — handy on Windows
# where a double-click or quick run confirms the device enumerates at all.
if __name__ == "__main__":
    # Force UTF-8 on the Windows console so box characters never mojibake.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    for info in list_ports():
        flag = "  [ADSBin]" if info.is_adsbin else ""
        vidpid = (f"{info.vid:04X}:{info.pid:04X}"
                  if info.vid is not None else "----:----")
        print(f"{info.device:<12} {vidpid}  {info.description}{flag}")
