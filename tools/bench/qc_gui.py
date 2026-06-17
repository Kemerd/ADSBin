# -*- coding: utf-8 -*-
"""
qc_gui.py — ADSBin manufacturing QC + live-status dashboard (Tkinter).

A single plug-and-play window for the production line: plug a freshly-built unit
into USB, run this, and the GUI auto-detects the device, runs a fast go/no-go QC
sweep, and then keeps a live status panel updating so the operator can watch the
unit breathe. One glance = ship / don't ship.

What it checks (the frozen wire contract, tools/bench/WIRE_CONTRACT.md)
----------------------------------------------------------------------
  PASS criteria (a unit must pass ALL of these to ship):
    1. USB enumerate ....... device auto-detected by VID:PID.
    2. Dongle streaming .... RF BLK=/s > 0 on the sink_debug RF line.
    3. Decode path ......... +INJECT a known frame (KLM1023) and confirm it
                             decodes with the right callsign — proves the whole
                             DSP→decode→traffic chain WITHOUT a plane overhead.
    4. GDL90 output ........ every captured GDL90 frame passes CRC.

  WARN (informational, never fails the unit):
    - Antenna hearing real RF ... PRE=/s > 0 (needs a live 1090 signal in range;
      a shielded bench legitimately hears nothing).

Live monitor (refreshes continuously after the QC sweep)
--------------------------------------------------------
  - SDR: dongle present / streaming, RF BLK/PRE/FRM per second, signal level.
  - Aircraft: live target count from the traffic table.
  - Temperature: live + peak die temp (via +STATUS).        [needs firmware +STATUS]
  - GPS: ladder rung + fix.                                  [needs firmware +STATUS]
  - Health: OK / NO_DONGLE / OVERTEMP.                       [needs firmware +STATUS]

Design
------
This is the PRESENTATION + ORCHESTRATION layer only. Every byte of protocol logic
is reused from the existing bench modules — cdc_link (transport), gdl90 (deframe +
CRC), inject (the +INJECT handshake), canned_msgs (ground truth), sink_debug_parse
(the RF + traffic grammar), and status_query (the +STATUS line). All serial I/O
runs on a background worker thread; the Tk UI thread only ever renders a snapshot,
so the window never freezes while the link is busy.

Pure stdlib + pyserial (Tkinter ships with CPython). Windows-console / UTF-8 safe.
"""

from __future__ import annotations

import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import tkinter as tk
from tkinter import font as tkfont

# Reuse the existing, tested bench modules — no protocol logic is duplicated here.
import cdc_link
import gdl90
import canned_msgs
import sink_debug_parse
from inject import inject_frame
from status_query import query_status, UnitStatus


# ─────────────────────────────────────────────────────────────────────────────
#  Apple-HIG-flavoured palette. A calm dark surface, San-Francisco-ish system
#  font, and a single saturated accent per state so the operator's eye lands on
#  the one thing that matters: the big PASS / FAIL verdict.
# ─────────────────────────────────────────────────────────────────────────────
COL_BG        = "#1c1c1e"   # window background (systemGray6 dark)
COL_CARD      = "#2c2c2e"   # raised card surface
COL_CARD_HI   = "#3a3a3c"   # card hover / divider
COL_TEXT      = "#f2f2f7"   # primary label
COL_SUBTLE    = "#8e8e93"   # secondary label (systemGray)
COL_GREEN     = "#30d158"   # pass / nominal
COL_RED       = "#ff453a"   # fail
COL_AMBER     = "#ff9f0a"   # warn / waiting
COL_BLUE      = "#0a84ff"   # accent / running

# The golden injection frame: carries callsign "KLM1023" (ICAO 4840D6) and needs
# no RF, so it deterministically proves the decode path end-to-end.
_QC_FRAME_NAME = "ident_KLM1023"

# QC capture windows (seconds). Kept short so a unit clears the bench fast.
_RF_WINDOW_S      = 2.0    # watch the RF BLK=/s line settle.
_INJECT_WINDOW_S  = 2.0    # capture sink_debug after the inject.
_GDL90_WINDOW_S   = 2.0    # capture GDL90 to CRC-check.

# Live-monitor refresh cadence (seconds) once QC has finished.
_MONITOR_PERIOD_S = 1.0


# ═════════════════════════════════════════════════════════════════════════════
#  Snapshot model — the single object the worker fills and the UI renders.
#  Everything the window draws comes from one of these, swapped atomically, so the
#  render path never reaches into live serial state.
# ═════════════════════════════════════════════════════════════════════════════
@dataclass
class CheckResult:
    """One QC check's outcome."""
    name: str
    state: str = "pending"     # pending / running / pass / fail / warn
    detail: str = ""


@dataclass
class LiveSnapshot:
    """Everything the live-monitor panel renders, filled by the worker."""
    # SDR / RF (from the sink_debug "RF BLK=.. PRE=.. FRM=.. SIG=.." line).
    rf_blk: Optional[int] = None
    rf_pre: Optional[int] = None
    rf_frm: Optional[int] = None
    rf_sig: Optional[int] = None
    aircraft: Optional[int] = None

    # Unit status (+STATUS line; None until/unless the firmware answers).
    status: Optional[UnitStatus] = None
    status_supported: Optional[bool] = None   # None=unknown, True/False once tried


@dataclass
class GuiState:
    """Top-level state the UI polls each tick (worker writes, UI reads)."""
    port: Optional[str] = None
    connected: bool = False
    phase: str = "searching"          # searching / qc / monitor / error
    message: str = ""                 # current narrator line
    verdict: Optional[bool] = None    # None=undecided, True=PASS, False=FAIL
    checks: List[CheckResult] = field(default_factory=list)
    live: LiveSnapshot = field(default_factory=LiveSnapshot)


# ═════════════════════════════════════════════════════════════════════════════
#  Worker thread — owns ALL serial I/O. Communicates with the UI only by pushing
#  fresh GuiState snapshots onto a queue, so the Tk thread never touches a port.
# ═════════════════════════════════════════════════════════════════════════════
class QcWorker(threading.Thread):
    """
    Background QC + monitor engine.

    Lifecycle: wait for an ADSBin port → open link → run the QC sweep → loop the
    live monitor forever (re-running QC on a fresh connect). All state is shipped
    to the UI as immutable :class:`GuiState` snapshots via ``out_queue``.
    """

    def __init__(self, out_queue: "queue.Queue[GuiState]",
                 port: Optional[str] = None) -> None:
        super().__init__(name="qc-worker", daemon=True)
        self._out = out_queue
        self._forced_port = port            # explicit --port, or None to auto-detect
        self._stop = threading.Event()
        self._rerun = threading.Event()     # operator pressed "Run again"
        self._state = GuiState()

    # ── public control (called from the UI thread) ─────────────────────────────

    def request_rerun(self) -> None:
        """Ask the worker to re-run the QC sweep on its next loop."""
        self._rerun.set()

    def stop(self) -> None:
        """Signal the worker to exit at the next opportunity."""
        self._stop.set()

    # ── snapshot publishing ────────────────────────────────────────────────────

    def _publish(self) -> None:
        """Ship a deep-enough copy of the current state to the UI queue."""
        # The UI only reads; we copy the mutable containers so a later worker
        # mutation can't race a half-drawn frame.
        snap = GuiState(
            port=self._state.port,
            connected=self._state.connected,
            phase=self._state.phase,
            message=self._state.message,
            verdict=self._state.verdict,
            checks=[CheckResult(c.name, c.state, c.detail) for c in self._state.checks],
            live=LiveSnapshot(
                rf_blk=self._state.live.rf_blk,
                rf_pre=self._state.live.rf_pre,
                rf_frm=self._state.live.rf_frm,
                rf_sig=self._state.live.rf_sig,
                aircraft=self._state.live.aircraft,
                status=self._state.live.status,
                status_supported=self._state.live.status_supported,
            ),
        )
        self._out.put(snap)

    def _set(self, **kw) -> None:
        """Update top-level state fields and publish."""
        for k, v in kw.items():
            setattr(self._state, k, v)
        self._publish()

    def _init_checks(self) -> None:
        """Reset the QC checklist to its pending baseline."""
        self._state.checks = [
            CheckResult("USB enumerate"),
            CheckResult("Dongle streaming"),
            CheckResult("Decode path (inject)"),
            CheckResult("GDL90 output"),
            CheckResult("Antenna RF (warn)"),
        ]
        self._state.verdict = None
        self._publish()

    def _check(self, idx: int, state: str, detail: str = "") -> None:
        """Update one checklist entry by index and publish."""
        self._state.checks[idx].state = state
        self._state.checks[idx].detail = detail
        self._publish()

    # ── main loop ──────────────────────────────────────────────────────────────

    def run(self) -> None:
        while not self._stop.is_set():
            # 1) Find a device (auto-detect by VID:PID, unless forced).
            port = self._wait_for_port()
            if port is None:
                return   # stop requested while searching

            # 2) Open the link and run the full session against it.
            try:
                link = cdc_link.open_link(port)
            except Exception as e:
                self._set(phase="error", connected=False,
                          message=f"Could not open {port}: {e}")
                # Back off, then retry the search (maybe a replug fixes it).
                if self._stop.wait(2.0):
                    return
                continue

            with link:
                self._set(port=port, connected=True)
                self._run_session(link)

            # Session ended (device vanished or stop). Loop back to searching.
            self._set(connected=False, phase="searching",
                      message="Device disconnected — waiting for a unit…")

    def _wait_for_port(self) -> Optional[str]:
        """Block until an ADSBin port appears (or stop), returning its name."""
        self._set(phase="searching", message="Searching for an ADSBin unit on USB…")
        while not self._stop.is_set():
            # An explicit port short-circuits detection but still waits for it to
            # actually exist, so "Run before plugging in" behaves sanely.
            if self._forced_port:
                names = [p.device for p in cdc_link.list_ports()]
                if self._forced_port in names:
                    return self._forced_port
            else:
                auto = cdc_link.auto_detect_port()
                if auto:
                    return auto
            if self._stop.wait(0.8):
                return None
        return None

    def _run_session(self, link: cdc_link.CdcLink) -> None:
        """Run one QC sweep then the live monitor until the device goes away."""
        # ── QC SWEEP ─────────────────────────────────────────────────────────
        self._set(phase="qc", message="Running QC sweep…")
        self._init_checks()
        try:
            self._do_qc(link)
        except Exception as e:
            # A mid-sweep I/O error means the device likely vanished; surface it
            # and fall through to the monitor loop, which will detect the drop.
            self._set(message=f"QC error: {e}")

        # ── LIVE MONITOR ─────────────────────────────────────────────────────
        self._set(phase="monitor")
        while not self._stop.is_set():
            # Operator asked for a fresh sweep — re-run it in place.
            if self._rerun.is_set():
                self._rerun.clear()
                self._set(phase="qc", message="Re-running QC sweep…")
                self._init_checks()
                try:
                    self._do_qc(link)
                except Exception as e:
                    self._set(message=f"QC error: {e}")
                self._set(phase="monitor")

            # One monitor refresh; if it raises, the device is gone — bail out so
            # run() returns to searching.
            try:
                self._refresh_monitor(link)
            except Exception:
                return
            if self._stop.wait(_MONITOR_PERIOD_S):
                return

    # ── QC sweep steps ─────────────────────────────────────────────────────────

    def _do_qc(self, link: cdc_link.CdcLink) -> None:
        """Run all four go/no-go checks plus the antenna warn, set the verdict."""
        # 1) USB enumerate — we are here, so the port opened. Record which one.
        self._check(0, "pass", self._state.port or "")

        # 2) Dongle streaming + (5) antenna RF, both read off the RF line.
        self._set(message="Checking SDR dongle is streaming…")
        self._check(1, "running")
        self._check(4, "running")
        link.drain()
        rf_lines = link.read_lines(_RF_WINDOW_S)
        blk, pre, frm, sig = _scan_rf(rf_lines)
        self._state.live.rf_blk, self._state.live.rf_pre = blk, pre
        self._state.live.rf_frm, self._state.live.rf_sig = frm, sig

        if blk is None:
            self._check(1, "fail", "no RF line seen — firmware not publishing?")
        elif blk > 0:
            self._check(1, "pass", f"BLK={blk}/s (radio streaming)")
        else:
            self._check(1, "fail", "BLK=0/s — dongle not delivering IQ")

        # Antenna RF is a WARN, never a fail: a shielded bench hears nothing.
        if pre is None:
            self._check(4, "warn", "no RF line to read PRE from")
        elif pre > 0:
            self._check(4, "pass", f"PRE={pre}/s (hearing live 1090)")
        else:
            self._check(4, "warn", "PRE=0/s (no live signal — OK on a bench)")

        # 3) Decode path — inject the golden frame, confirm its callsign decodes.
        self._set(message="Injecting test frame and verifying decode…")
        self._check(2, "running")
        ok, detail = self._verify_inject(link)
        self._check(2, "pass" if ok else "fail", detail)

        # 4) GDL90 output — every captured frame must pass CRC.
        self._set(message="Validating GDL90 output (CRC)…")
        self._check(3, "running")
        ok, detail = self._verify_gdl90(link)
        self._check(3, "pass" if ok else "fail", detail)

        # Verdict: ALL hard checks (indices 0..3) must pass; index 4 is a warn.
        hard = self._state.checks[:4]
        verdict = all(c.state == "pass" for c in hard)
        self._set(verdict=verdict,
                  message="QC complete." if verdict
                          else "QC FAILED — see the red check(s).")

    def _verify_inject(self, link: cdc_link.CdcLink) -> "tuple[bool, str]":
        """Inject the golden frame and confirm the expected callsign decodes."""
        frame = canned_msgs.get_frame(_QC_FRAME_NAME)
        if frame is None:
            return False, f"missing corpus frame {_QC_FRAME_NAME!r}"

        link.drain()
        res = inject_frame(link, frame.hex, timeout=2.0)
        if not res.ok:
            return False, f"device rejected +INJECT: {res.raw_reply or res.reason}"

        # Capture the next debug block(s) and find our injected aircraft.
        lines = link.read_lines(_INJECT_WINDOW_S)
        blocks = sink_debug_parse.parse_lines(lines)
        tgt = sink_debug_parse.find_target(blocks, frame.truth.icao)
        if tgt is None:
            return False, f"ICAO {frame.truth.icao:06X} never appeared after inject"

        want = frame.truth.callsign
        if want is not None and tgt.callsign != want:
            return False, f"callsign got {tgt.callsign!r}, want {want!r}"
        return True, f"{want} decoded (ICAO {frame.truth.icao:06X})"

    def _verify_gdl90(self, link: cdc_link.CdcLink) -> "tuple[bool, str]":
        """Capture the GDL90 stream and CRC-check every frame."""
        link.drain()
        stream = link.collect_bytes(_GDL90_WINDOW_S)
        frames = gdl90.deframe_stream(stream)
        if not frames:
            return False, f"no GDL90 frames in {len(stream)} bytes"

        bad = sum(1 for f in frames if not f.crc_ok)
        if bad:
            return False, f"{bad}/{len(frames)} frames FAILED CRC"
        return True, f"{len(frames)} frames, all CRC-OK"

    # ── live monitor refresh ───────────────────────────────────────────────────

    def _refresh_monitor(self, link: cdc_link.CdcLink) -> None:
        """One pass of the live panel: RF line, aircraft count, +STATUS."""
        # Pull ~1 s of lines and read the freshest RF + traffic figures.
        lines = link.read_lines(_MONITOR_PERIOD_S)
        blk, pre, frm, sig = _scan_rf(lines)
        if blk is not None:
            self._state.live.rf_blk = blk
            self._state.live.rf_pre = pre
            self._state.live.rf_frm = frm
            self._state.live.rf_sig = sig

        # Latest traffic block → live aircraft count (header "count" is authoritative).
        blocks = sink_debug_parse.parse_lines(lines)
        if blocks:
            last = blocks[-1]
            self._state.live.aircraft = (last.count if last.count is not None
                                         else len(last.targets))

        # +STATUS for temp / GPS / health. If the firmware doesn't answer once, we
        # remember that and stop nagging the link every second.
        if self._state.live.status_supported is not False:
            st = query_status(link, timeout=1.0)
            if st is not None:
                self._state.live.status = st
                self._state.live.status_supported = True
            elif self._state.live.status_supported is None:
                # First miss: mark unsupported so we don't keep querying a build
                # that predates +STATUS (older firmware just never replies).
                self._state.live.status_supported = False

        self._set(message="Live — monitoring unit.")


def _scan_rf(lines: List[str]):
    """
    Pull the freshest RF gauge values from a batch of sink_debug lines.

    The firmware emits one ``RF BLK=<n>/s PRE=<n>/s FRM=<n>/s SIG=<n>`` line per
    publish cycle. We scan for the LAST such line in the batch (newest wins) and
    return (blk, pre, frm, sig), any of which is None if no RF line was present.
    """
    blk = pre = frm = sig = None
    for ln in lines:
        s = ln.strip()
        if not s.startswith("RF "):
            continue
        # Tolerant token parse: "RF BLK=12/s PRE=3/s FRM=1/s SIG=4200".
        for tok in s.split():
            key, _, val = tok.partition("=")
            num = val.split("/", 1)[0]   # strip the "/s" suffix on rate tokens
            try:
                n = int(num)
            except ValueError:
                continue
            if key == "BLK":
                blk = n
            elif key == "PRE":
                pre = n
            elif key == "FRM":
                frm = n
            elif key == "SIG":
                sig = n
    return blk, pre, frm, sig


# ═════════════════════════════════════════════════════════════════════════════
#  The Tkinter window. Pure rendering: it polls the worker's snapshot queue on a
#  Tk timer and redraws. It never blocks and never touches a serial port.
# ═════════════════════════════════════════════════════════════════════════════
class QcApp:
    """The QC dashboard window."""

    def __init__(self, root: tk.Tk, worker: QcWorker,
                 in_queue: "queue.Queue[GuiState]") -> None:
        self._root = root
        self._worker = worker
        self._q = in_queue
        self._latest = GuiState()

        root.title("ADSBin · Unit QC")
        root.configure(bg=COL_BG)
        root.minsize(720, 640)

        # ── Fonts: a system-native, weight-graded type ramp (HIG-ish). ────────
        base = "Segoe UI" if sys.platform.startswith("win") else "Helvetica"
        self._f_hero    = tkfont.Font(family=base, size=46, weight="bold")
        self._f_title   = tkfont.Font(family=base, size=20, weight="bold")
        self._f_body    = tkfont.Font(family=base, size=12)
        self._f_body_b  = tkfont.Font(family=base, size=12, weight="bold")
        self._f_small   = tkfont.Font(family=base, size=10)
        self._f_mono    = tkfont.Font(family="Consolas" if sys.platform.startswith("win")
                                      else "Menlo", size=12)

        self._build_ui()
        # Begin the render pump (~30 fps feel; cheap, just drains a queue).
        self._root.after(50, self._pump)

    # ── layout ─────────────────────────────────────────────────────────────────

    def _card(self, parent: tk.Widget, pad: int = 16) -> tk.Frame:
        """A rounded-ish raised surface (flat Tk, but HIG spacing/colour)."""
        c = tk.Frame(parent, bg=COL_CARD, bd=0, highlightthickness=0)
        c.configure(padx=pad, pady=pad)
        return c

    def _build_ui(self) -> None:
        root = self._root
        outer = tk.Frame(root, bg=COL_BG, padx=22, pady=22)
        outer.pack(fill="both", expand=True)

        # ── Header: app name + connection chip. ───────────────────────────────
        header = tk.Frame(outer, bg=COL_BG)
        header.pack(fill="x")
        tk.Label(header, text="ADSBin Unit QC", font=self._f_title,
                 bg=COL_BG, fg=COL_TEXT).pack(side="left")
        self._conn = tk.Label(header, text="● searching", font=self._f_small,
                              bg=COL_BG, fg=COL_AMBER)
        self._conn.pack(side="right", pady=(8, 0))

        # ── Hero verdict card: the giant PASS / FAIL the operator reads first. ─
        hero = self._card(outer, pad=24)
        hero.pack(fill="x", pady=(16, 14))
        self._verdict_lbl = tk.Label(hero, text="…", font=self._f_hero,
                                     bg=COL_CARD, fg=COL_SUBTLE)
        self._verdict_lbl.pack()
        self._narrator = tk.Label(hero, text="Starting up…", font=self._f_body,
                                  bg=COL_CARD, fg=COL_SUBTLE)
        self._narrator.pack(pady=(6, 0))

        # ── QC checklist card. ────────────────────────────────────────────────
        checks_card = self._card(outer)
        checks_card.pack(fill="x", pady=(0, 14))
        tk.Label(checks_card, text="QC CHECKS", font=self._f_small,
                 bg=COL_CARD, fg=COL_SUBTLE).pack(anchor="w", pady=(0, 8))
        self._check_rows: List[Dict[str, tk.Label]] = []
        for _ in range(5):
            row = tk.Frame(checks_card, bg=COL_CARD)
            row.pack(fill="x", pady=3)
            dot = tk.Label(row, text="○", font=self._f_body_b,
                           bg=COL_CARD, fg=COL_SUBTLE, width=2)
            dot.pack(side="left")
            name = tk.Label(row, text="", font=self._f_body_b,
                            bg=COL_CARD, fg=COL_TEXT, width=22, anchor="w")
            name.pack(side="left")
            detail = tk.Label(row, text="", font=self._f_small,
                              bg=COL_CARD, fg=COL_SUBTLE, anchor="w")
            detail.pack(side="left", fill="x", expand=True)
            self._check_rows.append({"dot": dot, "name": name, "detail": detail})

        # ── Live monitor card: SDR / aircraft / temp / GPS tiles. ─────────────
        live_card = self._card(outer)
        live_card.pack(fill="both", expand=True)
        tk.Label(live_card, text="LIVE STATUS", font=self._f_small,
                 bg=COL_CARD, fg=COL_SUBTLE).pack(anchor="w", pady=(0, 10))

        grid = tk.Frame(live_card, bg=COL_CARD)
        grid.pack(fill="both", expand=True)
        for c in range(2):
            grid.columnconfigure(c, weight=1, uniform="tiles")

        # Each tile: a big value over a small caption. Built once, updated live.
        self._tiles: Dict[str, Dict[str, tk.Label]] = {}
        specs = [
            ("sdr",      "SDR DONGLE",  0, 0),
            ("rf",       "RF (per sec)",0, 1),
            ("aircraft", "AIRCRAFT",    1, 0),
            ("temp",     "TEMPERATURE", 1, 1),
            ("gps",      "GPS",         2, 0),
            ("health",   "HEALTH",      2, 1),
        ]
        for key, caption, r, c in specs:
            tile = tk.Frame(grid, bg=COL_CARD_HI, padx=14, pady=12)
            tile.grid(row=r, column=c, sticky="nsew", padx=5, pady=5)
            val = tk.Label(tile, text="--", font=self._f_title,
                           bg=COL_CARD_HI, fg=COL_TEXT, anchor="w")
            val.pack(anchor="w")
            cap = tk.Label(tile, text=caption, font=self._f_small,
                           bg=COL_CARD_HI, fg=COL_SUBTLE, anchor="w")
            cap.pack(anchor="w")
            sub = tk.Label(tile, text="", font=self._f_small,
                           bg=COL_CARD_HI, fg=COL_SUBTLE, anchor="w")
            sub.pack(anchor="w")
            self._tiles[key] = {"val": val, "cap": cap, "sub": sub}

        # ── Footer: the one operator action — re-run. ─────────────────────────
        footer = tk.Frame(outer, bg=COL_BG)
        footer.pack(fill="x", pady=(16, 0))
        self._rerun_btn = tk.Button(
            footer, text="Run QC again", font=self._f_body_b,
            bg=COL_BLUE, fg="white", activebackground="#0060df",
            activeforeground="white", relief="flat", bd=0,
            padx=18, pady=10, cursor="hand2", command=self._on_rerun)
        self._rerun_btn.pack(side="right")
        tk.Label(footer, text="Plug a unit in to begin · re-runs automatically on reconnect",
                 font=self._f_small, bg=COL_BG, fg=COL_SUBTLE).pack(side="left", pady=(10, 0))

    # ── events ───────────────────────────────────────────────────────────────

    def _on_rerun(self) -> None:
        """Operator pressed the re-run button."""
        self._worker.request_rerun()

    # ── render pump ────────────────────────────────────────────────────────────

    def _pump(self) -> None:
        """Drain the snapshot queue, render the newest, reschedule."""
        try:
            while True:
                self._latest = self._q.get_nowait()
        except queue.Empty:
            pass
        self._render(self._latest)
        self._root.after(50, self._pump)

    def _render(self, s: GuiState) -> None:
        # Connection chip.
        if s.connected:
            self._conn.configure(text=f"● {s.port}", fg=COL_GREEN)
        elif s.phase == "error":
            self._conn.configure(text="● error", fg=COL_RED)
        else:
            self._conn.configure(text="● searching", fg=COL_AMBER)

        # Hero verdict.
        if s.verdict is True:
            self._verdict_lbl.configure(text="PASS", fg=COL_GREEN)
        elif s.verdict is False:
            self._verdict_lbl.configure(text="FAIL", fg=COL_RED)
        elif s.phase == "qc":
            self._verdict_lbl.configure(text="testing…", fg=COL_BLUE)
        else:
            self._verdict_lbl.configure(text="…", fg=COL_SUBTLE)
        self._narrator.configure(text=s.message or "")

        # Checklist rows.
        palette = {
            "pending": (COL_SUBTLE, "○"),
            "running": (COL_BLUE,   "◐"),
            "pass":    (COL_GREEN,  "●"),
            "fail":    (COL_RED,    "✕"),
            "warn":    (COL_AMBER,  "▲"),
        }
        for i, row in enumerate(self._check_rows):
            if i < len(s.checks):
                chk = s.checks[i]
                colour, glyph = palette.get(chk.state, (COL_SUBTLE, "○"))
                row["dot"].configure(text=glyph, fg=colour)
                row["name"].configure(text=chk.name)
                row["detail"].configure(text=chk.detail)
            else:
                row["dot"].configure(text="○", fg=COL_SUBTLE)
                row["name"].configure(text="")
                row["detail"].configure(text="")

        self._render_live(s.live)

    def _render_live(self, live: LiveSnapshot) -> None:
        st = live.status

        # SDR tile.
        if st is not None:
            present = st.present
            val = "ON" if (present and st.streaming) else ("IDLE" if present else "OFF")
            colour = COL_GREEN if (present and st.streaming) else (COL_AMBER if present else COL_RED)
            sub = f"{st.dongles} dongle(s)"
        else:
            # No +STATUS — fall back to the RF needle for liveness.
            streaming = (live.rf_blk or 0) > 0
            val = "ON" if streaming else "?"
            colour = COL_GREEN if streaming else COL_SUBTLE
            sub = "from RF stream"
        self._set_tile("sdr", val, sub, colour)

        # RF tile (BLK/PRE/FRM per second).
        if live.rf_blk is not None:
            self._set_tile("rf", f"{live.rf_blk}",
                           f"PRE {live.rf_pre}  FRM {live.rf_frm}  SIG {live.rf_sig}",
                           COL_GREEN if live.rf_blk > 0 else COL_RED)
        else:
            self._set_tile("rf", "--", "BLK / PRE / FRM", COL_SUBTLE)

        # Aircraft tile.
        if live.aircraft is not None:
            self._set_tile("aircraft", str(live.aircraft),
                           "live targets", COL_TEXT)
        else:
            self._set_tile("aircraft", "--", "live targets", COL_SUBTLE)

        # Temperature tile (+STATUS only).
        if st is not None and st.temp_valid:
            peak = f"peak {st.peak_c:.1f}°C" if st.peak_valid else "peak --"
            tcol = COL_RED if st.temp_c >= 85 else (COL_AMBER if st.temp_c >= 75 else COL_GREEN)
            self._set_tile("temp", f"{st.temp_c:.1f}°C", peak, tcol)
        elif live.status_supported is False:
            self._set_tile("temp", "n/a", "needs +STATUS firmware", COL_SUBTLE)
        else:
            self._set_tile("temp", "--", "awaiting sample", COL_SUBTLE)

        # GPS tile (+STATUS only).
        if st is not None:
            gcol = COL_GREEN if st.gps_fix else (COL_AMBER if st.gps != "NONE" else COL_SUBTLE)
            self._set_tile("gps", st.gps.replace("_", " ").title(),
                           "fix" if st.gps_fix else "no fix", gcol)
        elif live.status_supported is False:
            self._set_tile("gps", "n/a", "needs +STATUS firmware", COL_SUBTLE)
        else:
            self._set_tile("gps", "--", "awaiting status", COL_SUBTLE)

        # Health tile (+STATUS only).
        if st is not None:
            hcol = {"OK": COL_GREEN, "OVERTEMP": COL_RED,
                    "NO_DONGLE": COL_RED}.get(st.health, COL_AMBER)
            self._set_tile("health", st.health.replace("_", " ").title(), "", hcol)
        elif live.status_supported is False:
            self._set_tile("health", "n/a", "needs +STATUS firmware", COL_SUBTLE)
        else:
            self._set_tile("health", "--", "awaiting status", COL_SUBTLE)

    def _set_tile(self, key: str, value: str, sub: str, colour: str) -> None:
        """Update one live tile's big value + sub-caption + accent colour."""
        t = self._tiles[key]
        t["val"].configure(text=value, fg=colour)
        t["sub"].configure(text=sub)


# ═════════════════════════════════════════════════════════════════════════════
#  Entry point
# ═════════════════════════════════════════════════════════════════════════════
def main(argv: Optional[List[str]] = None) -> int:
    """Launch the QC GUI. Optional first arg is an explicit port (e.g. COM7)."""
    # UTF-8 console so any stderr glyphs are clean on Windows.
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

    argv = list(sys.argv[1:] if argv is None else argv)
    forced_port = argv[0] if argv else None

    # Wire the worker → UI snapshot channel, then start both halves.
    snap_q: "queue.Queue[GuiState]" = queue.Queue()
    worker = QcWorker(snap_q, port=forced_port)

    root = tk.Tk()
    app = QcApp(root, worker, snap_q)
    worker.start()

    # Clean teardown: signal the worker to stop when the window closes.
    def _on_close() -> None:
        worker.stop()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", _on_close)

    root.mainloop()
    worker.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
