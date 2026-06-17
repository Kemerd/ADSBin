# -*- coding: utf-8 -*-
"""
status_query.py — Drive the firmware's ``+STATUS`` console command over USB-CDC.

``+STATUS`` is the manufacturing-QC unit-status query (WIRE_CONTRACT.md §4): the
bench sends a single ``+STATUS`` line and the device replies with one frozen
``=== ADSBIN STATUS ... ===`` line carrying SDR-dongle liveness, die temperature
(live + peak), coarse health, and the GPS ladder rung / fix. This module is the
thin request/response + parse layer over a :class:`cdc_link.CdcLink`, mirroring
``inject.py``: it sends the verb, scans the shared link for the status line, and
returns a typed :class:`UnitStatus`.

Pure protocol over an injected link object — unit-testable with a fake link.
Windows-console / UTF-8 safe.
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass
from typing import Optional


# The status line is keyed by this literal prefix on the shared console link, so a
# leading IDF log fragment or a spliced GDL90 byte can't be mistaken for it.
_STATUS_PREFIX = "=== ADSBIN STATUS"

# One regex over the frozen token set. We anchor on the prefix and capture each
# always-present KEY=VALUE; floats accept the printf NAN/INF spellings so an
# unsampled temperature parses cleanly to a Python float('nan').
_FLOAT = r"([-+]?(?:nan|inf|\d+(?:\.\d+)?))"
_STATUS_RE = re.compile(
    r"===\s*ADSBIN STATUS\s+"
    r"DONGLES=(\d+)\s+"
    r"PRESENT=([01])\s+"
    r"STREAMING=([01])\s+"
    r"TEMP=" + _FLOAT + r"\s+"
    r"PEAK=" + _FLOAT + r"\s+"
    r"HEALTH=(\w+)\s+"
    r"GPS=(\w+)\s+"
    r"GPSFIX=([01])",
    re.IGNORECASE,
)


@dataclass
class UnitStatus:
    """One parsed ``=== ADSBIN STATUS ... ===`` line (WIRE_CONTRACT.md §4)."""
    dongles: int            # Count of adopted RTL-SDR dongles.
    present: bool           # A dongle is enumerated.
    streaming: bool         # The dongle is delivering IQ samples.
    temp_c: float           # Live die temperature, °C (NaN if no sample yet).
    peak_c: float           # Peak die temperature since boot, °C (NaN if never).
    health: str             # OK / NO_DONGLE / OVERTEMP.
    gps: str                # NONE / FREE_RUNNING / NMEA_FIX / HOLDOVER / DISCIPLINED.
    gps_fix: bool           # A valid GPS ownship fix is live.
    raw: str                # The exact reply line, for diagnostics.

    @property
    def temp_valid(self) -> bool:
        """True when the live temperature is a real (finite) sample."""
        return math.isfinite(self.temp_c)

    @property
    def peak_valid(self) -> bool:
        """True when the peak temperature is a real (finite) sample."""
        return math.isfinite(self.peak_c)


def parse_status_line(line: str) -> Optional[UnitStatus]:
    """
    Parse a single ``=== ADSBIN STATUS ... ===`` line into a :class:`UnitStatus`.

    @param line  One text line from the device (framing already stripped).
    @return      A UnitStatus, or None if the line is not a status line.
    """
    m = _STATUS_RE.search(line)
    if not m:
        return None

    return UnitStatus(
        dongles=int(m.group(1)),
        present=m.group(2) == "1",
        streaming=m.group(3) == "1",
        temp_c=float(m.group(4)),
        peak_c=float(m.group(5)),
        health=m.group(6).upper(),
        gps=m.group(7).upper(),
        gps_fix=m.group(8) == "1",
        raw=line.strip(),
    )


def query_status(link, timeout: float = 2.0) -> Optional[UnitStatus]:
    """
    Send ``+STATUS`` and read back the device's status line.

    @param link     A live :class:`cdc_link.CdcLink` (or any object exposing
                    ``write_line`` / ``read_line(timeout)`` / optional ``drain``).
    @param timeout  Seconds to wait for the status line.
    @return         A :class:`UnitStatus`, or None on timeout / no reply.
    """
    # Clear stale text so a previous block's line can't be mistaken for ours.
    if hasattr(link, "drain"):
        link.drain()
    link.write_line("+STATUS")

    # The reply shares the link with the live sink_debug table + GDL90, so scan a
    # few lines for the first one that matches the status grammar.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        line = link.read_line(timeout=remaining)
        if line is None:
            break
        if _STATUS_PREFIX in line:
            st = parse_status_line(line)
            if st is not None:
                return st

    return None
