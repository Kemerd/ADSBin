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
# unsampled temperature (or an absent GPS fix) parses cleanly to float('nan').
#
# The grammar matches the firmware's status_handle_line() emission EXACTLY and is
# named-group based so adding/removing a token is a one-line change on each side.
# (No shipped units exist yet, so the parser and firmware move together — there is
# no legacy line shape to stay compatible with.)
_FLOAT = r"[-+]?(?:nan|inf|\d+(?:\.\d+)?)"
_STATUS_RE = re.compile(
    r"===\s*ADSBIN STATUS\s+"
    r"DONGLES=(?P<dongles>\d+)\s+"
    r"PRESENT=(?P<present>[01])\s+"
    r"STREAMING=(?P<streaming>[01])\s+"
    r"B1090=(?P<b1090p>[01])/(?P<b1090s>[01])\s+"
    r"B978=(?P<b978p>[01])/(?P<b978s>[01])\s+"
    r"TEMP=(?P<temp>" + _FLOAT + r")\s+"
    r"PEAK=(?P<peak>" + _FLOAT + r")\s+"
    r"HEALTH=(?P<health>\w+)\s+"
    r"GPS=(?P<gps>\w+)\s+"
    r"GPSFIX=(?P<gpsfix>[01])\s+"
    r"POSVALID=(?P<posvalid>[01])\s+"
    r"LAT=(?P<lat>" + _FLOAT + r")\s+"
    r"LON=(?P<lon>" + _FLOAT + r")\s+"
    r"PRE=(?P<pre>\d+)\s+"
    r"FRM=(?P<frm>\d+)\s+"
    r"COK=(?P<cok>\d+)\s+"
    r"CFAIL=(?P<cfail>\d+)\s+"
    r"DFDROP=(?P<dfdrop>\d+)\s+"
    r"POS=(?P<pos>\d+)",
    re.IGNORECASE,
)


@dataclass
class UnitStatus:
    """One parsed ``=== ADSBIN STATUS ... ===`` line (WIRE_CONTRACT.md §4)."""
    dongles: int            # Count of adopted RTL-SDR dongles.
    present: bool           # Any dongle is enumerated.
    streaming: bool         # Any dongle is delivering IQ samples.

    # ── Per-band SDR recognition (which antenna/dongle is alive). ────────────
    b1090_present: bool     # The 1090ES traffic dongle enumerated.
    b1090_streaming: bool   # The 1090ES dongle is delivering IQ.
    b978_present: bool      # The 978 UAT weather dongle enumerated.
    b978_streaming: bool    # The 978 dongle is delivering IQ.

    temp_c: float           # Live die temperature, °C (NaN if no sample yet).
    peak_c: float           # Peak die temperature since boot, °C (NaN if never).
    health: str             # OK / NO_DONGLE / OVERTEMP.

    # ── GPS (ladder + live position). ────────────────────────────────────────
    gps: str                # NONE / FREE_RUNNING / NMEA_FIX / HOLDOVER / DISCIPLINED.
    gps_fix: bool           # A valid GPS ownship fix is live.
    pos_valid: bool         # The ownship lat/lon below are usable.
    lat_deg: float          # Ownship latitude,  WGS-84 (NaN if no fix).
    lon_deg: float          # Ownship longitude, WGS-84 (NaN if no fix).

    # ── 1090 decode ladder (RF → preamble → frame → CRC → position). ─────────
    preambles: int          # Cumulative Mode-S preambles detected.
    frames: int             # Cumulative candidate frames sliced.
    crc_ok: int             # Frames that passed the Mode-S CRC.
    crc_fail: int           # Frames that failed CRC.
    df_dropped: int         # Frames dropped at the DF gate.
    positions: int          # Resolved aircraft positions.

    raw: str                # The exact reply line, for diagnostics.

    @property
    def temp_valid(self) -> bool:
        """True when the live temperature is a real (finite) sample."""
        return math.isfinite(self.temp_c)

    @property
    def peak_valid(self) -> bool:
        """True when the peak temperature is a real (finite) sample."""
        return math.isfinite(self.peak_c)

    @property
    def pos_str(self) -> str:
        """Human position string: 'lat, lon' to 5 dp, or 'no fix'."""
        if self.pos_valid and math.isfinite(self.lat_deg) and math.isfinite(self.lon_deg):
            return f"{self.lat_deg:.5f}, {self.lon_deg:.5f}"
        return "no fix"


def parse_status_line(line: str) -> Optional[UnitStatus]:
    """
    Parse a single ``=== ADSBIN STATUS ... ===`` line into a :class:`UnitStatus`.

    @param line  One text line from the device (framing already stripped).
    @return      A UnitStatus, or None if the line is not a status line.
    """
    m = _STATUS_RE.search(line)
    if not m:
        return None

    g = m.group
    return UnitStatus(
        dongles=int(g("dongles")),
        present=g("present") == "1",
        streaming=g("streaming") == "1",
        b1090_present=g("b1090p") == "1",
        b1090_streaming=g("b1090s") == "1",
        b978_present=g("b978p") == "1",
        b978_streaming=g("b978s") == "1",
        temp_c=float(g("temp")),
        peak_c=float(g("peak")),
        health=g("health").upper(),
        gps=g("gps").upper(),
        gps_fix=g("gpsfix") == "1",
        pos_valid=g("posvalid") == "1",
        lat_deg=float(g("lat")),
        lon_deg=float(g("lon")),
        preambles=int(g("pre")),
        frames=int(g("frm")),
        crc_ok=int(g("cok")),
        crc_fail=int(g("cfail")),
        df_dropped=int(g("dfdrop")),
        positions=int(g("pos")),
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
