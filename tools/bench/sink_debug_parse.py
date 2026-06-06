# -*- coding: utf-8 -*-
"""
sink_debug_parse.py — Parser for the firmware's ``sink_debug`` text protocol.

The device renders its live traffic table as UTF-8 lines over USB-CDC. The exact
token grammar is frozen in ``tools/bench/WIRE_CONTRACT.md`` §1; this module is the
host-side reader for it. It turns each ``ICAO=...`` line into a typed record and
each ``MSG ...`` verbose line into a raw-frame record, so the bench can assert on
decoded fields without re-implementing the grammar in every subcommand.

Grammar recap (WIRE_CONTRACT.md §1, frozen)::

    === ADSBIN TRAFFIC <count> @ <now_us> ===
    ICAO=<6hex> [CS=<callsign>] [LAT=<deg>] [LON=<deg>] [ALT=<ft>] [GS=<kt>]
        [TRK=<deg>] [VR=<fpm>] [CAT=<n>] [NIC=<n>] [NACp=<n>] [RNG=<nm>]
        [BRG=<deg>] MSGS=<n> SEEN=<age_ms>
    === END ===

    MSG ICAO=<6hex> DF=<n> TC=<n> RAW=<14hex|28hex>      (verbose only)

Parser rules (also frozen):
  * The parser keys on lines that START WITH ``ICAO=``. Header/footer ``=== ===``
    lines are optional context and may be ignored.
  * Tokens are space-separated ``KEY=VALUE``. Optional tokens are OMITTED entirely
    when invalid; ``ICAO=``, ``MSGS=`` and ``SEEN=`` are always present.

Pure text-in / dataclass-out — no serial, no GDL90. Windows-console / UTF-8 safe.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Dict, List, Optional


# ─────────────────────────────────────────────────────────────────────────────
#  Token typing table.
#
#  Maps each frozen KEY to the converter that turns its VALUE string into a
#  Python value. Keeping this declarative means adding a future token is a one-
#  line change and the parser stays grammar-driven rather than a wall of ifs.
# ─────────────────────────────────────────────────────────────────────────────
_INT = int
_FLOAT = float


def _str(v: str) -> str:
    """Identity converter for free-text tokens (e.g. the callsign)."""
    return v


# Converter per frozen token. Order here mirrors WIRE_CONTRACT.md §1 for clarity.
_TOKEN_CONVERTERS = {
    "ICAO": _str,    # Kept as the raw 6-hex string; numeric form is icao_int.
    "CS":   _str,    # 1..8 chars, no spaces.
    "LAT":  _FLOAT,  # Signed decimal degrees.
    "LON":  _FLOAT,  # Signed decimal degrees.
    "ALT":  _INT,    # Signed integer feet.
    "GS":   _INT,    # Unsigned integer knots.
    "TRK":  _INT,    # Unsigned integer degrees.
    "VR":   _INT,    # Signed integer feet/min.
    "CAT":  _INT,    # Emitter category integer.
    "NIC":  _INT,    # 0..11.
    "NACp": _INT,    # 0..11.
    "RNG":  _FLOAT,  # Decimal nautical miles.
    "BRG":  _INT,    # Unsigned integer degrees.
    "MSGS": _INT,    # Message count.
    "SEEN": _INT,    # Milliseconds since last heard.
}


@dataclass
class DebugTarget:
    """
    One parsed ``ICAO=...`` traffic line.

    Mirrors the live ``traffic_target_t`` view the firmware renders. Optional
    fields are None when the device omitted that token (position not yet
    resolved, no ownship, etc.), matching the ``has_*`` guards on the C struct.
    """
    icao: str                       # 6 uppercase hex digits, as on the wire.
    icao_int: int                   # Same address as an int, for convenience.
    msgs: int                       # MSGS — always present.
    seen_ms: int                    # SEEN — always present.

    callsign: Optional[str] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_ft: Optional[int] = None
    gs_kt: Optional[int] = None
    trk_deg: Optional[int] = None
    vr_fpm: Optional[int] = None
    cat: Optional[int] = None
    nic: Optional[int] = None
    nacp: Optional[int] = None
    rng_nm: Optional[float] = None
    brg_deg: Optional[int] = None

    # Any tokens we didn't model (forward-compat); empty in the frozen grammar.
    extra: Dict[str, str] = field(default_factory=dict)


@dataclass
class DebugMsgLine:
    """One parsed verbose ``MSG ...`` line (only present when sink is verbose)."""
    icao: str                       # 6 uppercase hex digits.
    icao_int: int
    df: int                         # Downlink format.
    tc: int                         # ADS-B type code.
    raw_hex: str                    # 14- or 28-char raw Mode-S frame hex.


@dataclass
class DebugBlock:
    """
    One full publish block delimited by the ``=== ... ===`` header/footer.

    The footer/header are optional per the grammar, so ``count``/``now_us`` are
    None when the harness only saw ICAO lines (e.g. a mid-stream capture).
    """
    targets: List[DebugTarget] = field(default_factory=list)
    msgs: List[DebugMsgLine] = field(default_factory=list)
    count: Optional[int] = None     # Header's declared target count, if seen.
    now_us: Optional[int] = None    # Header's adsbin_now_us() timestamp, if seen.


# Header regex: "=== ADSBIN TRAFFIC <count> @ <now_us> ===". now_us is int64 µs.
_HEADER_RE = re.compile(
    r"^===\s*ADSBIN TRAFFIC\s+(\d+)\s*@\s*(-?\d+)\s*===\s*$")

# Footer regex: "=== END ===".
_FOOTER_RE = re.compile(r"^===\s*END\s*===\s*$")


def _split_tokens(line: str) -> List[str]:
    """
    Split a token line on runs of whitespace into ``KEY=VALUE`` chunks.

    The grammar guarantees no spaces inside any value (the callsign is emitted
    only when space-free), so a plain whitespace split is exact and we avoid a
    brittle regex over the whole line.
    """
    return line.split()


def parse_target_line(line: str) -> Optional[DebugTarget]:
    """
    Parse a single ``ICAO=...`` traffic line into a :class:`DebugTarget`.

    @param line  One stripped text line from the device.
    @return      A DebugTarget, or None if the line is not an ICAO line. Lines
                 that start with ``ICAO=`` but are malformed raise ValueError so
                 a wire drift surfaces loudly rather than being silently dropped.
    """
    line = line.strip()

    # The frozen rule: we only key on lines that START WITH "ICAO=".
    if not line.startswith("ICAO="):
        return None

    # Collect every KEY=VALUE token; ignore stray bare words defensively.
    values: Dict[str, str] = {}
    for tok in _split_tokens(line):
        if "=" not in tok:
            continue
        key, _, val = tok.partition("=")
        values[key] = val

    # ICAO / MSGS / SEEN are mandatory per the contract — their absence means the
    # firmware diverged from the frozen grammar, which is exactly what we test.
    for required in ("ICAO", "MSGS", "SEEN"):
        if required not in values:
            raise ValueError(
                f"sink_debug line missing required token {required!r}: {line!r}")

    icao_hex = values["ICAO"].upper()

    # Build the record, converting each present token via its typed converter.
    tgt = DebugTarget(
        icao=icao_hex,
        icao_int=int(icao_hex, 16),
        msgs=int(values["MSGS"]),
        seen_ms=int(values["SEEN"]),
    )

    # Field name on the dataclass for each optional wire token.
    optional_map = {
        "CS":   "callsign",
        "LAT":  "lat_deg",
        "LON":  "lon_deg",
        "ALT":  "alt_ft",
        "GS":   "gs_kt",
        "TRK":  "trk_deg",
        "VR":   "vr_fpm",
        "CAT":  "cat",
        "NIC":  "nic",
        "NACp": "nacp",
        "RNG":  "rng_nm",
        "BRG":  "brg_deg",
    }

    # Populate each optional field that the device actually emitted.
    for key, attr in optional_map.items():
        if key in values:
            setattr(tgt, attr, _TOKEN_CONVERTERS[key](values[key]))

    # Stash any unmodelled tokens for forward compatibility (none in the frozen
    # grammar, but this keeps a future field from being silently lost).
    known = set(optional_map) | {"ICAO", "MSGS", "SEEN"}
    for key, val in values.items():
        if key not in known:
            tgt.extra[key] = val

    return tgt


# Verbose MSG line: "MSG ICAO=<6hex> DF=<n> TC=<n> RAW=<hex>".
_MSG_RE = re.compile(
    r"^MSG\s+ICAO=([0-9A-Fa-f]{6})\s+DF=(\d+)\s+TC=(\d+)\s+RAW=([0-9A-Fa-f]+)\s*$")


def parse_msg_line(line: str) -> Optional[DebugMsgLine]:
    """
    Parse a verbose ``MSG ...`` line into a :class:`DebugMsgLine`.

    @param line  One stripped text line.
    @return      A DebugMsgLine, or None if the line is not a MSG line.
    """
    m = _MSG_RE.match(line.strip())
    if not m:
        return None

    icao_hex = m.group(1).upper()
    raw_hex = m.group(4).upper()

    return DebugMsgLine(
        icao=icao_hex,
        icao_int=int(icao_hex, 16),
        df=int(m.group(2)),
        tc=int(m.group(3)),
        raw_hex=raw_hex,
    )


def parse_lines(lines: List[str]) -> List[DebugBlock]:
    """
    Parse a sequence of text lines into one or more :class:`DebugBlock`\\ s.

    Groups lines by the optional ``=== ... ===`` header/footer. Because the
    header/footer are optional and the stream can be joined mid-block, we also
    flush a block whenever a header opens a new one. ICAO and MSG lines that
    appear without any surrounding markers still land in a single trailing block.

    @param lines  Text lines (already newline-split, e.g. from CdcLink).
    @return       Blocks in stream order; never raises on non-matching lines.
    """
    blocks: List[DebugBlock] = []
    current = DebugBlock()
    saw_content = False     # Whether `current` has any real content to keep.

    def flush() -> None:
        # Commit the in-progress block if it carried anything meaningful.
        nonlocal current, saw_content
        if saw_content or current.count is not None:
            blocks.append(current)
        current = DebugBlock()
        saw_content = False

    for raw in lines:
        line = raw.strip()
        if not line:
            continue

        # A header both closes any prior block and opens a fresh one.
        hm = _HEADER_RE.match(line)
        if hm:
            flush()
            current.count = int(hm.group(1))
            current.now_us = int(hm.group(2))
            continue

        # A footer closes the current block.
        if _FOOTER_RE.match(line):
            flush()
            continue

        # Traffic line.
        if line.startswith("ICAO="):
            tgt = parse_target_line(line)
            if tgt is not None:
                current.targets.append(tgt)
                saw_content = True
            continue

        # Verbose per-message line.
        if line.startswith("MSG "):
            ml = parse_msg_line(line)
            if ml is not None:
                current.msgs.append(ml)
                saw_content = True
            continue

        # Anything else (GDL90 bytes that leaked onto the text side, IDF log
        # spam, blank context) is intentionally ignored — the grammar says only
        # ICAO/MSG lines are significant.

    # Commit whatever the final (possibly unterminated) block accumulated.
    flush()
    return blocks


def find_target(blocks: List[DebugBlock], icao: int) -> Optional[DebugTarget]:
    """
    Return the most-recent parsed target for @p icao across @p blocks, or None.

    @param blocks  Parsed blocks in stream order.
    @param icao    24-bit ICAO address as an int.
    @return        The latest DebugTarget matching ``icao``, or None if absent.
                   Used by ``inject-verify`` to find the injected aircraft.
    """
    found: Optional[DebugTarget] = None

    # Walk forward so the last (newest) matching record wins.
    for block in blocks:
        for tgt in block.targets:
            if tgt.icao_int == (icao & 0xFFFFFF):
                found = tgt

    return found
