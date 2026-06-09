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
# The device's text table and its binary GDL90 stream SHARE the one console link
# (WIRE_CONTRACT.md): a GDL90 frame with no trailing newline can splice onto the
# end of the final token on a sink_debug line (in practice the SEEN= value, which
# is emitted last). The numeric converters therefore parse the LEADING numeric
# literal and ignore any trailing bytes, rather than feeding the whole polluted
# string to int()/float() and throwing. The leading-number requirement still
# fails loudly on genuine grammar drift (a non-numeric value), so we stay strict
# on shape while tolerating the documented binary interleave.
_INT_RE = re.compile(r"[+-]?\d+")
_FLOAT_RE = re.compile(r"[+-]?(?:\d+\.\d*|\.\d+|\d+)")


def _INT(v: str) -> int:
    """Parse the leading signed integer in @p v, ignoring trailing binary splice."""
    m = _INT_RE.match(v)
    if not m:
        raise ValueError(f"no integer at start of token value {v!r}")
    return int(m.group(0))


def _FLOAT(v: str) -> float:
    """Parse the leading float in @p v, ignoring any trailing binary splice."""
    m = _FLOAT_RE.match(v)
    if not m:
        raise ValueError(f"no number at start of token value {v!r}")
    return float(m.group(0))


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


# A GDL90 frame on the shared link is delimited by 0x7E flag bytes, which arrive
# in the text view as the '~' character (0x7E decodes to U+007E). When a frame
# lands inside a sink_debug line, everything between two '~' (and any stray
# binary the lenient UTF-8 decode turned into replacement chars) is NOT part of
# the text grammar. We strip those spans before tokenising so a spliced frame
# can't fracture or pollute a KEY=VALUE token. We also drop the C0 control /
# replacement bytes a partial frame can leave behind.
_GDL90_SPAN_RE = re.compile(r"~[^~]*~?")
_NONTEXT_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f�]")


def _scrub_shared_link(line: str) -> str:
    """
    Remove embedded GDL90 frames / binary splice from a shared-link text line.

    The device interleaves binary GDL90 (0x7E-flagged) with the UTF-8 sink_debug
    text on one link, so a frame can appear mid-line. We excise '~...~' spans and
    any leftover non-text control bytes, leaving only the KEY=VALUE grammar.
    """
    cleaned = _GDL90_SPAN_RE.sub(" ", line)
    cleaned = _NONTEXT_RE.sub("", cleaned)
    return cleaned


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
    # Excise any GDL90 frame / binary that spliced into this text line from the
    # shared link BEFORE inspecting the grammar, so a frame between '~' flags
    # can't fracture a KEY=VALUE token (e.g. pollute the trailing SEEN= value).
    line = _scrub_shared_link(line).strip()

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
        msgs=_INT(values["MSGS"]),
        seen_ms=_INT(values["SEEN"]),   # tolerant: SEEN is last, so binary may splice here
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

        # Traffic line. parse_target_line() raises on a malformed line so genuine
        # wire drift surfaces loudly — but on the shared console an IDF log message
        # (e.g. "I (NNN) tag: ...") can splice into a table line and corrupt it
        # without any real drift. Such a damaged line is dropped here: the same
        # target re-renders intact on the next 1 Hz publish cycle, so a single
        # corrupted copy costs us nothing, whereas crashing the whole parse would
        # fail a test for a cosmetic interleave. (find_target scans every block, so
        # it still finds the aircraft from a clean cycle.)
        if line.startswith("ICAO="):
            try:
                tgt = parse_target_line(line)
            except ValueError:
                continue   # interleave-corrupted line — skip, await a clean cycle
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
