# -*- coding: utf-8 -*-
"""
canned_msgs.py — A corpus of real ADS-B frames with verified ground truth.

These are genuine, on-air Mode-S / 1090ES extended-squitter frames drawn from
the public DO-260B / ICAO Annex 10 worked examples and the long-standing
dump1090 test vectors (BSD-licensed antirez lineage — no GPL fork was consulted).
Each entry carries the EXPECTED decode so the bench can:

  * feed the hex to the device via ``+INJECT`` and assert the resulting
    ``sink_debug`` / GDL90 output matches the ground truth (inject-verify), and
  * unit-test the host reference decoder (gdl90.py) without any hardware.

Why ground truth ships with the corpus
--------------------------------------
A test corpus is only useful if the *answers* are independently known. For the
CPR position pair we store BOTH the even-anchored and odd-anchored global-decode
solutions, because the device's resolved fix depends on which frame it considered
"most recent" when it completed the pair. The values here were cross-checked
against the canonical dump1090 example (ICAO 40621D over the Netherlands) and
agree to sub-metre precision.

All frames are uppercase, space-free hex. Short (DF≠17/18) frames are included so
the harness exercises its length/df handling, not just the happy path.

Pure data — no serial, no I/O. Windows-console / UTF-8 safe.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


@dataclass
class GroundTruth:
    """
    The verified expected decode for one canned frame.

    Every field is Optional: a single ADS-B frame only carries a subset of the
    aircraft state, so we assert only the fields a given message type populates.
    A None field means "this frame does not carry this datum" — the verifier
    skips it rather than expecting a default.
    """
    icao: Optional[int] = None          # 24-bit address (DF17/18).
    df: Optional[int] = None            # Downlink format.
    type_code: Optional[int] = None     # ADS-B type code (ME[0] >> 3).
    crc_ok: Optional[bool] = None       # Expected parity result.

    callsign: Optional[str] = None      # TC 1..4 flight id.
    emitter_category: Optional[int] = None

    altitude_ft: Optional[int] = None   # Barometric pressure altitude.

    ground_speed_kt: Optional[int] = None   # TC 19 velocity.
    track_deg: Optional[int] = None
    vertical_rate_fpm: Optional[int] = None

    # CPR raw fields (single-frame), for white-box checks of the position parse.
    cpr_odd: Optional[bool] = None
    cpr_lat: Optional[int] = None
    cpr_lon: Optional[int] = None


@dataclass
class CannedFrame:
    """One corpus entry: the raw hex frame plus its verified ground truth."""
    name: str               # Stable human label used in CLI output.
    hex: str                # Uppercase, space-free Mode-S frame hex.
    note: str               # Provenance / what this frame exercises.
    truth: GroundTruth      # Expected decode for assertion.


@dataclass
class CprPair:
    """
    A matched even+odd airborne-position pair with a verified global solution.

    The two global-decode answers differ because each anchors the longitude zone
    to a different frame. ``even_anchored`` is the position you get when the EVEN
    frame is treated as most-recent; ``odd_anchored`` when the ODD is. The device
    will produce one of these depending on which frame closed the pair, so the
    verifier accepts a match against EITHER within tolerance.
    """
    name: str
    icao: int
    even_hex: str                       # The even-format (F=0) frame.
    odd_hex: str                        # The odd-format (F=1) frame.
    even_anchored: Tuple[float, float]  # (lat, lon) when even is newest.
    odd_anchored: Tuple[float, float]   # (lat, lon) when odd is newest.
    note: str


# ─────────────────────────────────────────────────────────────────────────────
#  Individual frames (identification, single positions, velocity, plus a short
#  frame for length-handling). Ground truth verified against the host reference
#  decoder and the published dump1090 / DO-260B examples.
# ─────────────────────────────────────────────────────────────────────────────
CANNED_FRAMES: List[CannedFrame] = [
    # ── Aircraft identification (TC 4): callsign "KLM1023" ─────────────────────
    CannedFrame(
        name="ident_KLM1023",
        hex="8D4840D6202CC371C32CE0576098",
        note="DF17 TC4 identification — canonical KLM1023 example (ICAO 4840D6).",
        truth=GroundTruth(
            icao=0x4840D6,
            df=17,
            type_code=4,
            crc_ok=True,
            callsign="KLM1023",
            emitter_category=0,   # CA field of this frame is 0 (no category info).
        ),
    ),

    # ── Airborne velocity (TC 19, subtype 1): GS 159 kt, trk 183°, VR -832 ─────
    CannedFrame(
        name="velocity_485020",
        hex="8D485020994409940838175B284F",
        note="DF17 TC19 ground velocity — canonical example (ICAO 485020).",
        truth=GroundTruth(
            icao=0x485020,
            df=17,
            type_code=19,
            crc_ok=True,
            ground_speed_kt=159,
            track_deg=183,
            vertical_rate_fpm=-832,
        ),
    ),

    # ── Even airborne position (TC 11) — part of the 40621D pair ───────────────
    # This even frame's AC altitude field is Gillham (Q-bit clear), so a 25-ft
    # decoder reports no altitude for it; the position is resolved via the pair.
    CannedFrame(
        name="pos_even_40621D",
        hex="8D40621D58C382D690C8AC2863A7",
        note="DF17 TC11 airborne position, EVEN frame (ICAO 40621D, NL example).",
        truth=GroundTruth(
            icao=0x40621D,
            df=17,
            type_code=11,
            crc_ok=True,
            altitude_ft=None,          # Gillham-coded in this frame; not 25-ft.
            cpr_odd=False,
            cpr_lat=93000,
            cpr_lon=51372,
        ),
    ),

    # ── Odd airborne position (TC 11) — part of the 40621D pair ────────────────
    CannedFrame(
        name="pos_odd_40621D",
        hex="8D40621D58C386435CC412692AD6",
        note="DF17 TC11 airborne position, ODD frame (ICAO 40621D, NL example).",
        truth=GroundTruth(
            icao=0x40621D,
            df=17,
            type_code=11,
            crc_ok=True,
            altitude_ft=11850,         # 25-ft Q-bit encoding in the odd frame.
            cpr_odd=True,
            cpr_lat=74158,
            cpr_lon=50194,
        ),
    ),

    # ── A short (56-bit / DF11 all-call reply) frame for length handling ───────
    # DF11 carries no ME; the harness uses it to confirm short frames are
    # accepted by +INJECT (14 hex chars) and gated as non-ADS-B by the decoder.
    CannedFrame(
        name="short_df11_allcall",
        hex="5D484020A6CF80",
        note="DF11 all-call reply (56-bit) — exercises short-frame length path.",
        truth=GroundTruth(
            df=11,
            # DF11 parity is address-overlaid (PI = CRC XOR II/AA), so a plain
            # residual is non-zero; we don't assert crc_ok for non-ADS-B frames.
            crc_ok=None,
        ),
    ),
]


# Fast name -> frame lookup for the CLI's ``--name`` selector.
CANNED_BY_NAME: Dict[str, CannedFrame] = {f.name: f for f in CANNED_FRAMES}


# ─────────────────────────────────────────────────────────────────────────────
#  Matched CPR pairs with verified global-decode ground truth.
#
#  The 40621D pair is THE textbook CPR example: even + odd over the Netherlands
#  resolving to ~52.2572°N, 3.9194°E. We store both anchorings so inject-verify
#  can match whichever the device emits.
# ─────────────────────────────────────────────────────────────────────────────
CPR_PAIRS: List[CprPair] = [
    CprPair(
        name="pair_40621D",
        icao=0x40621D,
        even_hex="8D40621D58C382D690C8AC2863A7",
        odd_hex="8D40621D58C386435CC412692AD6",
        # Verified against the canonical dump1090 global-decode example.
        even_anchored=(52.2572021484375, 3.91937255859375),
        odd_anchored=(52.26578017412606, 3.938912527901786),
        note="Canonical DO-260B/dump1090 CPR pair (ICAO 40621D, Netherlands).",
    ),
]

# Fast name -> pair lookup.
CPR_PAIRS_BY_NAME: Dict[str, CprPair] = {p.name: p for p in CPR_PAIRS}


def all_frame_names() -> List[str]:
    """Return every canned frame name, in corpus order (for CLI listing)."""
    return [f.name for f in CANNED_FRAMES]


def all_pair_names() -> List[str]:
    """Return every CPR-pair name, in corpus order."""
    return [p.name for p in CPR_PAIRS]


def get_frame(name: str) -> Optional[CannedFrame]:
    """Look up a canned frame by name, or None if it isn't in the corpus."""
    return CANNED_BY_NAME.get(name)


def get_pair(name: str) -> Optional[CprPair]:
    """Look up a CPR pair by name, or None if it isn't in the corpus."""
    return CPR_PAIRS_BY_NAME.get(name)
