# -*- coding: utf-8 -*-
"""
sink_debug_ref.py - Host reference for the sink_debug traffic-table wire format.

This mirrors §1 of tools/bench/WIRE_CONTRACT.md and the emitter in
components/sinks (sink_debug). It provides:

    render_target(target)  -> the exact "ICAO=... MSGS=... SEEN=..." line the
                              firmware emits for one live target.
    parse_target(line)     -> dict of the tokens on such a line.

Having both directions in one place lets the unit tests prove a clean
render -> parse -> render round-trip, and lets the live bench harness reuse the
SAME parser the tests validate (so the harness can never silently drift from the
contract).

Token contract recap (authoritative copy lives in WIRE_CONTRACT.md):
    ICAO=<6hex> [CS=..] [LAT=..] [LON=..] [ALT=..] [GS=..] [TRK=..] [VR=..]
        [CAT=..] [NIC=..] [NACp=..] [RNG=..] [BRG=..] MSGS=<n> SEEN=<age_ms>

    - The parser keys on lines that START WITH "ICAO=".
    - Tokens are space-separated KEY=VALUE in the order above.
    - Optional tokens are OMITTED ENTIRELY when the field is invalid.
    - ICAO, MSGS, SEEN are ALWAYS present.
    - Value formats: ICAO 6 upper-hex; LAT/LON >=4 fractional digits; ALT/VR
      signed int; GS/TRK/BRG unsigned int; RNG decimal NM; CAT/NIC/NACp ints.
"""

# Canonical token order. render_target walks this so emitted lines always match
# the contract's stated ordering, and the tests can assert against it directly.
TOKEN_ORDER = [
    "ICAO", "CS", "LAT", "LON", "ALT", "GS", "TRK", "VR",
    "CAT", "NIC", "NACp", "RNG", "BRG", "MSGS", "SEEN",
]


def render_target(t):
    """
    Render one traffic-table target dict into its sink_debug line.

    @param t  dict with at least: icao (int), msgs (int), seen_ms (int).
              Optional keys (emitted only when present AND not None):
              callsign, lat, lon, alt_ft, gs_kt, trk_deg, vr_fpm, cat,
              nic, nacp, rng_nm, brg_deg.
    @return   The line WITHOUT a trailing newline.
    """
    parts = []

    # ICAO: always present, 6 uppercase zero-padded hex digits.
    parts.append("ICAO=%06X" % (t["icao"] & 0xFFFFFF))

    # Callsign: 1-8 chars, no spaces, only when known.
    cs = t.get("callsign")
    if cs:
        parts.append("CS=%s" % cs)

    # Position: at least 4 fractional digits, signed.
    if t.get("lat") is not None:
        parts.append("LAT=%.4f" % t["lat"])
    if t.get("lon") is not None:
        parts.append("LON=%.4f" % t["lon"])

    # Altitude / vertical rate: signed integers.
    if t.get("alt_ft") is not None:
        parts.append("ALT=%d" % t["alt_ft"])

    # Ground speed / track: unsigned integers.
    if t.get("gs_kt") is not None:
        parts.append("GS=%d" % t["gs_kt"])
    if t.get("trk_deg") is not None:
        parts.append("TRK=%d" % t["trk_deg"])

    if t.get("vr_fpm") is not None:
        parts.append("VR=%d" % t["vr_fpm"])

    # Category / integrity codes.
    if t.get("cat") is not None:
        parts.append("CAT=%d" % t["cat"])
    if t.get("nic") is not None:
        parts.append("NIC=%d" % t["nic"])
    if t.get("nacp") is not None:
        parts.append("NACp=%d" % t["nacp"])

    # Relative geometry: only when ownship is set.
    if t.get("rng_nm") is not None:
        parts.append("RNG=%.1f" % t["rng_nm"])
    if t.get("brg_deg") is not None:
        parts.append("BRG=%d" % t["brg_deg"])

    # MSGS and SEEN: always present, trailing.
    parts.append("MSGS=%d" % t["msgs"])
    parts.append("SEEN=%d" % t["seen_ms"])

    return " ".join(parts)


# Integer-typed tokens, used by the parser to coerce values back to ints.
_INT_TOKENS = {"ALT", "VR", "GS", "TRK", "BRG", "CAT", "NIC", "NACp",
               "MSGS", "SEEN"}
# Float-typed tokens.
_FLOAT_TOKENS = {"LAT", "LON", "RNG"}


def parse_target(line):
    """
    Parse a sink_debug target line into a dict of its tokens.

    Only lines that start with "ICAO=" are valid targets; anything else (the
    "=== ... ===" header/footer, blank lines) returns None so a caller can scan a
    whole block and pick out the targets, exactly as the live harness does.

    @param line  One line of text (newline already stripped or not).
    @return      dict keyed by token name (ICAO -> int, LAT -> float, CS -> str,
                 ...) or None if the line is not a target row.
    """
    line = line.strip()
    if not line.startswith("ICAO="):
        return None

    out = {}
    for tok in line.split():
        # Be forgiving about stray tokens but require a single '=' split.
        if "=" not in tok:
            continue
        key, _, value = tok.partition("=")

        if key == "ICAO":
            out["ICAO"] = int(value, 16)
        elif key in _INT_TOKENS:
            out[key] = int(value)
        elif key in _FLOAT_TOKENS:
            out[key] = float(value)
        else:
            # CS and any future string token pass through untouched.
            out[key] = value

    return out
