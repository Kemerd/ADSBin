# -*- coding: utf-8 -*-
"""
canned_msgs.py - Ground-truth ADS-B frame corpus for the host CPR tests.

These are real, publicly-documented Mode-S Extended Squitter frames whose decoded
values are textbook-known, so the CPR + identification math can be checked against
independently-published answers rather than against our own decoder (which would
be circular). Each frame's 24-bit parity has a zero syndrome (verified in the
tests via mode_s_syndrome), so they are genuine valid frames, not synthetic.

Provenance:
    The airborne-position even/odd pair and the identification frame below are the
    canonical worked examples from the public ADS-B literature (the same pair used
    to illustrate global CPR in the open-access "The 1090 Megahertz Riddle" /
    mode-s.org teaching material and the original antirez dump1090 BSD docs). They
    are facts about the air-interface encoding, not licensed code.

Helpers here parse the raw ME bits into the structures the cpr_ref functions take.
Nothing in this file is firmware-specific; it is pure spec arithmetic.
"""

# ---------------------------------------------------------------------------
# The classic airborne-position even/odd pair for ICAO 0x40621D.
# Decoded ground truth (lat/lon resolved against the EVEN frame as the anchor):
#   latitude  = 52.2572021484375 deg
#   longitude =  3.91937255859375 deg
# ---------------------------------------------------------------------------
AIRBORNE_EVEN_HEX = "8D40621D58C382D690C8AC2863A7"
AIRBORNE_ODD_HEX = "8D40621D58C386435CC412692AD6"

# The reconciled ground truth (what global decode must produce when the EVEN
# frame is the anchor; this is the published answer for this pair).
AIRBORNE_GROUND_TRUTH = {
    "icao": 0x40621D,
    "lat": 52.2572021484375,
    "lon": 3.91937255859375,
    # The altitude encoded in these airborne-position frames is 38000 ft.
    "alt_ft": 38000,
}

# A reference position near the pair, used to exercise LOCAL decode. Decoding the
# even frame locally against this reference must land on the same ground truth.
LOCAL_REF = {"lat": 52.0, "lon": 4.0}

# ---------------------------------------------------------------------------
# A TC=4 aircraft identification frame for ICAO 0x4840D6.
# Decoded ground truth: callsign "KLM1023 " (trailing pad), emitter category
# group A, code 0 (TC 4 -> "A0" in the standard category table).
# ---------------------------------------------------------------------------
IDENT_HEX = "8D4840D6202CC371C32CE0576098"
IDENT_GROUND_TRUTH = {
    "icao": 0x4840D6,
    "callsign": "KLM1023",
    "type_code": 4,
}


# ---------------------------------------------------------------------------
# Parsing helpers (pure bit-twiddling on the raw frame bytes).
# ---------------------------------------------------------------------------
def frame_bytes(hexstr):
    """Convert a hex frame string into a bytes object."""
    return bytes.fromhex(hexstr)


def message_field(frame):
    """
    Return the 7-byte ME (Message, Extended squitter) field of a long DF17/18
    frame, i.e. bytes 4..10 inclusive.
    """
    return frame[4:11]


def type_code(frame):
    """ADS-B type code = the top 5 bits of the first ME byte."""
    return message_field(frame)[0] >> 3


def parse_cpr_frame(hexstr):
    """
    Parse an airborne-position frame into the dict cpr_ref expects:
        {odd, lat_cpr, lon_cpr}.

    The CPR fields live in the ME: the F (odd/even) bit, then the 17-bit encoded
    latitude and longitude. Bit layout per the airborne-position ME definition.
    """
    me = message_field(frame_bytes(hexstr))

    # F (CPR format) is bit 22 of the ME (0 = even, 1 = odd).
    odd = bool((me[2] >> 2) & 0x01)

    # 17-bit latitude: low 2 bits of ME byte 2, all of byte 3, top 7 bits of 4.
    lat_cpr = ((me[2] & 0x03) << 15) | (me[3] << 7) | (me[4] >> 1)

    # 17-bit longitude: low bit of ME byte 4, all of byte 5, all of byte 6.
    lon_cpr = ((me[4] & 0x01) << 16) | (me[5] << 8) | me[6]

    return {"odd": odd, "lat_cpr": lat_cpr, "lon_cpr": lon_cpr}


def parse_callsign(hexstr):
    """
    Decode the 8-character flight identification from a TC 1-4 ident frame.

    Each character is 6 bits packed across ME bytes 1..6, mapped through the
    Mode-S 6-bit charset (A-Z in 1..26, 0-9 in 48..57, space 32). Trailing pad
    characters are stripped, matching the firmware's callsign handling.
    """
    me = message_field(frame_bytes(hexstr))

    # The 48 bits of identification follow the TC+CA byte (ME byte 0).
    bits = 0
    for b in me[1:7]:
        bits = (bits << 8) | b

    charset = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"
    chars = []
    # Eight 6-bit characters, most significant first.
    for i in range(8):
        idx = (bits >> (42 - 6 * i)) & 0x3F
        chars.append(charset[idx])
    return "".join(chars).replace("_", " ").rstrip()


def mode_s_syndrome(hexstr):
    """
    Compute the 24-bit Mode-S CRC syndrome over a whole frame. A valid frame
    yields 0 (the parity is appended so the remainder vanishes).

    Generator polynomial: 0xFFF409 (the standard Mode-S 24-bit CRC). Used by the
    tests to assert the corpus frames are genuine.
    """
    rem = 0
    for byte in frame_bytes(hexstr):
        rem ^= byte << 16
        for _ in range(8):
            if rem & 0x800000:
                rem = ((rem << 1) ^ 0xFFF409) & 0xFFFFFF
            else:
                rem = (rem << 1) & 0xFFFFFF
    return rem
