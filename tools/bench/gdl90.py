# -*- coding: utf-8 -*-
"""
gdl90.py — Pure GDL90 + Mode-S/CPR reference implementation for the ADSBin bench.

This module is the host-side mirror of the firmware's GDL90 encoder
(``components/sinks/include/gdl90_encoder.h``) and CPR math
(``components/modes_decode/include/cpr.h``). It exists so the Python bench can
*independently* re-derive every wire value and assert the device agrees with us
byte-for-byte. Nothing here touches a serial port or the firmware — it is all
pure functions over bytes, which keeps it trivially unit-testable on a PC.

Everything is built strictly against ``tools/bench/WIRE_CONTRACT.md`` (the frozen
ABI) and the two frozen headers above. If a value here disagrees with the device,
the *device* is wrong — that is the entire point of an independent reference.

References (public specs only — no GPL source was consulted):
  * Garmin "GDL 90 Data Interface Specification" 560-1058-00 Rev A.
  * ForeFlight "GDL90 Extended Specification".
  * ICAO Annex 10 Vol IV / RTCA DO-260B for Mode-S + CPR field layouts.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


# ─────────────────────────────────────────────────────────────────────────────
#  GDL90 framing constants — these MUST match the table in WIRE_CONTRACT.md §3
#  and the FRAMING CONTRACT block in gdl90_encoder.h. Do not "tidy" the values.
# ─────────────────────────────────────────────────────────────────────────────
GDL90_FLAG = 0x7E          # Start/end-of-frame marker.
GDL90_ESCAPE = 0x7D        # Byte-stuffing escape prefix.
GDL90_ESCAPE_XOR = 0x20    # Stuffed byte == original XOR 0x20.

# Message ids carried in the first un-stuffed byte of every frame.
GDL90_ID_HEARTBEAT = 0x00
GDL90_ID_OWNSHIP = 0x0A
GDL90_ID_TRAFFIC = 0x14


# ═════════════════════════════════════════════════════════════════════════════
#  CRC-16 (CCITT table variant) — WIRE_CONTRACT.md §3 / gdl90_encoder.h.
#
#  GDL90 uses the same precomputed 256-entry CRC-16/CCITT table that Garmin's
#  spec publishes verbatim. We build the table once at import using the standard
#  polynomial 0x1021 (left-shifting / MSB-first form), which reproduces Garmin's
#  table exactly. The CRC runs over the UN-stuffed id+payload only.
# ═════════════════════════════════════════════════════════════════════════════
def _build_crc16_table() -> List[int]:
    """Construct the GDL90 CRC-16/CCITT lookup table (poly 0x1021, MSB-first)."""
    table: List[int] = []
    # One table entry per possible leading byte value 0..255.
    for i in range(256):
        crc = (i << 8) & 0xFFFF
        # Process the 8 bits of this byte, MSB first, as the spec's table does.
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


# Built once; reused for every frame the harness verifies.
_CRC16_TABLE = _build_crc16_table()


def crc16(payload: bytes) -> int:
    """
    Compute the GDL90 CRC-16 over an UN-framed id+payload buffer.

    @param payload  The message id byte followed by its payload (pre-stuffing).
    @return         The 16-bit CRC; the wire carries this little-endian.
    """
    crc = 0
    # Canonical GDL90 fold (Garmin spec appendix, matched byte-for-byte to the
    # firmware's gdl90_crc16 in components/sinks/gdl90_encoder.c):
    #
    #     crc = table[crc >> 8] ^ (crc << 8) ^ b
    #
    # NOTE: this is NOT the CRC-16/CCITT-FALSE "index-XOR" form
    # (table[(crc>>8) ^ b]). GDL90 XORs the data byte into the *running* CRC
    # AFTER the table lookup, not into the table index. Using the wrong variant
    # makes every real device frame look CRC-failed, which is exactly the bug we
    # exist to catch — so we mirror the device precisely.
    for b in payload:
        crc = (_CRC16_TABLE[(crc >> 8) & 0xFF] ^ ((crc << 8) & 0xFFFF) ^ b) & 0xFFFF
    return crc & 0xFFFF


# ═════════════════════════════════════════════════════════════════════════════
#  Byte-stuffing / de-stuffing (WIRE_CONTRACT.md §3, step 2 of the deframe).
#
#  Stuffing applies to id+payload+CRC but NEVER to the surrounding flags. Any
#  0x7E or 0x7D in that span is emitted as 0x7D followed by (byte XOR 0x20).
# ═════════════════════════════════════════════════════════════════════════════
def byte_stuff(data: bytes) -> bytes:
    """Escape 0x7E/0x7D within @p data per the GDL90 stuffing rule."""
    out = bytearray()
    for b in data:
        # Only the two reserved control bytes need escaping.
        if b in (GDL90_FLAG, GDL90_ESCAPE):
            out.append(GDL90_ESCAPE)
            out.append(b ^ GDL90_ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def byte_unstuff(data: bytes) -> bytes:
    """
    Reverse the GDL90 byte-stuffing on a flag-stripped frame body.

    @param data  Bytes between two 0x7E flags (still stuffed).
    @return      The original id+payload+CRC.
    @raises ValueError  On a dangling escape with no following byte.
    """
    out = bytearray()
    it = iter(range(len(data)))
    i = 0
    # Walk the buffer, expanding each 0x7D <b> pair back to <b XOR 0x20>.
    while i < len(data):
        b = data[i]
        if b == GDL90_ESCAPE:
            # An escape must be followed by exactly one stuffed byte.
            if i + 1 >= len(data):
                raise ValueError("dangling byte-stuff escape at end of frame")
            out.append(data[i + 1] ^ GDL90_ESCAPE_XOR)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


# ═════════════════════════════════════════════════════════════════════════════
#  Deframing — split a raw byte stream on 0x7E flags, un-stuff, verify CRC.
# ═════════════════════════════════════════════════════════════════════════════
@dataclass
class Gdl90Frame:
    """A single de-framed, CRC-checked GDL90 message."""
    msg_id: int             # First un-stuffed byte (e.g. 0x14 == Traffic).
    payload: bytes          # id-less payload (the bytes after msg_id, pre-CRC).
    crc_ok: bool            # Whether the trailing CRC-16 matched our recompute.
    raw_unstuffed: bytes    # Full un-stuffed id+payload+CRC, for diagnostics.


def deframe_stream(stream: bytes) -> List[Gdl90Frame]:
    """
    Extract every complete GDL90 frame from a raw USB-CDC byte stream.

    Follows WIRE_CONTRACT.md §3 "Deframe procedure": split on 0x7E, un-stuff,
    split off the trailing little-endian CRC-16, and verify it against a fresh
    CRC of the preceding id+payload. Malformed or short fragments are skipped
    rather than raised, because the stream is shared with UTF-8 debug text and
    we want robust resync.

    @param stream  Arbitrary bytes off the link (may contain partial frames).
    @return        List of frames in stream order (CRC failures included, flagged).
    """
    frames: List[Gdl90Frame] = []

    # Tokenise on the flag byte. Consecutive flags produce empty chunks we drop.
    for chunk in stream.split(bytes([GDL90_FLAG])):
        if not chunk:
            continue

        # A valid frame is at least id(1) + crc(2) = 3 bytes after un-stuffing.
        try:
            body = byte_unstuff(chunk)
        except ValueError:
            # Truncated escape — almost certainly a partial/garbage fragment.
            continue
        if len(body) < 3:
            continue

        # The link is SHARED with UTF-8 debug text, so a run of text (e.g.
        # "=== ADSBIN TRAFFIC ... ===\n") can sit between two real frames' flags
        # and masquerade as a frame chunk. Such a chunk starts with a text byte
        # ('=' 0x3D, '\n' 0x0A, ...), never a real GDL90 message id. Gate on the
        # id so text is skipped as resync noise rather than reported as a bogus
        # CRC-failing "frame"; this keeps the CRC tally meaningful for the frames
        # the device actually emitted. (A genuinely corrupted real frame keeps its
        # valid id and is still reported with crc_ok=False — the case we DO want.)
        msg_id = body[0]
        if msg_id not in _KNOWN_GDL90_IDS:
            continue

        # Last two bytes are the CRC (little-endian on the wire).
        crc_payload = body[:-2]
        crc_rx = body[-2] | (body[-1] << 8)
        crc_ok = crc16(crc_payload) == crc_rx

        frames.append(
            Gdl90Frame(
                msg_id=msg_id,
                payload=body[1:-2],
                crc_ok=crc_ok,
                raw_unstuffed=body,
            )
        )

    return frames


# Recognised GDL90 message ids — the gate that distinguishes a real frame chunk
# from interleaved debug text on the shared link. Mirrors WIRE_CONTRACT.md §3.
_KNOWN_GDL90_IDS = frozenset((
    GDL90_ID_HEARTBEAT,
    GDL90_ID_OWNSHIP,
    GDL90_ID_TRAFFIC,
))


# ═════════════════════════════════════════════════════════════════════════════
#  Field codecs shared by Traffic + Ownship (their payloads are identical).
#
#  Lat/lon: 24-bit two's-complement semicircles, value = round(deg * 2^23/180).
#  Altitude: 12-bit (alt_ft + 1000)/25, 0xFFF == invalid (WIRE_CONTRACT.md §3).
# ═════════════════════════════════════════════════════════════════════════════
_SEMICIRCLE = (1 << 23) / 180.0   # Degrees → 24-bit semicircle scale factor.


def semicircle_decode(raw24: int) -> float:
    """Decode a 24-bit two's-complement semicircle field back to degrees."""
    # Sign-extend the 24-bit field into Python's arbitrary-precision int.
    if raw24 & 0x800000:
        raw24 -= 1 << 24
    return raw24 / _SEMICIRCLE


def semicircle_encode(deg: float) -> int:
    """Encode degrees to a 24-bit two's-complement semicircle (for symmetry/tests)."""
    v = int(round(deg * _SEMICIRCLE))
    return v & 0xFFFFFF


def altitude_decode(raw12: int) -> Optional[int]:
    """Decode the 12-bit pressure-altitude field to feet, or None if invalid."""
    # 0xFFF is the spec's sentinel for "altitude unavailable".
    if raw12 == 0xFFF:
        return None
    return raw12 * 25 - 1000


# ═════════════════════════════════════════════════════════════════════════════
#  Parsed message dataclasses + parsers.
# ═════════════════════════════════════════════════════════════════════════════
@dataclass
class Heartbeat:
    """Decoded GDL90 Heartbeat (id 0x00) — gdl90_heartbeat_t mirror."""
    gps_pos_valid: bool
    maint_required: bool
    timestamp_s: int            # Seconds since UTC midnight (0..86399).
    msg_count_uplink: int
    msg_count_basic_long: int


def parse_heartbeat(payload: bytes) -> Heartbeat:
    """
    Parse a Heartbeat payload (the bytes AFTER the 0x00 id, before CRC).

    GDL90 Heartbeat layout after the id byte:
      [0] Status byte 1   (bit 7 = GPS pos valid)
      [1] Status byte 2   (bit 0 = timestamp MSB, bit 7 ... maint, etc.)
      [2..3] Timestamp    (little-endian low 16 bits; bit16 lives in status 2)
      [4..5] Message counts (uplink in top bits, basic/long in low 10 bits)
    """
    if len(payload) < 6:
        raise ValueError("heartbeat payload too short")

    st1 = payload[0]
    st2 = payload[1]

    # GPS position validity is status-byte-1 bit 7.
    gps_pos_valid = bool(st1 & 0x80)
    # Maintenance-required indicator is status-byte-1 bit 6 in the Garmin spec.
    maint_required = bool(st1 & 0x40)

    # Timestamp is 17 bits: low 16 in [2..3] LE, the 17th (MSB) is status-2 bit 7.
    ts = payload[2] | (payload[3] << 8)
    ts |= (st2 & 0x80) << 9   # st2 bit7 -> timestamp bit16.

    # Message counts: byte[4] top 5 bits = uplink, byte[4..5] low 10 = basic/long.
    msg_count_uplink = (payload[4] >> 3) & 0x1F
    msg_count_basic_long = ((payload[4] & 0x03) << 8) | payload[5]

    return Heartbeat(
        gps_pos_valid=gps_pos_valid,
        maint_required=maint_required,
        timestamp_s=ts,
        msg_count_uplink=msg_count_uplink,
        msg_count_basic_long=msg_count_basic_long,
    )


@dataclass
class TrafficReport:
    """
    Decoded GDL90 Traffic (0x14) / Ownship (0x0A) report.

    Shares the 27-byte payload layout per the spec; ``is_ownship`` records which
    message id carried it. This mirrors gdl90_traffic_t on the decode side.
    """
    is_ownship: bool
    alert_status: int
    addr_type: int
    icao: int
    lat_deg: float
    lon_deg: float
    alt_press_ft: Optional[int]
    airborne: bool
    nic: int
    nacp: int
    h_velocity_kt: int
    v_velocity_fpm: int
    track_heading_deg: float
    emitter_cat: int
    callsign: str
    emergency_code: int


def _parse_traffic_payload(payload: bytes, is_ownship: bool) -> TrafficReport:
    """Shared parser for the identical Traffic/Ownship 27-byte payload body."""
    # Payload (after id) is 27 bytes in the GDL90 spec; be strict so a layout
    # drift on the firmware side surfaces immediately rather than silently.
    if len(payload) < 27:
        raise ValueError(f"traffic payload too short: {len(payload)} bytes")

    # Byte 0: high nibble = alert status, low nibble = address type.
    alert_status = (payload[0] >> 4) & 0x0F
    addr_type = payload[0] & 0x0F

    # Bytes 1..3: 24-bit participant address (big-endian).
    icao = (payload[1] << 16) | (payload[2] << 8) | payload[3]

    # Bytes 4..6: 24-bit latitude semicircle; 7..9: longitude semicircle.
    lat_raw = (payload[4] << 16) | (payload[5] << 8) | payload[6]
    lon_raw = (payload[7] << 16) | (payload[8] << 8) | payload[9]
    lat_deg = semicircle_decode(lat_raw)
    lon_deg = semicircle_decode(lon_raw)

    # Bytes 10..11: 12-bit altitude (top of 12) + 4-bit misc nibble.
    alt_raw = (payload[10] << 4) | (payload[11] >> 4)
    alt_press_ft = altitude_decode(alt_raw)
    misc = payload[11] & 0x0F
    # Misc bit 3 (value 0x08) is the airborne flag in the GDL90 spec.
    airborne = bool(misc & 0x08)

    # Byte 12: high nibble = NIC, low nibble = NACp.
    nic = (payload[12] >> 4) & 0x0F
    nacp = payload[12] & 0x0F

    # Bytes 13..15: 12-bit horizontal velocity + 12-bit vertical velocity.
    h_velocity_kt = (payload[13] << 4) | (payload[14] >> 4)
    vv_raw = ((payload[14] & 0x0F) << 8) | payload[15]
    # Vertical velocity is a 12-bit signed value in units of 64 fpm.
    if vv_raw & 0x800:
        vv_raw -= 1 << 12
    v_velocity_fpm = vv_raw * 64

    # Byte 16: track/heading in units of 360/256 degrees.
    track_heading_deg = payload[16] * (360.0 / 256.0)

    # Byte 17: emitter category.
    emitter_cat = payload[17]

    # Bytes 18..25: 8-char callsign, space-padded; strip trailing pad/NULs.
    callsign = payload[18:26].decode("ascii", errors="replace").rstrip(" \x00")

    # Byte 26: high nibble = emergency/priority code.
    emergency_code = (payload[26] >> 4) & 0x0F

    return TrafficReport(
        is_ownship=is_ownship,
        alert_status=alert_status,
        addr_type=addr_type,
        icao=icao,
        lat_deg=lat_deg,
        lon_deg=lon_deg,
        alt_press_ft=alt_press_ft,
        airborne=airborne,
        nic=nic,
        nacp=nacp,
        h_velocity_kt=h_velocity_kt,
        v_velocity_fpm=v_velocity_fpm,
        track_heading_deg=track_heading_deg,
        emitter_cat=emitter_cat,
        callsign=callsign,
        emergency_code=emergency_code,
    )


def parse_traffic(payload: bytes) -> TrafficReport:
    """Parse a Traffic Report (id 0x14) payload."""
    return _parse_traffic_payload(payload, is_ownship=False)


def parse_ownship(payload: bytes) -> TrafficReport:
    """Parse an Ownship Report (id 0x0A) payload (same layout as Traffic)."""
    return _parse_traffic_payload(payload, is_ownship=True)


def parse_frame(frame: Gdl90Frame):
    """
    Dispatch a de-framed message to its typed parser.

    @return  A Heartbeat / TrafficReport, or None for an id we do not decode.
    """
    if frame.msg_id == GDL90_ID_HEARTBEAT:
        return parse_heartbeat(frame.payload)
    if frame.msg_id == GDL90_ID_TRAFFIC:
        return parse_traffic(frame.payload)
    if frame.msg_id == GDL90_ID_OWNSHIP:
        return parse_ownship(frame.payload)
    return None


# ═════════════════════════════════════════════════════════════════════════════
#  Mode-S / ADS-B frame parsing helpers (host reference for inject-verify).
#
#  These re-derive ICAO, type code, callsign, altitude and CPR fields straight
#  from the raw 56/112-bit frame so the harness can predict what the device's
#  decoder should produce for an injected frame.
# ═════════════════════════════════════════════════════════════════════════════
# Mode-S CRC uses the 24-bit polynomial 0xFFF409 (generator FFF409 in the spec).
_MODES_POLY = 0xFFF409


def modes_crc(msg: bytes) -> int:
    """
    Compute the 24-bit Mode-S parity over a frame (excluding its own PI field).

    @param msg  Full 7- or 14-byte frame; the last 3 bytes are the PI field.
    @return     24-bit remainder; equals the embedded PI for an un-corrupted,
                non-address-overlaid frame (DF17/18 with CRC, AA in PI == 0).
    """
    # Work over the message minus its trailing 24-bit parity, MSB-first.
    crc = 0
    nbits = (len(msg) - 3) * 8
    data = int.from_bytes(msg[:-3], "big")
    # Align the data into the top of a (nbits+24)-wide register.
    reg = data << 24
    # Long-division by the generator polynomial, bit by bit.
    topbit = 1 << (nbits + 24 - 1)
    poly = _MODES_POLY << (nbits - 1)
    for _ in range(nbits):
        if reg & topbit:
            reg ^= poly
        topbit >>= 1
        poly >>= 1
    crc = reg & 0xFFFFFF
    return crc


@dataclass
class ModesFrame:
    """A parsed Mode-S downlink frame (the fields the bench needs to verify)."""
    raw: bytes
    df: int                              # Downlink format (data[0] >> 3).
    icao: Optional[int] = None           # 24-bit address (DF17/18).
    type_code: Optional[int] = None      # ADS-B type code (ME[0] >> 3).
    callsign: Optional[str] = None       # Decoded flight id (TC 1..4).
    emitter_category: Optional[int] = None
    altitude_ft: Optional[int] = None    # Pressure altitude (airborne pos).
    cpr_odd: Optional[bool] = None       # CPR F bit (TC 9..18 / 20..22).
    cpr_lat: Optional[int] = None        # 17-bit encoded latitude.
    cpr_lon: Optional[int] = None        # 17-bit encoded longitude.
    crc_residual: int = 0                # parity(msg) XOR embedded PI.
    crc_ok: bool = False                 # True when residual == 0 (DF17/18).


# The 6-bit ICAO callsign character set (ADS-B identification, TC 1..4).
_CS_CHARSET = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"


def _decode_callsign(me: bytes) -> str:
    """Decode the 8-character flight id from a TC1-4 identification ME."""
    # The 48-bit character field starts after the 8-bit (TC+CA) header.
    bits = int.from_bytes(me[1:7], "big")
    chars = []
    # Eight 6-bit characters, MSB-first.
    for i in range(8):
        idx = (bits >> (42 - i * 6)) & 0x3F
        chars.append(_CS_CHARSET[idx])
    return "".join(chars).replace("#", "").rstrip("_ ").rstrip()


def _decode_ac12_altitude(me: bytes) -> Optional[int]:
    """
    Decode the 12-bit AC altitude field of an airborne-position ME (TC 9..18).

    Handles the common 25-ft Q-bit encoding; returns None when the field is zero
    (altitude unavailable) or uses the rare 100-ft Gillham path we don't model.
    """
    # The 12-bit AC field sits in ME bits 8..19 (byte5 low nibble + byte6).
    ac12 = ((me[5] & 0x0F) << 8) | me[6]
    if ac12 == 0:
        return None

    # Q-bit (bit 4 of the 12-bit field) selects the 25-ft encoding.
    q_bit = (ac12 >> 4) & 1
    if not q_bit:
        # Gillham-coded (100-ft) altitudes are out of scope for the bench corpus.
        return None

    # Drop the Q-bit: remaining 11 bits are N in feet = N*25 - 1000.
    n = ((ac12 & 0xFE0) >> 1) | (ac12 & 0x0F)
    return n * 25 - 1000


def parse_modes(raw: bytes) -> ModesFrame:
    """
    Parse a raw Mode-S frame into the fields the inject-verify path checks.

    Only DF17/DF18 (ADS-B / TIS-B) carry the ME we decode; other DFs return with
    just ``df`` populated. This is a reference decoder, not a full Mode-S stack.
    """
    df = raw[0] >> 3
    frame = ModesFrame(raw=raw, df=df)

    # Parity residual: for DF17/18 a clean frame gives residual 0.
    residual = modes_crc(raw) ^ int.from_bytes(raw[-3:], "big")
    frame.crc_residual = residual
    frame.crc_ok = (residual == 0)

    # Only ADS-B (17) and TIS-B (18) expose an ME block we decode here.
    if df in (17, 18) and len(raw) == 14:
        frame.icao = (raw[1] << 16) | (raw[2] << 8) | raw[3]
        me = raw[4:11]                      # 56-bit ME (7 bytes).
        tc = me[0] >> 3
        frame.type_code = tc

        if 1 <= tc <= 4:
            # Aircraft identification: callsign + emitter category.
            frame.callsign = _decode_callsign(me)
            frame.emitter_category = me[0] & 0x07
        elif 9 <= tc <= 18:
            # Airborne position (baro altitude): CPR lat/lon + AC altitude.
            frame.altitude_ft = _decode_ac12_altitude(me)
            frame.cpr_odd = bool((me[2] >> 2) & 1)
            frame.cpr_lat = ((me[2] & 0x03) << 15) | (me[3] << 7) | (me[4] >> 1)
            frame.cpr_lon = ((me[4] & 0x01) << 16) | (me[5] << 8) | me[6]

    return frame


# ═════════════════════════════════════════════════════════════════════════════
#  CPR decode — host mirror of cpr.h (clean-room from the public DO-260B math).
# ═════════════════════════════════════════════════════════════════════════════
_NZ = 15                       # Number of geographic latitude zones (constant).
_D_LAT_EVEN = 360.0 / 60.0     # Even-format latitude zone size (degrees).
_D_LAT_ODD = 360.0 / 59.0      # Odd-format latitude zone size (degrees).
_CPR_MAX = float(1 << 17)      # 2^17 — the CPR field's full scale.


def cpr_nl(lat: float) -> int:
    """
    Number of longitude zones (NL) at a given latitude — cpr_nl() mirror.

    Implements the transcendental NL(lat) from the CPR spec. Latitudes beyond
    the poles collapse to a single zone, as the spec requires.
    """
    # The poles are a degenerate single-zone case.
    if abs(lat) >= 87.0:
        return 1
    if lat == 0.0:
        return 59
    # NL(lat) = floor( 2*pi / arccos(1 - (1-cos(pi/(2*NZ))) / cos(lat)^2 ) ).
    nz_term = 1.0 - math.cos(math.pi / (2.0 * _NZ))
    denom = math.cos(math.radians(abs(lat))) ** 2
    inner = 1.0 - nz_term / denom
    # Guard against fp drift pushing the arccos argument out of [-1, 1].
    inner = max(-1.0, min(1.0, inner))
    return int(math.floor((2.0 * math.pi) / math.acos(inner)))


def cpr_global_decode(
    even_lat: int, even_lon: int, odd_lat: int, odd_lon: int, latest_is_odd: bool
) -> Optional[Tuple[float, float]]:
    """
    Global CPR decode from a matched even+odd pair — cpr_global_decode() mirror.

    @param even_lat/even_lon  17-bit CPR fields from the EVEN frame.
    @param odd_lat/odd_lon    17-bit CPR fields from the ODD frame.
    @param latest_is_odd      Which frame arrived most recently (anchors the zone).
    @return (lat, lon) in degrees, or None on an NL-zone mismatch reject.
    """
    # Normalise the encoded fields to fractional zone coordinates [0, 1).
    yz_even = even_lat / _CPR_MAX
    yz_odd = odd_lat / _CPR_MAX

    # Latitude zone index j is shared by both formats for a valid pair.
    j = math.floor(59.0 * yz_even - 60.0 * yz_odd + 0.5)

    # Recover each format's latitude, wrapped into the southern hemisphere band.
    rlat_even = _D_LAT_EVEN * ((j % 60) + yz_even)
    rlat_odd = _D_LAT_ODD * ((j % 59) + yz_odd)
    if rlat_even >= 270.0:
        rlat_even -= 360.0
    if rlat_odd >= 270.0:
        rlat_odd -= 360.0

    # Both latitudes must fall in the same NL zone or the pair is inconsistent.
    if cpr_nl(rlat_even) != cpr_nl(rlat_odd):
        return None

    # Pick the latitude/longitude that belongs to the most-recent frame.
    if latest_is_odd:
        rlat = rlat_odd
        nl = cpr_nl(rlat)
        ni = max(nl - 1, 1)
        d_lon = 360.0 / ni
        m = math.floor(
            (even_lon / _CPR_MAX) * (nl - 1) - (odd_lon / _CPR_MAX) * nl + 0.5
        )
        rlon = d_lon * ((m % ni) + odd_lon / _CPR_MAX)
    else:
        rlat = rlat_even
        nl = cpr_nl(rlat)
        ni = max(nl, 1)
        d_lon = 360.0 / ni
        m = math.floor(
            (even_lon / _CPR_MAX) * (nl - 1) - (odd_lon / _CPR_MAX) * nl + 0.5
        )
        rlon = d_lon * ((m % ni) + even_lon / _CPR_MAX)

    # Wrap longitude into the canonical [-180, 180) range.
    if rlon >= 180.0:
        rlon -= 360.0

    return (rlat, rlon)


def cpr_local_decode(
    odd: bool, lat_cpr: int, lon_cpr: int, ref_lat: float, ref_lon: float
) -> Optional[Tuple[float, float]]:
    """
    Local CPR decode against a reference position — cpr_local_decode() mirror.

    @param odd       CPR format bit of the single frame.
    @param lat_cpr   17-bit encoded latitude.
    @param lon_cpr   17-bit encoded longitude.
    @param ref_lat   Reference latitude (e.g. ownship), degrees.
    @param ref_lon   Reference longitude, degrees.
    @return (lat, lon) degrees, or None on reject.
    """
    d_lat = _D_LAT_ODD if odd else _D_LAT_EVEN
    yz = lat_cpr / _CPR_MAX

    # Latitude zone index nearest the reference latitude.
    j = math.floor(ref_lat / d_lat) + math.floor(
        0.5 + (((ref_lat % d_lat) / d_lat) - yz)
    )
    rlat = d_lat * (j + yz)

    # Longitude zone width depends on NL at the recovered latitude.
    nl = cpr_nl(rlat)
    ni = max(nl - 1, 1) if odd else max(nl, 1)
    d_lon = 360.0 / ni
    xz = lon_cpr / _CPR_MAX
    m = math.floor(ref_lon / d_lon) + math.floor(
        0.5 + (((ref_lon % d_lon) / d_lon) - xz)
    )
    rlon = d_lon * (m + xz)

    return (rlat, rlon)
