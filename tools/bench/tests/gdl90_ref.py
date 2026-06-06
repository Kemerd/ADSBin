# -*- coding: utf-8 -*-
"""
gdl90_ref.py - Pure-Python clean-room reference for GDL90 framing + CRC.

This is the host twin of components/sinks/include/gdl90_encoder.h and §3 of
tools/bench/WIRE_CONTRACT.md. It implements the wire-level transport (CRC-16,
byte-stuffing, 0x7E framing) and just enough message bit-packing (Heartbeat,
Traffic/Ownship Report) to round-trip and validate the device output without
firmware.

Provenance / license:
    Clean-room from the public Garmin "GDL 90 Data Interface Specification"
    (560-1058-00 Rev A) and the ForeFlight GDL90 Extended spec. No GPL code is
    used. The CRC-16 is the standard CCITT (poly 0x1021) table the GDL90 spec
    prints in its appendix; that table is a mathematical artefact of the
    polynomial, not copyrightable expression, and is regenerated here at import
    time rather than transcribed.

Wire contract this file MUST honour (see WIRE_CONTRACT.md / gdl90_encoder.h):
    - Frame = 0x7E | id+payload | CRC16(LE) | 0x7E.
    - CRC is computed over the UNSTUFFED id+payload only.
    - Byte-stuffing escapes 0x7E and 0x7D anywhere in id+payload+CRC (NEVER the
      flags): emit 0x7D then (byte XOR 0x20).
    - lat/lon are 24-bit two's-complement semicircles: round(deg * 2**23 / 180).
    - Pressure altitude is the 12-bit (alt_ft + 1000) / 25 encoding; 0xFFF means
      invalid/unavailable.
"""

import struct

# ---------------------------------------------------------------------------
# Framing constants. These mirror the table in WIRE_CONTRACT.md §3 exactly.
# ---------------------------------------------------------------------------
FLAG = 0x7E          # Start/end of every frame.
ESCAPE = 0x7D        # Byte-stuffing escape marker.
ESCAPE_XOR = 0x20    # Stuffed byte = original XOR 0x20.

# Message ids we model on the host side.
MSG_HEARTBEAT = 0x00
MSG_OWNSHIP = 0x0A
MSG_TRAFFIC = 0x14

# Sentinel the encoder writes when pressure altitude is unknown.
ALT_INVALID = 0xFFF


# ---------------------------------------------------------------------------
# CRC-16 (CCITT, poly 0x1021). Built once at import.
# ---------------------------------------------------------------------------
def _build_crc_table():
    """
    Generate the 256-entry CCITT CRC-16 table (the same one the GDL90 spec lists
    in its appendix). Regenerating it from the polynomial keeps this file free of
    any transcribed third-party data.
    """
    table = [0] * 256
    for i in range(256):
        crc = (i << 8) & 0xFFFF
        # Eight shift-and-conditional-xor steps per table slot.
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if (crc & 0x8000) else 0)) & 0xFFFF
        table[i] = crc
    return table


_CRC_TABLE = _build_crc_table()


def gdl90_crc16(payload):
    """
    GDL90 CRC-16 over an UNFRAMED, UNSTUFFED id+payload buffer.

    Matches gdl90_crc16() in gdl90_encoder.h. The returned integer is the host
    representation; on the wire it is appended little-endian (low byte first).

    @param payload  bytes/bytearray of id + message payload.
    @return         16-bit CRC as an int.
    """
    crc = 0
    for b in payload:
        crc = (_CRC_TABLE[(crc >> 8) & 0xFF] ^ ((crc << 8) & 0xFFFF) ^ b) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Byte-stuffing.
# ---------------------------------------------------------------------------
def stuff(data):
    """
    Apply GDL90 byte-stuffing to a buffer (id+payload+CRC, never the flags).

    Any 0x7E or 0x7D becomes 0x7D followed by (byte XOR 0x20). This is what makes
    the 0x7E flag unambiguous in the byte stream.
    """
    out = bytearray()
    for b in data:
        if b == FLAG or b == ESCAPE:
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def unstuff(data):
    """
    Reverse GDL90 byte-stuffing. Used by the deframer.

    @raises ValueError if an escape is the final byte (a truncated frame).
    """
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        b = data[i]
        if b == ESCAPE:
            # An escape must be followed by exactly one stuffed byte.
            if i + 1 >= n:
                raise ValueError("dangling escape byte at end of frame")
            out.append(data[i + 1] ^ ESCAPE_XOR)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)


# ---------------------------------------------------------------------------
# Frame / deframe.
# ---------------------------------------------------------------------------
def frame(id_payload):
    """
    Wrap a raw id+payload into a complete GDL90 frame.

    Appends the little-endian CRC, byte-stuffs id+payload+CRC, and brackets the
    result with 0x7E flags. The flags themselves are never stuffed.

    @param id_payload  bytes: message id byte followed by the payload.
    @return            bytes: the full on-wire frame.
    """
    crc = gdl90_crc16(id_payload)
    body = bytes(id_payload) + struct.pack("<H", crc)  # CRC little-endian.
    return bytes([FLAG]) + stuff(body) + bytes([FLAG])


def deframe(stream):
    """
    Split a raw GDL90 byte stream into validated id+payload buffers.

    Implements the host deframe procedure from WIRE_CONTRACT.md §3: split on
    0x7E, un-stuff, peel the trailing little-endian CRC, and verify it. Empty
    runs between back-to-back flags are ignored.

    @param stream  bytes from the device (may contain many frames + garbage).
    @return        list of bytes, each a verified id+payload (CRC stripped).
                   Frames with a bad CRC or that are too short are dropped.
    """
    frames = []
    # Walk flag-delimited segments. Two adjacent flags yield an empty segment,
    # which we simply skip.
    for segment in stream.split(bytes([FLAG])):
        if not segment:
            continue

        # Reverse stuffing; a malformed (truncated) segment is discarded.
        try:
            body = unstuff(segment)
        except ValueError:
            continue

        # Need at least one id byte plus the 2-byte CRC.
        if len(body) < 3:
            continue

        id_payload = body[:-2]
        got_crc = struct.unpack("<H", body[-2:])[0]
        if gdl90_crc16(id_payload) != got_crc:
            continue

        frames.append(id_payload)
    return frames


# ---------------------------------------------------------------------------
# Field encoders (just enough to build/validate the messages the bench uses).
# ---------------------------------------------------------------------------
def encode_latlon_semicircle(deg):
    """
    Pack a WGS-84 degree value as a 24-bit two's-complement semicircle.

    value = round(deg * 2**23 / 180), clamped to the signed 24-bit range, then
    returned as three big-endian bytes (GDL90 is big-endian for these fields).
    """
    raw = int(round(deg * (2 ** 23) / 180.0))
    # Clamp to the signed 24-bit window before taking the two's-complement view.
    raw = max(-(2 ** 23), min(2 ** 23 - 1, raw))
    if raw < 0:
        raw += (1 << 24)
    return bytes([(raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF])


def decode_latlon_semicircle(b3):
    """Inverse of encode_latlon_semicircle: three big-endian bytes -> degrees."""
    raw = (b3[0] << 16) | (b3[1] << 8) | b3[2]
    # Re-expand the sign bit of the 24-bit field.
    if raw & 0x800000:
        raw -= (1 << 24)
    return raw * 180.0 / (2 ** 23)


def encode_altitude(alt_ft):
    """
    Pack a pressure altitude (feet) into the 12-bit (alt + 1000)/25 code.

    Returns 0xFFF (invalid) for None or out-of-range altitudes, exactly as the
    encoder does for INT32_MIN.
    """
    if alt_ft is None:
        return ALT_INVALID
    code = (alt_ft + 1000) // 25
    if code < 0 or code > 0xFFE:
        return ALT_INVALID
    return code & 0xFFF


def decode_altitude(code):
    """Inverse of encode_altitude. Returns None for the 0xFFF invalid sentinel."""
    if code == ALT_INVALID:
        return None
    return code * 25 - 1000


# ---------------------------------------------------------------------------
# Message builders.
# ---------------------------------------------------------------------------
def build_heartbeat(gps_pos_valid, maint_required, timestamp_s,
                    msg_count_uplink=0, msg_count_basic_long=0):
    """
    Build a GDL90 Heartbeat (id 0x00) id+payload (pre-frame).

    Layout (7 bytes total): id, status byte 1, status byte 2, timestamp (LE
    16-bit low part with the 17th bit folded into status 2), then the message
    counts. We model the common subset the bench checks: the GPS-valid bit, the
    maintenance bit, and the 17-bit seconds-since-midnight timestamp.
    """
    st1 = 0x00
    if gps_pos_valid:
        st1 |= 0x80          # Status byte 1, bit 7 = GPS position valid.

    # The timestamp is 17 bits: low 16 bits live in two LE bytes, the top bit is
    # carried in status byte 2 bit 7 (per the spec's heartbeat layout).
    ts = timestamp_s & 0x1FFFF
    st2 = 0x00
    if maint_required:
        st2 |= 0x01          # Maintenance-required indicator.
    if ts & 0x10000:
        st2 |= 0x80          # Bit 16 of the timestamp.

    payload = bytearray()
    payload.append(MSG_HEARTBEAT)
    payload.append(st1)
    payload.append(st2)
    payload.append(ts & 0xFF)            # Timestamp low byte (little-endian).
    payload.append((ts >> 8) & 0xFF)     # Timestamp high byte.
    payload.append((msg_count_uplink >> 0) & 0xFF)
    payload.append(msg_count_basic_long & 0xFF)
    return bytes(payload)


def build_traffic_report(icao, lat_deg, lon_deg, alt_ft, misc_airborne,
                         nic, nacp, h_velocity_kt, v_velocity_fpm,
                         track_deg, emitter_cat, callsign,
                         alert_status=0, addr_type=0, emergency_code=0,
                         msg_id=MSG_TRAFFIC):
    """
    Build a GDL90 Traffic Report (id 0x14) or Ownship Report (id 0x0A).

    Both reports share the 28-byte payload layout from the GDL90 spec. This packs
    the subset of fields the bench validates; the bit-packing here is the same one
    gdl90_frame_traffic_report() must produce on the device.
    """
    p = bytearray()
    p.append(msg_id)

    # Byte 1: high nibble = alert status, low nibble = address type.
    p.append(((alert_status & 0x0F) << 4) | (addr_type & 0x0F))

    # Bytes 2-4: 24-bit participant address (big-endian).
    p.append((icao >> 16) & 0xFF)
    p.append((icao >> 8) & 0xFF)
    p.append(icao & 0xFF)

    # Bytes 5-7 latitude, 8-10 longitude (24-bit semicircles).
    p += encode_latlon_semicircle(lat_deg)
    p += encode_latlon_semicircle(lon_deg)

    # Bytes 11-12: 12-bit altitude + 4-bit misc field (airborne flag etc).
    alt = encode_altitude(alt_ft)
    misc = 0x9 if misc_airborne else 0x8   # bit3=track-is-true-track, bit0=airborne
    p.append((alt >> 4) & 0xFF)
    p.append(((alt & 0x0F) << 4) | (misc & 0x0F))

    # Byte 13: high nibble NIC, low nibble NACp.
    p.append(((nic & 0x0F) << 4) | (nacp & 0x0F))

    # Bytes 14-16: 12-bit horizontal velocity + 12-bit vertical velocity.
    hv = h_velocity_kt & 0xFFF
    # Vertical velocity is signed, in 64 fpm units, 12-bit two's complement.
    vv = int(round(v_velocity_fpm / 64.0))
    vv = max(-2048, min(2047, vv)) & 0xFFF
    p.append((hv >> 4) & 0xFF)
    p.append(((hv & 0x0F) << 4) | ((vv >> 8) & 0x0F))
    p.append(vv & 0xFF)

    # Byte 17: track/heading scaled to 360/256 degrees.
    p.append(int(round(track_deg * 256.0 / 360.0)) & 0xFF)

    # Byte 18: emitter category.
    p.append(emitter_cat & 0xFF)

    # Bytes 19-26: 8-char callsign, space-padded.
    cs = (callsign or "").encode("ascii", "replace")[:8].ljust(8, b" ")
    p += cs

    # Byte 27: high nibble emergency/priority code, low nibble spare.
    p.append((emergency_code & 0x0F) << 4)
    return bytes(p)
