# -*- coding: utf-8 -*-
"""
spoof_traffic.py - Synthesize live ADS-B traffic around a point and stream it
into the device via +INJECT, so it surfaces over the real decode -> GDL90 ->
WiFi path and shows up on ForeFlight as moving aircraft.

This is the ENCODE side - the mirror of the bench's decoders (cpr_ref.py,
gdl90.py). Nothing here is canned: it builds genuine DF17 extended-squitter
frames (airborne position even/odd CPR pair, ground velocity, and flight-id)
from a desired aircraft state, with a correct 24-bit Mode-S parity, so the
firmware decodes them on the exact same path a real off-air frame would take.

Provenance / license:
    Clean-room from the public CPR spec (ICAO Annex 10 / RTCA DO-260B) and the
    Mode-S frame layout. The CPR *encode* algebra here is the inverse of the
    decode in tools/bench/tests/cpr_ref.py; the 0xFFF409 parity reuses
    gdl90.modes_crc(). No GPL dump1090 fork was consulted.

Coordinate model: each spoofed aircraft is a small kinematic object (lat, lon,
altitude, ground speed, track). step() advances it by dt seconds along its
track; frames() renders its current state to the wire. The caller loops:
advance -> render -> +INJECT -> sleep, which makes the targets crawl across the
ForeFlight map in real time.

Pure computation + a thin dependency on gdl90.modes_crc. Windows / UTF-8 safe.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

import gdl90  # reuse the verified Mode-S CRC (poly 0xFFF409)


# ─────────────────────────────────────────────────────────────────────────────
#  CPR encode - the inverse of cpr_ref.cpr_global_decode.
#
#  Latitude is quantised into 4*NZ even bins (or 4*NZ-1 odd) and longitude into
#  NL(lat) zones (even) or NL(lat)-1 (odd). Encoding is just "take the fractional
#  position within this aircraft's bin and scale it to 17 bits". We keep NZ/NL in
#  lockstep with cpr_ref so a frame we encode here decodes back to where we put it.
# ─────────────────────────────────────────────────────────────────────────────
_NZ = 15
_CPR_MAX = 131072.0   # 2**17


def _cpr_nl(lat: float) -> int:
    """NL(lat): number of longitude zones at this latitude (1..59). Mirrors cpr_ref."""
    # One zone at/above ~87 deg; guards the acos() domain.
    if abs(lat) >= 87.0:
        return 1
    # Maximum at the equator; special-cased to avoid a needless acos.
    if lat == 0.0:
        return 59
    # The spec's closed form, built in pieces so it reads like cpr_ref.cpr_nl.
    numerator = 1.0 - math.cos(math.pi / (2.0 * _NZ))
    denominator = math.cos(math.radians(abs(lat))) ** 2
    return int(math.floor((2.0 * math.pi) / math.acos(1.0 - numerator / denominator)))


def cpr_encode(lat: float, lon: float, odd: bool) -> Tuple[int, int]:
    """
    Encode an absolute lat/lon into a 17-bit CPR (lat, lon) pair for one parity.

    @param lat  Latitude in degrees  (-90..90).
    @param lon  Longitude in degrees (-180..180).
    @param odd  False => even frame, True => odd frame.
    @return     (lat_cpr, lon_cpr), each a 17-bit integer (0..131071).
    """
    i = 1 if odd else 0

    # Latitude bin width depends on parity (4*NZ even bins, 4*NZ-1 odd bins).
    dlat = 360.0 / (4.0 * _NZ - i)

    # Fractional position of `lat` within its latitude bin, scaled to 17 bits.
    # floor(... + 0.5) rounds to the nearest representable CPR step.
    yz = math.floor(_CPR_MAX * ((lat % dlat) / dlat) + 0.5)
    lat_cpr = int(yz) % int(_CPR_MAX)

    # Longitude bin count uses NL at THIS latitude, minus one for odd frames,
    # clamped to >=1 so the bin width 360/nl never blows up near the poles.
    nl = _cpr_nl(lat) - i
    if nl < 1:
        nl = 1
    dlon = 360.0 / nl

    # Same fractional-position-in-bin scaling for longitude.
    xz = math.floor(_CPR_MAX * ((lon % dlon) / dlon) + 0.5)
    lon_cpr = int(xz) % int(_CPR_MAX)

    return lat_cpr, lon_cpr


# ─────────────────────────────────────────────────────────────────────────────
#  Altitude encode - the 12-bit AC field with the 25-ft Q-bit encoding.
#  (The inverse of gdl90._decode_ac12_altitude's common path.)
# ─────────────────────────────────────────────────────────────────────────────
def _encode_ac12(alt_ft: int) -> int:
    """
    Encode a pressure altitude (feet) into the 12-bit AC field, Q-bit = 1.

    The 25-ft encoding is: N = (alt + 1000) / 25, then the Q bit (bit 4 of the
    12-bit field) is set and N's bits are split around it. Returns the 12-bit
    value ready to drop into ME bytes 5..6.
    """
    # 25-ft steps, biased by 1000 ft so -1000 ft maps to zero.
    n = (int(alt_ft) + 1000) // 25
    if n < 0:
        n = 0
    if n > 0x7FF:
        n = 0x7FF
    # Q bit (value 0x10 within the 12-bit field) marks the 25-ft encoding. The
    # lower 4 bits of N sit below Q; the upper 7 bits sit above it.
    return ((n & 0x7F0) << 1) | 0x10 | (n & 0x0F)


# ─────────────────────────────────────────────────────────────────────────────
#  Frame assembly helpers.
# ─────────────────────────────────────────────────────────────────────────────
def _finish_frame(first11: bytes) -> str:
    """
    Append the 24-bit Mode-S parity to the first 11 bytes and return 28-hex.

    For DF17 with no address overlay the parity IS the CRC over the 11-byte
    message, so a receiver computing a zero residual accepts it. We reuse the
    bench's verified modes_crc().
    """
    assert len(first11) == 11
    # modes_crc divides the message-minus-parity; feed an 11-byte body + 3 zero
    # parity bytes so the divisor sees the right width, then take the remainder.
    parity = gdl90.modes_crc(first11 + b"\x00\x00\x00")
    frame = first11 + parity.to_bytes(3, "big")
    return frame.hex().upper()


def _df17_header(icao: int) -> bytes:
    """The common 4-byte DF17 header: DF=17, CA=5, then the 24-bit ICAO."""
    # DF17 (10001) in the top 5 bits, CA=5 (airborne, capable) in the low 3.
    b0 = (17 << 3) | 0x05
    return bytes([b0]) + icao.to_bytes(3, "big")


# ─────────────────────────────────────────────────────────────────────────────
#  Aircraft model + frame rendering.
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class SpoofAircraft:
    """One synthetic aircraft: identity + kinematic state that step() advances."""
    icao: int                     # 24-bit address (must be unique per target).
    callsign: str                 # up to 8 chars from the ADS-B charset.
    lat: float                    # current latitude  (deg)
    lon: float                    # current longitude (deg)
    alt_ft: int                   # pressure altitude (ft)
    gs_kt: float                  # ground speed (kt)
    track_deg: float              # true track over ground (deg, 0=N, 90=E)
    category: int = 1             # emitter category (1 = light aircraft)
    _toggle: bool = field(default=False, repr=False)  # even/odd CPR alternator

    def step(self, dt_s: float) -> None:
        """Advance the aircraft along its track by @p dt_s seconds."""
        # Ground speed kt -> deg/sec. 1 kt = 1 NM/h; 1 NM latitude = 1/60 deg.
        nm = self.gs_kt * (dt_s / 3600.0)
        dlat = nm / 60.0
        # Longitude degrees shrink with cos(lat); guard the pole singularity.
        coslat = max(0.05, math.cos(math.radians(self.lat)))
        dlon = nm / (60.0 * coslat)
        # Project the NM moved onto the track heading.
        th = math.radians(self.track_deg)
        self.lat += dlat * math.cos(th)
        self.lon += dlon * math.sin(th)

    def _callsign_field(self) -> int:
        """Pack the 8-char callsign into the 48-bit identification field."""
        # Inverse of gdl90._decode_callsign's charset lookup.
        charset = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######"
        cs = (self.callsign.upper() + "        ")[:8]
        bits = 0
        for ch in cs:
            idx = charset.find(ch)
            if idx < 0:
                idx = 32  # space -> '#'/blank slot
            bits = (bits << 6) | (idx & 0x3F)
        return bits

    def frame_ident(self) -> str:
        """Render a TC4 identification frame (callsign + emitter category)."""
        me = bytearray(7)
        # TC=4 (aircraft id), then the 3-bit category code in CA position.
        me[0] = (4 << 3) | (self.category & 0x07)
        # 48-bit callsign field across ME bytes 1..6.
        field48 = self._callsign_field()
        me[1:7] = field48.to_bytes(6, "big")
        return _finish_frame(_df17_header(self.icao) + bytes(me))

    def frame_position(self) -> str:
        """
        Render one airborne-position frame, alternating even/odd each call.

        ForeFlight needs a fresh even+odd PAIR to globally resolve position, so a
        caller emits frames repeatedly; we alternate parity so a pair completes
        within two calls.
        """
        odd = self._toggle
        self._toggle = not self._toggle

        lat_cpr, lon_cpr = cpr_encode(self.lat, self.lon, odd)
        ac12 = _encode_ac12(self.alt_ft)

        me = bytearray(7)
        # TC=11 (airborne position, barometric alt). Surveillance status 0, NICb 0.
        me[0] = (11 << 3)
        # 12-bit altitude occupies ME bits 8..19 (byte1 + byte2 high nibble).
        me[1] = (ac12 >> 4) & 0xFF
        me[2] = ((ac12 & 0x0F) << 4)
        # Time bit T=0; the F (parity) bit selects even/odd. They live in byte2 low.
        me[2] |= (0x01 if odd else 0x00)  # F bit
        # 17-bit latitude (byte2 low bit already used by F => lat starts byte3).
        # Pack lat_cpr into bits, then lon_cpr, matching the standard ME layout:
        #   byte2: [alt_lo(4)][T=0][F][lat16][lat15]
        # To keep this auditable we assemble the 56-bit ME as one integer.
        me_int = (
            (11 << 51)                       # TC
            | (0 << 48)                      # SS(2)+NICb(1) = 0
            | (ac12 << 36)                   # 12-bit altitude
            | (0 << 35)                      # T (time) = 0
            | ((1 if odd else 0) << 34)      # F (CPR parity)
            | (lat_cpr << 17)                # 17-bit CPR latitude
            | (lon_cpr << 0)                 # 17-bit CPR longitude
        )
        me_bytes = me_int.to_bytes(7, "big")
        return _finish_frame(_df17_header(self.icao) + me_bytes)

    def frame_velocity(self) -> str:
        """Render a TC19 subtype-1 ground-velocity frame (speed + track)."""
        # East/West and North/South velocity components in kt (subtype 1, subsonic).
        th = math.radians(self.track_deg)
        ve = self.gs_kt * math.sin(th)   # east component (+E)
        vn = self.gs_kt * math.cos(th)   # north component (+N)

        # Direction bits: 1 = west / south. Magnitude is |v|+1 (0 means "no data").
        ew_dir = 1 if ve < 0 else 0
        ns_dir = 1 if vn < 0 else 0
        ew_v = min(1023, int(round(abs(ve))) + 1)
        ns_v = min(1023, int(round(abs(vn))) + 1)

        me_int = (
            (19 << 51)        # TC=19 (velocity)
            | (1 << 48)       # subtype 1 (ground speed, subsonic)
            | (0 << 47)       # intent change flag
            | (0 << 46)       # IFR capability
            | (0 << 43)       # navigation uncertainty (NUCr)
            | (ew_dir << 42)
            | (ew_v << 32)
            | (ns_dir << 31)
            | (ns_v << 21)
            | (0 << 20)       # vertical-rate source
            | (0 << 19)       # vertical-rate sign
            | (0 << 10)       # vertical rate magnitude (level)
            | (0 << 0)        # reserved / GNSS-baro diff
        )
        me_bytes = me_int.to_bytes(7, "big")
        return _finish_frame(_df17_header(self.icao) + me_bytes)


# ─────────────────────────────────────────────────────────────────────────────
#  Scenario builder: a few low-and-slow GA targets around a centre point.
# ─────────────────────────────────────────────────────────────────────────────
def low_slow_scenario(center_lat: float, center_lon: float) -> List[SpoofAircraft]:
    """
    Build a few low, slow aircraft scattered a couple miles around the centre.

    Returns GA-style targets (1500-3000 ft, 100-150 kt) on varied tracks so they
    crawl across the ForeFlight map in different directions - the "something is
    flying right next to me" demo.
    """
    # Offsets in (north_nm, east_nm) so the targets straddle the centre point.
    # ~1 NM lat = 1/60 deg; longitude scaled by cos(lat) at render time.
    def offset(nm_n: float, nm_e: float) -> Tuple[float, float]:
        dlat = nm_n / 60.0
        dlon = nm_e / (60.0 * max(0.05, math.cos(math.radians(center_lat))))
        return center_lat + dlat, center_lon + dlon

    targets: List[SpoofAircraft] = []

    # 1) Northeast, tracking southwest toward the centre, 2200 ft, 120 kt.
    la, lo = offset(2.0, 2.0)
    targets.append(SpoofAircraft(icao=0xAC0001, callsign="N100AB",
                                 lat=la, lon=lo, alt_ft=2200,
                                 gs_kt=120, track_deg=225))

    # 2) West, tracking east across the nose, 1500 ft, 105 kt.
    la, lo = offset(0.3, -2.5)
    targets.append(SpoofAircraft(icao=0xAC0002, callsign="N200CD",
                                 lat=la, lon=lo, alt_ft=1500,
                                 gs_kt=105, track_deg=90))

    # 3) South, tracking north, climbing-ish slow GA, 3000 ft, 140 kt.
    la, lo = offset(-2.2, 0.5)
    targets.append(SpoofAircraft(icao=0xAC0003, callsign="N300EF",
                                 lat=la, lon=lo, alt_ft=3000,
                                 gs_kt=140, track_deg=10))

    return targets
