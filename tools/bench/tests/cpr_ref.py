# -*- coding: utf-8 -*-
"""
cpr_ref.py - Pure-Python clean-room reference for Compact Position Reporting.

This module is the HOST-SIDE twin of components/modes_decode/include/cpr.h. It
exists so the CPR math can be proven correct on a PC, completely independently of
the ESP32-P4 firmware: if the device-side cpr.c and this reference agree on the
same corpus, both are very likely right (and any divergence is a real bug rather
than a shared mistake).

Provenance / license:
    Clean-room implementation from the public CPR specification (ICAO Annex 10 /
    RTCA DO-260B), NOT copied from any GPL dump1090 fork. The structure mirrors
    the algebra in the spec and matches the FROZEN cpr.h API one-for-one:
        cpr_nl(lat)                              -> int  (1..59)
        cpr_global_decode(even, odd, latest_is_odd) -> (lat, lon) or None
        cpr_local_decode(frame, ref_lat, ref_lon)   -> (lat, lon) or None

    The firmware functions return 0 / negative; here we return a tuple on success
    and None on reject, which is the natural Python shape for unit tests. The
    NUMERIC results are required to match the firmware byte-for-byte.

A note on the one subtlety that bites every CPR implementation: the number of
geographic latitude zones NZ is fixed at 15. Latitude is quantised into 4*NZ even
bins or 4*NZ-1 odd bins; longitude into NL(lat) zones (even) or NL(lat)-1 (odd).
Everything below is just those two facts applied carefully with floored modulo.
"""

import math

# Number of geographic latitude zones. Fixed by the spec; do not change.
_NZ = 15

# Resolution of the 17-bit CPR fraction. 2**17 == 131072.
_CPR_MAX = 131072.0


def _floor_mod(a, b):
    """
    Floored modulo (a mod b) with the sign of the divisor, matching the
    mathematical 'mod' used throughout the CPR spec.

    Python's % already floors for positive divisors, but we spell it out so the
    intent is explicit and so the math reads the same as the C reference, which
    cannot lean on language-specific operator behaviour.
    """
    return a - b * math.floor(a / b)


def cpr_nl(lat):
    """
    CPR 'number of longitude zones' (NL) for a given latitude.

    This is the transcendental helper both global and local decode depend on. It
    returns how many longitude zones the Earth is divided into at that latitude:
    59 near the equator, shrinking to 1 at the poles.

    @param lat  Latitude in degrees.
    @return     NL in the range 1..59.
    """
    # Poleward of ~87 degrees there is exactly one longitude zone. Also guards the
    # acos() below from ever seeing an out-of-domain argument.
    if abs(lat) >= 87.0:
        return 1

    # At the equator NL is its maximum. Special-cased to dodge a needless acos.
    if lat == 0.0:
        return 59

    # The spec's closed form: NL is the floor of 2*pi / arccos(1 - (1-cos(pi/2NZ))
    # / cos^2(pi*lat/180)). We build it in pieces so the algebra is auditable.
    numerator = 1.0 - math.cos(math.pi / (2.0 * _NZ))
    denominator = math.cos(math.radians(abs(lat))) ** 2
    return int(math.floor((2.0 * math.pi) / math.acos(1.0 - numerator / denominator)))


def _nl_for_format(lat, odd):
    """
    The number of longitude zones used for ENCODING a frame of a given parity.

    Even frames use NL(lat) zones; odd frames use NL(lat)-1. Both are clamped to a
    minimum of 1 so the longitude bin width 360/ni never blows up near the poles.
    """
    ni = cpr_nl(lat) - 1 if odd else cpr_nl(lat)
    return ni if ni >= 1 else 1


def cpr_global_decode(even, odd, latest_is_odd):
    """
    Global CPR: recover an absolute lat/lon from a matched even+odd frame pair.

    No reference position is required - this is what lets the receiver report
    aircraft positions with no ownship fix at all. The caller guarantees the two
    frames are a fresh, genuine pair; latest_is_odd picks which frame's latitude
    zone anchors the answer (you decode against the MOST RECENT frame so the fix
    is current).

    @param even          dict-like with keys lat_cpr, lon_cpr (17-bit ints).
    @param odd           dict-like with keys lat_cpr, lon_cpr (17-bit ints).
    @param latest_is_odd True if the odd frame arrived last.
    @return              (lat_deg, lon_deg) on success, or None on NL mismatch
                         (which means the pair straddles a zone boundary and is
                         not safely resolvable).
    """
    # Latitude bin widths differ by parity: 4*NZ even bins, 4*NZ-1 odd bins.
    dlat_even = 360.0 / (4.0 * _NZ)
    dlat_odd = 360.0 / (4.0 * _NZ - 1.0)

    # Raw 17-bit latitude fractions, normalised into [0, 1).
    yz_even = even["lat_cpr"] / _CPR_MAX
    yz_odd = odd["lat_cpr"] / _CPR_MAX

    # The 'latitude index' j ties the two parities together into one global bin.
    j = math.floor((59.0 * yz_even - 60.0 * yz_odd) + 0.5)

    # Candidate latitudes from each frame, wrapped into the southern hemisphere
    # convention used by the spec (anything >= 270 belongs below the equator).
    rlat_even = dlat_even * (_floor_mod(j, 60.0) + yz_even)
    rlat_odd = dlat_odd * (_floor_mod(j, 59.0) + yz_odd)
    if rlat_even >= 270.0:
        rlat_even -= 360.0
    if rlat_odd >= 270.0:
        rlat_odd -= 360.0

    # If the two latitudes fall in different NL zones the pair is ambiguous; the
    # spec mandates rejecting it rather than emitting a wrong fix.
    if cpr_nl(rlat_even) != cpr_nl(rlat_odd):
        return None

    # Decode longitude against whichever frame is the freshest, using that frame's
    # latitude as the truth for NL.
    if latest_is_odd:
        lat = rlat_odd
        ni = _nl_for_format(rlat_odd, odd=True)
        # 'm' is the longitude zone index, again tying both parities together.
        m = math.floor(
            (even["lon_cpr"] * (cpr_nl(rlat_odd) - 1)
             - odd["lon_cpr"] * cpr_nl(rlat_odd)) / _CPR_MAX + 0.5)
        lon = (360.0 / ni) * (_floor_mod(m, ni) + odd["lon_cpr"] / _CPR_MAX)
    else:
        lat = rlat_even
        ni = _nl_for_format(rlat_even, odd=False)
        m = math.floor(
            (even["lon_cpr"] * (cpr_nl(rlat_even) - 1)
             - odd["lon_cpr"] * cpr_nl(rlat_even)) / _CPR_MAX + 0.5)
        lon = (360.0 / ni) * (_floor_mod(m, ni) + even["lon_cpr"] / _CPR_MAX)

    # Fold longitude into the canonical (-180, 180] presentation.
    if lon >= 180.0:
        lon -= 360.0

    return lat, lon


def cpr_local_decode(frame, ref_lat, ref_lon):
    """
    Local CPR: resolve a SINGLE frame relative to a known nearby reference.

    Faster than global (one message, no even/odd wait) but only valid when the
    reference (ownship, or the last good fix) is within ~180 NM of the target, so
    the zone disambiguation lands in the right bin.

    @param frame    dict-like with keys odd (bool), lat_cpr, lon_cpr.
    @param ref_lat  Reference latitude (degrees).
    @param ref_lon  Reference longitude (degrees).
    @return         (lat_deg, lon_deg) on success, or None on reject.
    """
    odd = bool(frame["odd"])

    # Latitude bin width for this frame's parity.
    dlat = 360.0 / (4.0 * _NZ - 1.0) if odd else 360.0 / (4.0 * _NZ)

    yz = frame["lat_cpr"] / _CPR_MAX

    # Pick the latitude zone nearest the reference. The two floor terms together
    # find the integer zone index whose centre is closest to ref_lat.
    j = (math.floor(ref_lat / dlat)
         + math.floor(0.5 + _floor_mod(ref_lat, dlat) / dlat - yz))
    rlat = dlat * (j + yz)

    # Longitude proceeds the same way, but the zone count depends on rlat.
    ni = _nl_for_format(rlat, odd)
    dlon = 360.0 / ni
    xz = frame["lon_cpr"] / _CPR_MAX
    m = (math.floor(ref_lon / dlon)
         + math.floor(0.5 + _floor_mod(ref_lon, dlon) / dlon - xz))
    rlon = dlon * (m + xz)

    return rlat, rlon
