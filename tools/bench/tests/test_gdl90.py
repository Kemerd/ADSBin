# -*- coding: utf-8 -*-
"""
test_gdl90.py - Host unit tests for the GDL90 transport (gdl90_ref.py).

Proves the framing layer the firmware's gdl90_encoder.h must produce: CRC-16
against the published spec example, byte-stuffing round-trips (including the
adversarial 0x7E/0x7D cases), full frame -> deframe round-trips, and the field
packings (semicircle lat/lon, 12-bit altitude). No firmware needed.
"""

import struct
import unittest

import gdl90_ref as g


class TestCrc16(unittest.TestCase):
    """CRC-16 must match the GDL90 spec's worked example."""

    def test_spec_heartbeat_example(self):
        # The spec lists this heartbeat id+payload with CRC 0xB38B on the wire.
        # On the wire the CRC is little-endian, so the integer value is 0x8BB3.
        msg = bytes([0x00, 0x81, 0x41, 0xDB, 0xD0, 0x08, 0x02])
        crc = g.gdl90_crc16(msg)
        self.assertEqual(crc, 0x8BB3)
        # And its LE wire bytes are 0xB3, 0x8B.
        self.assertEqual(struct.pack("<H", crc), bytes([0xB3, 0x8B]))

    def test_empty_payload_crc_is_zero(self):
        # CRC of nothing is the initial value, 0.
        self.assertEqual(g.gdl90_crc16(b""), 0x0000)

    def test_crc_is_order_sensitive(self):
        # A different byte order must (overwhelmingly) give a different CRC.
        self.assertNotEqual(g.gdl90_crc16(b"\x01\x02"), g.gdl90_crc16(b"\x02\x01"))


class TestStuffing(unittest.TestCase):
    """Byte-stuffing must escape exactly 0x7E and 0x7D and nothing else."""

    def test_no_special_bytes_passthrough(self):
        data = bytes([0x00, 0x14, 0x55, 0xAA])
        self.assertEqual(g.stuff(data), data)

    def test_flag_byte_is_escaped(self):
        # 0x7E -> 0x7D, (0x7E ^ 0x20) = 0x7D 0x5E.
        self.assertEqual(g.stuff(bytes([0x7E])), bytes([0x7D, 0x5E]))

    def test_escape_byte_is_escaped(self):
        # 0x7D -> 0x7D, (0x7D ^ 0x20) = 0x7D 0x5D.
        self.assertEqual(g.stuff(bytes([0x7D])), bytes([0x7D, 0x5D]))

    def test_stuff_unstuff_roundtrip_exhaustive(self):
        # Every single byte value must survive stuff -> unstuff unchanged.
        for b in range(256):
            data = bytes([b, b ^ 0x20, 0x7E, 0x7D, b])
            self.assertEqual(g.unstuff(g.stuff(data)), data)

    def test_dangling_escape_raises(self):
        # A lone trailing escape is a malformed (truncated) segment.
        with self.assertRaises(ValueError):
            g.unstuff(bytes([0x01, 0x7D]))


class TestFraming(unittest.TestCase):
    """Full frame/deframe behaviour and CRC verification."""

    def test_frame_brackets_with_flags(self):
        framed = g.frame(bytes([0x00, 0x01, 0x02]))
        self.assertEqual(framed[0], g.FLAG)
        self.assertEqual(framed[-1], g.FLAG)

    def test_frame_deframe_roundtrip(self):
        payload = bytes([0x14, 0x00, 0x12, 0x34, 0x56])
        stream = g.frame(payload)
        recovered = g.deframe(stream)
        self.assertEqual(recovered, [payload])

    def test_frame_with_special_bytes_in_payload(self):
        # A payload containing both special bytes must still round-trip; the
        # framing has to stuff them and the deframer un-stuff them.
        payload = bytes([0x0A, 0x7E, 0x7D, 0x7E, 0xFF, 0x00])
        stream = g.frame(payload)
        # The raw stream must contain no bare 0x7E except the two flags.
        self.assertEqual(stream.count(g.FLAG), 2)
        self.assertEqual(g.deframe(stream), [payload])

    def test_multiple_frames_in_one_stream(self):
        a = g.frame(bytes([0x00, 0xAA]))
        b = g.frame(bytes([0x14, 0xBB, 0xCC]))
        # Includes a shared flag boundary and leading garbage to mimic real links.
        self.assertEqual(g.deframe(a + b), [bytes([0x00, 0xAA]), bytes([0x14, 0xBB, 0xCC])])

    def test_bad_crc_is_dropped(self):
        payload = bytes([0x00, 0x11, 0x22])
        framed = bytearray(g.frame(payload))
        # Corrupt a payload byte (index 1, just after the leading flag).
        framed[1] ^= 0xFF
        self.assertEqual(g.deframe(bytes(framed)), [])

    def test_empty_segments_ignored(self):
        # Back-to-back flags (idle fill) must not produce phantom frames.
        self.assertEqual(g.deframe(bytes([g.FLAG, g.FLAG, g.FLAG])), [])


class TestFieldEncodings(unittest.TestCase):
    """Semicircle lat/lon and 12-bit altitude packings."""

    def test_latlon_semicircle_roundtrip(self):
        for deg in (0.0, 52.2572, -33.8688, 179.999, -179.999, 90.0, -90.0):
            packed = g.encode_latlon_semicircle(deg)
            back = g.decode_latlon_semicircle(packed)
            # 24-bit semicircles resolve to ~0.0000214 deg; allow one LSB.
            self.assertAlmostEqual(back, deg, delta=180.0 / (2 ** 23) + 1e-9)

    def test_latlon_zero_is_zero(self):
        self.assertEqual(g.encode_latlon_semicircle(0.0), bytes([0, 0, 0]))

    def test_altitude_roundtrip(self):
        # The 25 ft quantisation means we test on the grid.
        for ft in (-1000, -25, 0, 25, 1000, 38000, 101325):
            code = g.encode_altitude(ft)
            if code != g.ALT_INVALID:
                self.assertEqual(g.decode_altitude(code), ft)

    def test_altitude_invalid_sentinel(self):
        # None and out-of-range both map to the 0xFFF invalid code.
        self.assertEqual(g.encode_altitude(None), g.ALT_INVALID)
        self.assertEqual(g.encode_altitude(10_000_000), g.ALT_INVALID)
        self.assertIsNone(g.decode_altitude(g.ALT_INVALID))


class TestMessages(unittest.TestCase):
    """Whole-message build -> frame -> deframe round-trips."""

    def test_heartbeat_roundtrips_through_frame(self):
        msg = g.build_heartbeat(gps_pos_valid=True, maint_required=False,
                                timestamp_s=43200)
        frames = g.deframe(g.frame(msg))
        self.assertEqual(frames, [msg])
        # First byte is the heartbeat id.
        self.assertEqual(frames[0][0], g.MSG_HEARTBEAT)

    def test_traffic_report_roundtrips_and_decodes(self):
        msg = g.build_traffic_report(
            icao=0xA1B2C3, lat_deg=52.2572, lon_deg=3.91937, alt_ft=38000,
            misc_airborne=True, nic=8, nacp=9, h_velocity_kt=420,
            v_velocity_fpm=-1216, track_deg=183, emitter_cat=3,
            callsign="KLM1023")
        frames = g.deframe(g.frame(msg))
        self.assertEqual(frames, [msg])

        body = frames[0]
        self.assertEqual(body[0], g.MSG_TRAFFIC)
        # ICAO is bytes 2..4, big-endian.
        icao = (body[2] << 16) | (body[3] << 8) | body[4]
        self.assertEqual(icao, 0xA1B2C3)
        # Lat/lon are bytes 5..7 and 8..10.
        self.assertAlmostEqual(g.decode_latlon_semicircle(body[5:8]), 52.2572,
                               delta=180.0 / (2 ** 23) + 1e-6)
        self.assertAlmostEqual(g.decode_latlon_semicircle(body[8:11]), 3.91937,
                               delta=180.0 / (2 ** 23) + 1e-6)

    def test_ownship_uses_own_id(self):
        msg = g.build_traffic_report(
            icao=0x000001, lat_deg=0.0, lon_deg=0.0, alt_ft=0,
            misc_airborne=False, nic=0, nacp=0, h_velocity_kt=0,
            v_velocity_fpm=0, track_deg=0, emitter_cat=0, callsign="OWNSHIP",
            msg_id=g.MSG_OWNSHIP)
        self.assertEqual(g.deframe(g.frame(msg))[0][0], g.MSG_OWNSHIP)


if __name__ == "__main__":
    unittest.main()
