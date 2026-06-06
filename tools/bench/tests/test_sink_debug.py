# -*- coding: utf-8 -*-
"""
test_sink_debug.py - Host unit tests for the sink_debug token format.

Proves the render <-> parse round-trip of the traffic-table line frozen in
WIRE_CONTRACT.md §1: required tokens always present, optional tokens omitted
when absent, ICAO/LAT/LON formatting rules, and that the live-harness parser
keys correctly on "ICAO=" lines while ignoring headers/footers.
"""

import unittest

import sink_debug_ref as sd


class TestRender(unittest.TestCase):
    """render_target() must honour the token contract."""

    def test_minimal_line_required_tokens_only(self):
        # With no optional fields, only ICAO, MSGS, SEEN appear, in that order.
        line = sd.render_target({"icao": 0xA1B2C3, "msgs": 5, "seen_ms": 1200})
        self.assertEqual(line, "ICAO=A1B2C3 MSGS=5 SEEN=1200")

    def test_icao_is_six_upper_hex_zero_padded(self):
        # A small ICAO must zero-pad to six uppercase hex digits.
        line = sd.render_target({"icao": 0x00CD, "msgs": 1, "seen_ms": 0})
        self.assertTrue(line.startswith("ICAO=0000CD "))

    def test_lat_lon_have_four_fraction_digits(self):
        line = sd.render_target({
            "icao": 0x1, "msgs": 1, "seen_ms": 0,
            "lat": 52.2572021, "lon": -3.5})
        self.assertIn("LAT=52.2572", line)
        # Negative longitude keeps its sign and the >=4 fractional-digit rule.
        self.assertIn("LON=-3.5000", line)

    def test_optional_tokens_omitted_when_absent(self):
        # None-valued optionals must not appear at all (not "LAT=" with no value).
        line = sd.render_target({
            "icao": 0x1, "msgs": 1, "seen_ms": 0,
            "lat": None, "callsign": None, "alt_ft": None})
        self.assertNotIn("LAT=", line)
        self.assertNotIn("CS=", line)
        self.assertNotIn("ALT=", line)

    def test_token_order_matches_contract(self):
        # A fully-populated target must emit tokens in the canonical order.
        line = sd.render_target({
            "icao": 0xABCDEF, "callsign": "DLH456", "lat": 1.0, "lon": 2.0,
            "alt_ft": 35000, "gs_kt": 450, "trk_deg": 270, "vr_fpm": -640,
            "cat": 3, "nic": 8, "nacp": 9, "rng_nm": 12.3, "brg_deg": 95,
            "msgs": 42, "seen_ms": 300})
        keys = [tok.split("=", 1)[0] for tok in line.split()]
        self.assertEqual(keys, sd.TOKEN_ORDER)


class TestParse(unittest.TestCase):
    """parse_target() must key on ICAO= lines and coerce types."""

    def test_non_target_lines_return_none(self):
        # Header/footer and blanks are not target rows.
        self.assertIsNone(sd.parse_target("=== ADSBIN TRAFFIC 3 @ 12345 ==="))
        self.assertIsNone(sd.parse_target("=== END ==="))
        self.assertIsNone(sd.parse_target(""))
        self.assertIsNone(sd.parse_target("   "))

    def test_parse_minimal(self):
        d = sd.parse_target("ICAO=A1B2C3 MSGS=5 SEEN=1200")
        self.assertEqual(d["ICAO"], 0xA1B2C3)
        self.assertEqual(d["MSGS"], 5)
        self.assertEqual(d["SEEN"], 1200)

    def test_parse_types(self):
        d = sd.parse_target(
            "ICAO=ABCDEF CS=DLH456 LAT=1.2345 LON=-2.0 ALT=35000 "
            "GS=450 TRK=270 VR=-640 CAT=3 NIC=8 NACp=9 RNG=12.3 BRG=95 "
            "MSGS=42 SEEN=300")
        self.assertIsInstance(d["ICAO"], int)
        self.assertEqual(d["CS"], "DLH456")
        self.assertIsInstance(d["LAT"], float)
        self.assertAlmostEqual(d["LAT"], 1.2345)
        self.assertEqual(d["ALT"], 35000)
        self.assertEqual(d["VR"], -640)        # signed
        self.assertEqual(d["NACp"], 9)
        self.assertAlmostEqual(d["RNG"], 12.3)


class TestRoundTrip(unittest.TestCase):
    """render -> parse must preserve every field value."""

    def test_full_roundtrip(self):
        target = {
            "icao": 0x4840D6, "callsign": "KLM1023", "lat": 52.2572,
            "lon": 3.9194, "alt_ft": 38000, "gs_kt": 487, "trk_deg": 183,
            "vr_fpm": -1216, "cat": 3, "nic": 8, "nacp": 9, "rng_nm": 7.4,
            "brg_deg": 312, "msgs": 128, "seen_ms": 95}

        parsed = sd.parse_target(sd.render_target(target))

        self.assertEqual(parsed["ICAO"], target["icao"])
        self.assertEqual(parsed["CS"], target["callsign"])
        self.assertAlmostEqual(parsed["LAT"], target["lat"], places=4)
        self.assertAlmostEqual(parsed["LON"], target["lon"], places=4)
        self.assertEqual(parsed["ALT"], target["alt_ft"])
        self.assertEqual(parsed["GS"], target["gs_kt"])
        self.assertEqual(parsed["TRK"], target["trk_deg"])
        self.assertEqual(parsed["VR"], target["vr_fpm"])
        self.assertEqual(parsed["CAT"], target["cat"])
        self.assertEqual(parsed["NIC"], target["nic"])
        self.assertEqual(parsed["NACp"], target["nacp"])
        self.assertAlmostEqual(parsed["RNG"], target["rng_nm"], places=1)
        self.assertEqual(parsed["BRG"], target["brg_deg"])
        self.assertEqual(parsed["MSGS"], target["msgs"])
        self.assertEqual(parsed["SEEN"], target["seen_ms"])

    def test_block_scan_picks_only_targets(self):
        # A whole publish block: scanning it must yield exactly the target rows.
        block = (
            "=== ADSBIN TRAFFIC 2 @ 999 ===\n"
            "ICAO=AAAAAA MSGS=1 SEEN=10\n"
            "ICAO=BBBBBB CS=TEST LAT=1.0000 LON=2.0000 MSGS=2 SEEN=20\n"
            "=== END ===\n")
        targets = [sd.parse_target(ln) for ln in block.splitlines()]
        targets = [t for t in targets if t is not None]
        self.assertEqual(len(targets), 2)
        self.assertEqual(targets[0]["ICAO"], 0xAAAAAA)
        self.assertEqual(targets[1]["CS"], "TEST")


if __name__ == "__main__":
    unittest.main()
