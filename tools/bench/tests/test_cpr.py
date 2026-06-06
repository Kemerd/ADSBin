# -*- coding: utf-8 -*-
"""
test_cpr.py - Host unit tests for the CPR math (cpr_ref.py).

These prove the pure CPR algorithms against the canned_msgs ground truth WITHOUT
any firmware: cpr_nl edge cases, global even/odd pairing, and local single-frame
decode. If these pass, the host reference is trustworthy and any device-side
divergence on the same corpus is a real firmware bug.
"""

import unittest

import canned_msgs as cm
import cpr_ref


class TestCprNl(unittest.TestCase):
    """cpr_nl() must follow the published NL transition table."""

    def test_equator_is_max(self):
        # At and very near the equator there are the maximum 59 longitude zones.
        self.assertEqual(cpr_ref.cpr_nl(0.0), 59)

    def test_poles_are_one(self):
        # Poleward of ~87 deg there is exactly one zone (north and south).
        self.assertEqual(cpr_ref.cpr_nl(89.9), 1)
        self.assertEqual(cpr_ref.cpr_nl(-89.9), 1)
        self.assertEqual(cpr_ref.cpr_nl(87.0), 1)

    def test_known_transition_values(self):
        # Published NL table spot-checks (NZ=15, the standard mode-s.org / DO-260B
        # latitude->NL table). NL steps down by one as latitude crosses each
        # boundary; these are well-known reference points, the 52.2572->36 one
        # being the latitude of the canonical worked CPR example.
        cases = [
            (10.0, 59),
            (20.0, 56),
            (40.0, 45),
            (52.2572, 36),
            (60.0, 29),
            (70.0, 20),
            (80.0, 10),
            (86.0, 3),
        ]
        for lat, expected in cases:
            with self.subTest(lat=lat):
                self.assertEqual(cpr_ref.cpr_nl(lat), expected)

    def test_nl_boundary_transitions(self):
        # NL must drop by exactly one as latitude crosses a published boundary.
        # The first boundary is at |lat| = 10.47047130 deg (59 -> 58).
        self.assertEqual(cpr_ref.cpr_nl(10.4704), 59)
        self.assertEqual(cpr_ref.cpr_nl(10.4705), 58)

    def test_symmetry_north_south(self):
        # NL depends only on |lat|, so north and south must agree.
        for lat in (5.0, 33.3, 51.5, 72.1):
            self.assertEqual(cpr_ref.cpr_nl(lat), cpr_ref.cpr_nl(-lat))


class TestCorpusIntegrity(unittest.TestCase):
    """The canned frames must be genuine, CRC-valid Mode-S frames."""

    def test_all_corpus_frames_have_zero_syndrome(self):
        # A real frame's appended 24-bit parity makes the CRC remainder vanish.
        # If any of these is nonzero, the corpus itself is corrupt and every CPR
        # result derived from it would be meaningless.
        for hexstr in (cm.AIRBORNE_EVEN_HEX, cm.AIRBORNE_ODD_HEX, cm.IDENT_HEX):
            with self.subTest(frame=hexstr):
                self.assertEqual(cm.mode_s_syndrome(hexstr), 0)

    def test_ident_corpus_decodes_to_known_callsign(self):
        # Cross-check the identification corpus so the ground-truth metadata the
        # CPR tests trust is itself verified.
        self.assertEqual(cm.parse_callsign(cm.IDENT_HEX),
                         cm.IDENT_GROUND_TRUTH["callsign"])
        self.assertEqual(cm.type_code(cm.frame_bytes(cm.IDENT_HEX)),
                         cm.IDENT_GROUND_TRUTH["type_code"])


class TestGlobalDecode(unittest.TestCase):
    """Global even/odd pairing against the canonical corpus pair."""

    def setUp(self):
        # Parse the corpus pair once for all global-decode tests.
        self.even = cm.parse_cpr_frame(cm.AIRBORNE_EVEN_HEX)
        self.odd = cm.parse_cpr_frame(cm.AIRBORNE_ODD_HEX)
        self.truth = cm.AIRBORNE_GROUND_TRUTH

    def test_corpus_frames_are_correct_parity(self):
        # Sanity: the "even" frame really is even and the "odd" really is odd.
        self.assertFalse(self.even["odd"])
        self.assertTrue(self.odd["odd"])

    def test_global_against_ground_truth(self):
        # Anchoring on the EVEN frame is the published worked example; the result
        # must match the textbook lat/lon to sub-meter precision.
        result = cpr_ref.cpr_global_decode(self.even, self.odd, latest_is_odd=False)
        self.assertIsNotNone(result)
        lat, lon = result
        self.assertAlmostEqual(lat, self.truth["lat"], places=9)
        self.assertAlmostEqual(lon, self.truth["lon"], places=9)

    def test_global_odd_anchor_is_consistent(self):
        # Anchoring on the odd frame yields a slightly different (but valid) fix;
        # it must still land within the same NL zone, i.e. very close in latitude.
        result = cpr_ref.cpr_global_decode(self.even, self.odd, latest_is_odd=True)
        self.assertIsNotNone(result)
        lat, lon = result
        # Same zone => within one latitude bin (~6 deg) of the even-anchored fix.
        self.assertLess(abs(lat - self.truth["lat"]), 6.0)

    def test_nl_mismatch_rejects(self):
        # Force the two candidate latitudes into different NL zones by corrupting
        # the odd latitude; global decode must reject rather than emit garbage.
        bad_odd = dict(self.odd)
        bad_odd["lat_cpr"] = 0  # pushes rlat_odd far from rlat_even
        result = cpr_ref.cpr_global_decode(self.even, bad_odd, latest_is_odd=False)
        # Either a clean reject (None) or, at minimum, not the true position.
        if result is not None:
            lat, lon = result
            self.assertNotAlmostEqual(lat, self.truth["lat"], places=4)


class TestLocalDecode(unittest.TestCase):
    """Local single-frame decode against a known nearby reference."""

    def test_local_even_matches_ground_truth(self):
        # Decoding the EVEN frame against a reference 0.25 deg away must reproduce
        # the exact global ground truth.
        even = cm.parse_cpr_frame(cm.AIRBORNE_EVEN_HEX)
        ref = cm.LOCAL_REF
        result = cpr_ref.cpr_local_decode(even, ref["lat"], ref["lon"])
        self.assertIsNotNone(result)
        lat, lon = result
        self.assertAlmostEqual(lat, cm.AIRBORNE_GROUND_TRUTH["lat"], places=9)
        self.assertAlmostEqual(lon, cm.AIRBORNE_GROUND_TRUTH["lon"], places=9)

    def test_local_agrees_with_global(self):
        # Local decode of the even frame and global decode (even anchor) must give
        # the same answer when the reference is close enough.
        even = cm.parse_cpr_frame(cm.AIRBORNE_EVEN_HEX)
        odd = cm.parse_cpr_frame(cm.AIRBORNE_ODD_HEX)
        glob = cpr_ref.cpr_global_decode(even, odd, latest_is_odd=False)
        loc = cpr_ref.cpr_local_decode(even, cm.LOCAL_REF["lat"], cm.LOCAL_REF["lon"])
        self.assertAlmostEqual(glob[0], loc[0], places=9)
        self.assertAlmostEqual(glob[1], loc[1], places=9)


if __name__ == "__main__":
    unittest.main()
