# ADSBin Host-Only Unit Tests

Pure-Python tests that prove the **algorithms** behind the firmware on a PC, with
**no ESP32-P4, no RTL-SDR, and no firmware build**. They are the host-side twin of
the frozen device contracts:

| Test file | Proves | Mirrors |
|---|---|---|
| `test_cpr.py` | CPR `cpr_nl`, global even/odd pairing, local single-frame decode against ground truth | `components/modes_decode/include/cpr.h` |
| `test_gdl90.py` | GDL90 CRC-16, byte-stuffing, framing/deframing round-trip, semicircle lat/lon + 12-bit altitude packing | `components/sinks/include/gdl90_encoder.h`, `tools/bench/WIRE_CONTRACT.md` §3 |
| `test_sink_debug.py` | `sink_debug` traffic-table token render/parse round-trip | `tools/bench/WIRE_CONTRACT.md` §1 |

The whole point is **independent verification**: the reference modules below are a
clean-room reimplementation of the same specs the firmware targets. If the device
code and these references agree on the same corpus, both are almost certainly
right — and any disagreement is a real firmware bug, not a shared mistake.

## Reference modules (reusable by the live bench harness)

- `cpr_ref.py` — clean-room CPR math (ICAO Annex 10 / DO-260B), API-aligned with
  `cpr.h`.
- `gdl90_ref.py` — clean-room GDL90 transport + field packing (Garmin GDL90 spec
  + ForeFlight Extended). The CRC table is **regenerated** from the CCITT
  polynomial at import, never transcribed.
- `sink_debug_ref.py` — `render_target()` / `parse_target()` for the WIRE_CONTRACT
  token line. The live harness in `tools/bench/` can import `parse_target()`
  directly so it can never drift from what these tests validate.
- `canned_msgs.py` — ground-truth ADS-B frame corpus (real, CRC-valid frames with
  textbook-known decoded values).

## Running

```bat
run_tests.bat
run_tests.bat -v      :: verbose, one line per test
```

Or directly with any Python 3.x:

```bat
python -m unittest discover -s . -p "test_*.py"
```

No third-party packages are required — standard-library `unittest` only.

## Ground truth used by `test_cpr.py`

The canonical airborne-position even/odd pair for ICAO `0x40621D`:

```
even = 8D40621D58C382D690C8AC2863A7
odd  = 8D40621D58C386435CC412692AD6
  -> lat 52.2572021484375, lon 3.91937255859375   (even-anchored)
```

and a TC-4 identification frame for ICAO `0x4840D6` decoding to callsign
`KLM1023`. Each frame's 24-bit Mode-S parity has a zero syndrome (asserted in the
tests), so they are genuine air-interface frames, not synthetic.
