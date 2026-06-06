# Upstream license texts

This directory holds the **verbatim** license file of every piece of third-party
code incorporated into ADSBin, one file per dependency
(e.g. `dump1090-antirez-LICENSE.txt`, `librtlsdr-LICENSE.txt`).

Currently present (the only genuinely-borrowed permissive sources):

- `dump1090-antirez-LICENSE.txt` — **BSD-3-Clause** (antirez dump1090). The firmware
  *adapts* its envelope/preamble/PPM DSP, the 24-bit Mode-S CRC, the DF17/18 parity→ICAO
  recovery, and the CPR decoder structure; the BSD notice is preserved in the source headers.
- `pyserial-LICENSE.txt` — **BSD-3-Clause** (pyserial). A non-vendored runtime dependency
  of the host bench harness (`tools/bench/cdc_link.py`); recorded for attribution.

Everything else in the tree is clean-roomed from a public spec/datasheet (RTL2832U,
R820T2, GDL90), wholly original (config, ownship), or a non-vendored ESP-IDF (Apache-2.0)
toolchain dependency — see `THIRD_PARTY.md` for the full ledger.

Before any further external code lands in the tree, follow the procedure in
[`THIRD_PARTY.md`](../THIRD_PARTY.md):

1. Drop the upstream license here as `licenses/<name>-LICENSE.txt`.
2. Preserve the upstream copyright/attribution notices in the borrowed source.
3. Add/flip the matching row in `THIRD_PARTY.md` to the verified license + status.

See the **license-compatibility warning** at the top of `THIRD_PARTY.md` — the
PolyForm-Noncommercial vs GPLv3 question must be resolved before any copyleft
ADS-B code is incorporated into a distributed build.
