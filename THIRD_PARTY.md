# Third-Party Code & License Compliance

This file tracks the provenance and license of every piece of outside code, data, or
design incorporated into **ADSBin**. Keep it current: **every time** borrowed code lands
in the repo, add a row and drop the upstream license text in `licenses/`.

> ADSBin itself is offered under the **[PolyForm Noncommercial License 1.0.0](LICENSE.md)**.
> Some dependencies below use licenses that may be **incompatible** with that choice — read
> §"License-compatibility warning" before incorporating or distributing anything.

---

## License-compatibility warning ⚠️ (read first)

The most natural building blocks for an ADS-B receiver — **dump1090**, **dump978**, and the
**SoftRF / dump5892** lineage — are commonly licensed under the **GNU GPLv3**. GPLv3 is a
**strong copyleft** license:

- A work that incorporates or derives from GPLv3 code must, when distributed, **also be
  offered under the GPLv3**, with complete corresponding source.
- GPLv3 **forbids adding further restrictions** — including a **noncommercial-only**
  restriction. **PolyForm Noncommercial is such a restriction.**

**Consequence:** you generally **cannot** ship a distributed binary that links GPLv3 ADS-B
code *and* place the whole under PolyForm Noncommercial. The two terms conflict. Before any
GPLv3-derived code is incorporated into a **distributed** ADSBin, pick one path:

1. **Use a permissively-licensed source instead.** e.g. the original **antirez dump1090**
   is **BSD**; permissive/MIT/BSD ADS-B and GDL90 implementations exist. Permissive code can
   live happily inside a PolyForm-Noncommercial project (preserve upstream notices).
2. **Release ADSBin's firmware under GPLv3** to match the copyleft dependency (drop or
   rework PolyForm for the distributed firmware; Novabox can still sell hardware + offer a
   separate commercial license for any independently-owned parts).
3. **Clean-room reimplement** the needed algorithms from public specifications (the ADS-B /
   Mode-S message formats, CPR, and GDL90 are all publicly documented) so no copyleft code is
   copied.

Decide and **record the decision here** before the first distribution. For private,
non-distributed bench builds this conflict is dormant — but resolve it before anything leaves
your bench.

---

## Component ledger

> Status legend: `planned` = intended, not yet in tree · `in-tree` = code present ·
> `cleared` = license reviewed & compatible/path chosen · `adapted` = permissive upstream
> structure/algorithm adapted (not copied verbatim), notice preserved · `clean-room` =
> reimplemented from a public spec/datasheet, no upstream code consulted · `original` =
> wholly original ADSBin code · `linked` = system/toolchain dependency used via public
> API, not vendored · `blocked` = conflict unresolved.
>
> **Path legend** (from the warning above): **Path 1** = use a permissive source ·
> **Path 2** = relicense to match copyleft · **Path 3** = clean-room from public spec.
>
> **Borrowing summary:** the only outside *code* genuinely adapted into the firmware is
> **antirez dump1090 (BSD-3-Clause)** — its envelope/preamble/PPM DSP, the 24-bit Mode-S
> CRC, the DF17/18 parity→ICAO recovery, and the CPR decoder *structure*. Everything else
> is either clean-roomed from a public spec/datasheet (RTL2832U, R820T2, GDL90), wholly
> original (config, ownship), or a non-vendored toolchain/runtime dependency
> (ESP-IDF Apache-2.0; pyserial BSD-3-Clause). **No GPLv2 librtlsdr and no GPLv3
> dump1090 fork (FlightAware / Malcolm Robb) code was copied or consulted** — the
> permissive-only path was taken deliberately to keep ADSBin under PolyForm Noncommercial.

| Component | Used for | Upstream | License (verified) | Status / path |
|---|---|---|---|---|
| ESP-IDF (USB Host, USB-Serial/JTAG, GPIO, esp_timer, NVS, temperature_sensor, FreeRTOS) | RTOS, USB-HS host enumeration + bulk-IN, EP0 control, byte output over P4 USB-C, persistent settings, die-temp, LED/time base, build | Espressif ESP-IDF (`usb`, `esp_driver_usb_serial_jtag`, `nvs_flash`, `driver`, `esp_timer`, FreeRTOS) | Apache-2.0 | `linked` — public APIs only, no source vendored |
| RTL2832U register/control interface (USB vendor requests, demod paging, I2C-repeater bit, resampler ratio, DEMOD_CTL/GPIO bias-tee) | USB control, raw-IQ baseband bring-up, sample-rate resampler, bias-tee GPIO — `components/usb_rtlsdr/usb_rtlsdr.c` | Realtek RTL2832U DVB-T COFDM + USB 2.0 datasheet (public) | Datasheet reference only — **no code copied** | `cleared` (clean-room) — **Path 3**; deliberately NOT adapted from GPLv2 `librtlsdr` |
| R820T2 tuner init / PLL tune / fixed-gain register sequence (regs 0x05–0x1F, I²C 0x1A, fractional-N PLL math, LNA/mixer/VGA manual gain, high-band tracking filter) | R820T2 power-up, LO tune to 1090 MHz+IF, fixed ~49.6 dB gain (AGC off), optional HW AGC — `components/usb_rtlsdr/usb_rtlsdr.c` | Rafael Micro R820T2 Register Description (rtl-sdr.com PDF) + R820T2 datasheet (public) | Register-description reference only — **no code copied** | `cleared` (clean-room) — **Path 3**; deliberately NOT adapted from GPLv2 `librtlsdr` |
| 1090ES demod front end (magnitude-LUT envelope, 8 µs preamble correlation, PPM half-bit Manchester slicing) | core DSP, re-derived for 2.4 Msps fractional samples-per-bit — `components/demod1090/demod1090.c` | antirez dump1090 (https://github.com/antirez/dump1090) | **BSD-3-Clause** | `adapted` — **Path 1**; structure adapted, no verbatim copy, BSD notice preserved in file header |
| Mode-S 24-bit CRC (poly 0xFFF409) + DF17/18 parity→ICAO recovery | table-driven CRC + extended-squitter parity validation — `components/modes_decode/modes_decode.c` | antirez dump1090 (https://github.com/antirez/dump1090) | **BSD-3-Clause** | `adapted` — **Path 1**; reimplemented byte-wise, BSD notice preserved (header + cpr.c). No GPLv3 fork consulted |
| CPR global/local decode + NL longitude-zone lookup | even/odd & single-message position resolution — `components/modes_decode/cpr.c` | antirez dump1090 (BSD) + ICAO Annex 10 Vol IV / RTCA DO-260B §2.2.3.2.3 | **BSD-3-Clause** (decoder structure) + public spec (NL numbers) | `adapted` — **Path 1**; decoder structure adapted, NL table clean-room from spec, BSD notice preserved in `cpr.c` |
| Gillham (Gray-coded Mode-C) 100-ft altitude decode | legacy Q=0 altitude path — `components/modes_decode/modes_decode.c` (`modes_gillham_to_ft`) | ICAO Annex 10 Mode-C spec; same fold as antirez dump1090 (BSD) | **BSD-3-Clause** / public spec | `adapted` — **Path 1**; public Gray-code fold, covered by the preserved BSD notice |
| GDL90 message encoder (CRC-16/CCITT 0x1021, 0x7E framing, 0x7D/0x20 byte-stuffing, Heartbeat/Traffic/Ownship, 24-bit semicircle lat/lon, 12-bit pressure altitude) | GDL90 output to EFB apps over USB-CDC — `components/sinks/gdl90_encoder.c` | Garmin "GDL 90 Data Interface Specification" 560-1058-00 Rev A + ForeFlight GDL90 Extended (public specs) | Public spec — **no code copied** | `cleared` (clean-room) — **Path 3**; CRC pinned to spec heartbeat vector 0x8BB3 in a host unit test. No GPLv3 fork consulted |
| config (NVS settings store) | persistent operator settings (tuner gain, ref position, band/sink maps, traffic cull filters) — `components/config/adsbin_config.c` | Original ADSBin code on ESP-IDF `nvs_flash` (Apache-2.0) | Original (PolyForm Noncommercial); ESP-IDF API Apache-2.0 | `original` — written from scratch against the frozen header; no third-party source |
| ownship (reference-position service, haversine distance) | ownship ref position + great-circle distance — `components/ownship/ownship.c` | none (original ADSBin code) | PolyForm Noncommercial (project license) | `original` — textbook geodesy, no code copied |
| Host bench: GDL90 reference (CRC-16/CCITT table + fold) | verifies device GDL90 frames — `tools/bench/gdl90.py`, `tools/bench/tests/gdl90_ref.py` | Garmin GDL 90 spec 560-1058-00 Rev A + ForeFlight extension (public) | Public spec — **no code copied** | `cleared` (clean-room) — **Path 3**; CRC table regenerated from poly 0x1021, cross-checked vs firmware |
| Host bench: Mode-S 24-bit CRC predictor | predicts injected-frame parity for inject-verify — `tools/bench/gdl90.py` (`modes_crc`) | ICAO Annex 10 Vol IV / RTCA DO-260B (public) | Public spec — **no code copied** | `cleared` (clean-room) — **Path 3**; bit-serial long-division from the generator polynomial |
| Host bench: CPR reference (NL/global/local) | host-side position verification — `tools/bench/gdl90.py`, `tools/bench/tests/cpr_ref.py` | ICAO Annex 10 / RTCA DO-260B CPR; antirez dump1090 lineage (BSD) | Public spec; BSD-compatible | `cleared` (clean-room) — **Path 3**; matches 40621D worked example; permissive-only, no GPL fork |
| Host bench: canned ADS-B test corpus | ground-truth lat/lon + callsign for CPR/ident tests — `tools/bench/canned_msgs.py`, `tools/bench/tests/canned_msgs.py` | Public DO-260B worked examples + antirez dump1090 (BSD) docs / mode-s.org | Reference data / facts (not licensable code); BSD-compatible | `cleared` — public example squitters, ground truth recomputed by host decoder |
| pyserial (host bench dependency) | cross-platform serial enumeration + USB-CDC transport — `tools/bench/cdc_link.py`, pinned `pyserial>=3.5` | https://github.com/pyserial/pyserial | **BSD-3-Clause** | `linked` — normal runtime dependency, **not vendored**; license recorded for attribution |

> No `⚠️` rows remain: every borrowing is either a non-vendored `linked` dependency, an
> `adapted` permissive (BSD-3-Clause) source with its notice preserved (**Path 1**), or a
> `clean-room` / `original` implementation (**Path 3**) with no copyleft code copied. The
> PolyForm-Noncommercial vs GPLv3 conflict is therefore **dormant and resolved** for the
> current tree. Re-evaluate if a future component (e.g. dump978 / UAT, or a Garmin TIS-A
> encoder) is added.

---

## How to add an entry
1. Add the dependency's full license file under `licenses/<name>-LICENSE.txt`.
2. Preserve all required upstream copyright/attribution notices in the source files.
3. Add/update the row above with the real (verified) license and status.
4. If it's copyleft, note in the row which resolution path (1/2/3) was taken and why.

---

*Required Notice: Copyright 2026 D Everett Hinton — Novabox.Works (https://novabox.works/)*
