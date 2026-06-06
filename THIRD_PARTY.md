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
> `cleared` = license reviewed & compatible/path chosen · `blocked` = conflict unresolved.

| Component | Used for | Upstream | License (verify!) | Status |
|---|---|---|---|---|
| ESP-IDF + components | RTOS, USB host, drivers, build | Espressif | Apache-2.0 (verify) | planned |
| RTL2832U / R820T2 init | tuner + demod control | `librtlsdr` (Osmocom) | **GPLv2+** (verify) | planned ⚠️ |
| 1090ES demod + Mode-S/ADS-B decode | core decode | dump1090 — **which fork matters** | antirez = **BSD**; Malcolm Robb/FlightAware forks = **GPLv3** | planned ⚠️ |
| ESP32-P4 + RTL-SDR reference firmware | USB-host + demod starting point | LILYGO T-Display-P4 ADS-B demo (per rtl-sdr.com) | **verify** | planned ⚠️ |
| 978 UAT decode (later) | weather/UAT band | dump978 | **verify** | planned ⚠️ |
| GDL90 encoder | output to EFB/displays | GDL90 spec (Garmin) + ForeFlight GDL90 Extended spec | spec is public; any borrowed impl = verify | planned |
| Garmin TIS-A encoder (later) | RS-232 to panel | community reverse-engineering | format not officially published | planned ⚠️ |

> Each `⚠️` row must be moved to **cleared** (with the chosen path from the warning above) or
> **blocked** before that code is incorporated into a distributed build.

---

## How to add an entry
1. Add the dependency's full license file under `licenses/<name>-LICENSE.txt`.
2. Preserve all required upstream copyright/attribution notices in the source files.
3. Add/update the row above with the real (verified) license and status.
4. If it's copyleft, note in the row which resolution path (1/2/3) was taken and why.

---

*Required Notice: Copyright 2026 D Everett Hinton — Novabox.Works (https://novabox.works/)*
