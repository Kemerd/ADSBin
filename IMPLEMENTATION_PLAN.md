# ADSBin — Implementation Plan

> **ADSBin** = "ADS-B **In**" — a small, cheap, receive-only ADS-B box for general
> aviation. It listens to nearby aircraft and serves their positions to a display
> (EFB tablet, panel, or a companion box). A Novabox product, sibling to LidarAGL.
>
> Pun intended: **ADS-B In** is the actual aviation term for *receiving* (as opposed
> to **ADS-B Out**, which transmits). This box only ever receives — it is electrically
> incapable of transmitting, which is exactly what keeps it legal and liability-light.

---

## 0. Scope & Guiding Decisions

### MVP (build this first)
- **Single band: 1090 MHz (1090ES) traffic only.** No weather yet.
- **Hardware:** ESP32-P4 (USB 2.0 High-Speed host) + one Nooelec **NESDR Nano 3**
  (RTL2832U + **R820T2** tuner). The dongle's USB lines are wired directly to the
  P4's HS host port.
- **Output (MVP):** decoded traffic emitted as **GDL90** frames, plus a human-readable
  debug dump. GDL90 transport in the MVP is over **USB-CDC serial** (so a PC/test
  harness can validate it without WiFi). WiFi/UDP delivery to tablets comes later.
- **No GPS required for MVP** — see §5.4 (we decode absolute target positions via
  CPR even/odd pairing; the consuming display supplies its own ownship position).

### Deferred (designed-for, not built yet)
- **978 MHz UAT** — second dongle, adds free US weather (FIS-B) + UAT traffic.
- **WiFi GDL90 → EFB tablets** (ForeFlight / Garmin Pilot) via the on-board ESP32-C6
  companion radio (P4 has no native WiFi).
- **RS-232 / Garmin TIS-A output** to drive a panel display (e.g. G3X Touch).
- **GPS / NMEA ownship** for relative formats (TIS-A) + range/altitude filtering.

### Hard rules carried into the design
- **Build the superset, sell subsets.** Firmware auto-detects how many dongles /
  which bands are present and enables features accordingly. One binary, many SKUs
  (ADSBin / ADSBin Lite / ADSBin Go / ADSBin Panel).
- **No software feature-locks.** The decode core is derived from GPLv3 projects
  (dump1090 / dump978 / SoftRF lineage). Tier by *hardware populated*, never by a
  paywalled software gate — GPLv3 forbids it and customers hate it.
- **Receive-only, non-certified, experimental aircraft only.** Loud disclaimers,
  same posture as LidarAGL's DISCLAIMER.md.

---

## 1. Hardware

| Item | Part | Notes |
|---|---|---|
| Compute | **ESP32-P4** dev board (ideally one with on-board **ESP32-C6** for later WiFi) | Dual-core RISC-V ~400 MHz, USB 2.0 **HS** OTG host. No native WiFi/BT. |
| Receiver | **Nooelec NESDR Nano 3** (RTL2832U + R820T2) | USB-HS device. ~300–500 mA @ 5 V. Metal case helps thermals in a sealed box. |
| Antenna | 1090 MHz λ/4 whip (~68 mm) on SMA + small ground plane/counterpoise | DIY is fine. Keep away from digital noise sources (§8). |
| Power in | USB-C (5 V) — battery bank, plane USB-C, or hardwired | See §1.2. |
| Hardwire | Pigtail (power, optional future RS-232) tucked inside by default | User-exposed only when permanently installed. |
| Status | 1–2 LEDs (power / receiving-traffic heartbeat) | |
| Enclosure | Small **plastic/composite** box (RF-transparent) | Metal would block an internal antenna. Likely **no fan** (§7). |

### 1.1 USB wiring (dongle → P4)
The NESDR Nano's USB-A plug is wired **directly** to the P4's USB 2.0 HS host port:
- **D+ / D-** → P4 HS host data lines (keep the pair short, matched, away from noise).
- **VBUS (5 V)** → P4 supplies 5 V to the dongle (host mode = we power it). Confirm the
  board's 5 V rail can source the dongle's ~300–500 mA on top of the P4 itself.
- **GND** → common ground.

> The P4's **USB-C** port is used for **power + debug** (USB-CDC / USB-Serial-JTAG).
> The **HS OTG host** port is a *separate* interface dedicated to the dongle. Do not
> conflate them — most P4 boards break these out as different connectors.

### 1.2 Power
- Input: **USB-C 5 V** (plane USB-C, battery bank, or bench supply). Budget ≥ **1.5 A**
  headroom (P4 + dongle + future C6).
- Optional **hardwire pigtail** for permanent panel installs (e.g. ship's 5 V, or a
  small buck from 12/14 V — *out of MVP scope, but leave the footprint/pads*).
- Single firmware behaves identically regardless of power source — "glareshield today,
  panel tomorrow."

---

## 2. System Architecture

```
                 ┌──────────────────────────── ESP32-P4 ────────────────────────────┐
                 │                                                                    │
 [1090 antenna]  │  CORE 0 (RX + DSP, time-critical)      CORE 1 (decode + serve)     │
       │         │  ┌───────────────┐   ┌─────────────┐   ┌──────────────┐            │
   [NESDR Nano]──┼─►│ USB-HS host   │──►│ IQ ring buf │──►│ Mode-S /     │            │
   RTL2832U+     │  │ RTL-SDR driver│   │ + magnitude │   │ ADS-B decoder│            │
   R820T2        │  │ (bulk xfer)   │   │ + preamble  │   │ (DF17/18,CRC,│            │
                 │  └───────────────┘   │ + demod →   │   │  CPR,vel,id) │            │
                 │                       │  raw frames │   └──────┬───────┘            │
                 │                       └─────────────┘          │                    │
                 │                                          ┌─────▼───────┐            │
                 │                                          │ Traffic mgr │            │
                 │                                          │ (table+age) │            │
                 │                                          └─────┬───────┘            │
                 │                                                │                    │
                 │                              ┌─────────────────┼──────────────┐     │
                 │                              ▼                 ▼              ▼     │
                 │                       [GDL90 sink]     [debug dump]   [future sinks]│
                 │                       USB-CDC (MVP)    USB-CDC        WiFi / TIS     │
                 └────────────────────────────────────────────────────────────────────┘
```

**Threading / core strategy (critical for real-time):**
- **Core 0:** USB bulk transfers + DSP (magnitude, preamble correlation, bit slicing).
  This is the hard-real-time path; nothing else competes with it.
- **Core 1:** message decode, traffic table maintenance, output sinks, config, LEDs.
- Lock-free **ring buffer** between USB callback and the DSP consumer; a second queue
  hands raw candidate frames from DSP to the decoder.

---

## 3. Software Stack & Reuse (do NOT reinvent)

- **Framework:** **ESP-IDF** (P4-capable release), FreeRTOS, dual-core. Matches the
  LidarAGL toolchain. 
- **Proof it works:** there is already published **ESP32-P4 + RTL-SDR ADS-B firmware**
  (the LILYGO T-Display-P4 demo covered on rtl-sdr.com). **Locate it and use it as the
  reference / starting point** for the USB-host + demod path rather than writing the
  RTL-SDR stack from scratch.
- **RTL-SDR control:** port the **R820T2 + RTL2832U init/tuning sequence from
  `librtlsdr`** (register writes for sample rate, center freq, tuner gain). This is the
  fiddliest part; reuse, don't guess register values.
- **1090 demodulation + Mode-S/ADS-B decode:** adapt the **dump1090** demod core
  (magnitude LUT, preamble detect, 56/112-bit frame slice, CRC, DF17/18 parse, CPR).
- **(Later) 978 UAT:** adapt **dump978** for the second band.
- **GDL90 encoder:** implement per the **GDL 90 Data Interface Spec** + **ForeFlight
  GDL90 Extended Spec** (Heartbeat + Traffic Report messages).

### 3.1 Developer tooling & bench testing (build these alongside the firmware)
- **Windows `.bat` deploy scripts** (repo root, e.g. `tools/`) to one-shot the
  build → flash → monitor loop for fast P4 test iterations. (User runs them; we never
  invoke build/flash ourselves.) Keep them UTF-8 / Windows-console safe.
- **Python bench-test harness** (`tools/bench/`) that reads ADSBin's **USB-CDC output**
  and validates it **before** any WiFi work exists:
  - Decode/print the `sink_debug` traffic table.
  - Parse the `sink_gdl90` byte stream and verify Heartbeat + Traffic Report framing
    (CRC, byte-stuffing) against the GDL90 spec.
  - Optionally **inject canned ADS-B messages** (à la dump5892's terminal-injection
    mode) so decode logic can be tested deterministically without live air.
- Order of validation: **wired USB-CDC output first → only then** bring up WiFi/UDP
  GDL90 (Phase 10). Don't gate MVP correctness on the radio.

> ⚠️ **License gate:** dump1090/dump978/SoftRF forks are typically **GPLv3** (some
> antirez-lineage dump1090 is BSD — *check the exact fork's LICENSE before lifting
> code*). Whatever we pull in dictates ADSBin's license obligations. Document the
> provenance of every borrowed file in `THIRD_PARTY.md`.

---

## 4. Module Breakdown

Each module = its own component/folder under `components/`, with a clean header API so
sinks/sources can be swapped (supports the "auto-detect & tier" goal).

### 4.1 `usb_rtlsdr` — RTL-SDR USB-HS host driver
- Enumerate the RTL2832U over the P4 HS host; supply VBUS.
- Configure: sample rate (**2.4 Msps**, 8-bit I/Q), center freq **1090 MHz**, tuner
  **R820T2** gain (fixed high, ~49.6 dB, AGC off — best for ADS-B bursts).
- Continuous **bulk IN** transfers into the IQ ring buffer; never block the USB callback.
- Health: detect dongle present/absent, overflow, reset/re-enumerate on stall.
- **API:** `usb_rtlsdr_start(cfg)`, callback delivering IQ blocks; `usb_rtlsdr_count()`
  (returns # of dongles — feeds band auto-detection).

### 4.2 `demod1090` — 1090ES demodulator (Core 0)
- Compute **magnitude** from I/Q (precomputed LUT for 8-bit I/Q → mag).
- **Preamble** detection (correlate against the 8 µs 1090 preamble).
- **Bit slicing** (PPM) → 56-bit or 112-bit candidate frames.
- Hand candidate frames to `modes_decode` via queue. Keep this tight; it's the CPU hog.

### 4.3 `modes_decode` — Mode-S / ADS-B parser (Core 1)
- CRC check (24-bit Mode-S parity); accept DF17/DF18 (ADS-B / TIS-B).
- Extract **ICAO** address.
- Decode: **airborne position** (CPR), **velocity**, **identification/callsign**,
  **altitude** (Gillham/25 ft).
- **CPR:** implement **global** even/odd pairing (no ownship needed → absolute lat/lon)
  *and* **local** decode for when a reference position exists (faster, single-message).
- **API:** emits `adsb_msg_t` structs to the traffic manager.

### 4.4 `traffic` — traffic table manager
- Keyed by ICAO; merge position/velocity/ident updates per target.
- **Aging/expiry** (drop targets not heard for N seconds; configurable).
- Caps + sanity filters (impossible positions, optional range/altitude filter early to
  save CPU — mirrors dump5892's "too far/too high" cull).
- **API:** snapshot/iterator for sinks; counts for status LED + auto-detect.

### 4.5 `sinks` — pluggable output layer
- `sink_debug` — human-readable table to USB-CDC (bring-up & field debugging).
- `sink_gdl90` — encode **Heartbeat** + **Traffic Report** GDL90 messages.
  - **MVP transport:** USB-CDC byte stream (validate against a PC GDL90 decoder).
  - **Later transport:** WiFi UDP broadcast (port 4000) via the C6 — *same encoder*.
- **Future:** `sink_tis` (RS-232 Garmin TIS-A) to drive a panel display.
- Registration model so available sinks light up based on detected hardware/config.

### 4.6 `ownship` — reference position (stubbed for MVP)
- MVP: optional **manually configured** reference lat/lon (enables local CPR + range
  filtering). If absent, fall back to global CPR (still works).
- Later: GPS module or **NMEA-in from the panel** feeds live ownship.

### 4.7 `config` — NVS-backed settings
- Tuner gain, reference position, output mode(s), range/altitude filters, band map.
- Hold-at-boot or serial-command config (mirror LidarAGL's NVS menu philosophy).

### 4.8 `status` — LEDs / health
- Power LED; "traffic heard" heartbeat (blink on each decoded position).
- Internal temp watchdog (informs the no-fan decision, §7).

---

## 5. Key Technical Notes

### 5.1 Sample rate & data rate
2.4 Msps × 2 bytes (I/Q) ≈ **4.8 MB/s** over USB. Comfortably within USB-HS (480 Mbps);
**impossible on ESP32-S3** (Full-Speed 12 Mbps) — this is precisely why the P4 is required.

### 5.2 CPU budget (the real risk)
1090 demod at 2.4 Msps is heavy for a 400 MHz core. Mitigations:
- Dedicate **Core 0** entirely to USB+DSP; magnitude via **LUT**; tight preamble gate so
  full decode only runs on candidates.
- Early **range/altitude culling** before expensive CPR math.
- The existing P4 firmware demonstrates it's achievable — start from its DSP, profile early.

### 5.3 Tuner gain
ADS-B = weak bursts. Use **fixed maximum tuner gain**, RTL AGC **off**. Expose gain in
config for noisy installs.

### 5.4 Why no GPS is needed for MVP
GDL90 **Traffic Report** messages carry each target's **absolute** lat/lon/altitude. The
*display* (EFB) computes relative geometry from its own GPS. So ADSBin only needs absolute
target positions → **global CPR (even/odd pairing)** gives those with **no ownship fix**.
Ownship is only required for a **relative** output format (Garmin TIS-A, a later phase).

### 5.5 Frequency stability
The NESDR Nano's TCXO is *nice* but **not needed** for ADS-B (wide burst signal; decoders
tolerate drift). Don't design around needing it; a plain R820T2 works too.

---

## 6. Build Phases (hand these to the coder in order)

> Each phase ends with a concrete, testable result. Do the **entire** implementation of
> each phase — no stubs/TODOs/fake data unless explicitly flagged as a deferred phase.

- **Phase 0 — Scaffold.** ESP-IDF project targeting ESP32-P4. Partition table (room for
  future OTA dual-app). Dual-core FreeRTOS skeleton, logging over USB-CDC, status LED.
  Add the **`.bat` build/flash/monitor scripts** (§3.1) now so every later phase is one
  click to test.
- **Phase 1 — USB host bring-up.** Enumerate the NESDR Nano; pull raw bulk data; prove
  IQ bytes flow into the ring buffer at rate. (Validate byte rate / no overflow.)
- **Phase 2 — Tune it.** Port librtlsdr R820T2/RTL2832U init; set 1090 MHz / 2.4 Msps /
  max gain. Confirm tuned (sanity via raw magnitude histogram near a known emitter).
- **Phase 3 — Demod.** Magnitude LUT + preamble detect + bit slice → candidate frames;
  CRC. Count valid Mode-S frames/sec. Compare against `dump1090` on a PC fed the same air.
- **Phase 4 — Decode.** DF17/18 → ICAO, altitude, velocity, callsign, **CPR position**
  (global even/odd first; local with reference second).
- **Phase 5 — Traffic table.** Merge by ICAO, aging/expiry, range/alt filters, counts.
- **Phase 6 — Output.** `sink_debug` table + `sink_gdl90` over USB-CDC. Validate with the
  **Python bench harness** (§3.1) — confirm the debug table and that GDL90 framing/CRC
  parse cleanly on a PC *before* touching WiFi.
- **Phase 7 — Config + ownship stub.** NVS settings (gain, reference pos, filters,
  output mode). Optional manual reference position.
- **Phase 8 — Enclosure & field test.** Antenna placement, thermals, power sources;
  drive near an airport, count targets vs a reference receiver.

### Deferred phases (post-MVP)
- **Phase 9 — 978 UAT.** Second dongle auto-detected; dump978 path; weather/FIS-B.
- **Phase 10 — WiFi GDL90.** Bring up the **C6** (ESP-Hosted); UDP-broadcast GDL90 to
  tablets. (Reuses the Phase-6 encoder; only the transport changes.)
- **Phase 11 — Panel output.** RS-232 + **Garmin TIS-A** encoder (`sink_tis`) for G3X
  Touch et al. (relative format — needs ownship).

---

## 7. Thermal (the "probably no fan" question)
- One dongle + 1090-only ≠ a maxed P4. **Likely passive-cool-able.**
- Plan for it anyway: vents/airflow path, thermal pad from P4 (and dongle's metal case)
  to the enclosure wall, `status` temp watchdog logging worst-case temps in field test.
- **Decision rule:** if sustained junction temps stay in spec during Phase 8 field test
  → **no fan**. Keep a fan header/footprint as cheap insurance, unpopulated by default.

---

## 8. Antenna & RF placement (don't deafen yourself)
- Antenna **element** away from the P4, switching regulators, USB, and (later) the C6
  radio — these raise the noise floor and ADS-B is weak.
- Coax run **short** (loss at 1090 MHz). If it must be long → LNA at the antenna.
- Plastic/composite enclosure for an internal antenna; **metal box ⇒ external antenna**.
- Shield the digital section (small grounded can) if the antenna lives inside the box.
- Keep future 1090 / 978 / GPS / WiFi antennas physically separated to avoid desense.

---

## 9. BoM (MVP) & rough cost
| Part | ~Cost |
|---|---|
| ESP32-P4 (+C6) board | ~$30–40 |
| NESDR Nano 3 (R820T2) | ~$30–45 (or generic R820T2 ~$15–20) |
| 1090 λ/4 antenna (DIY) | ~$2–3 |
| Enclosure, USB-C jack, LEDs, wiring | ~$10 |
| **MVP total** | **~$55–80** |

(Dual-band later: + second dongle + 978 antenna + USB hub if needed.)

---

## 10. Risks & Mitigations
| Risk | Mitigation |
|---|---|
| P4 USB-host ecosystem still young | Start from the proven P4+RTL-SDR firmware; isolate the driver behind `usb_rtlsdr` API. |
| Demod too heavy for 400 MHz core | Dedicate Core 0; LUT magnitude; tight preamble gate; early culling; profile in Phase 3. |
| RTL-SDR heat/drift in sealed box | Metal-case dongle as heatsink; vents; temp watchdog; gain config. |
| VBUS brownout (dongle inrush) | Verify 5 V rail headroom (≥1.5 A); bulk caps near the dongle's VBUS tap. |
| GPL contamination of product | Quarantine borrowed code, `THIRD_PARTY.md`, tier by hardware not software locks. |
| No ownship → no relative (TIS-A) output | Fine for MVP (GDL90 absolute positions); add GPS/NMEA in Phase 11. |
| Liability (avionics-adjacent) | Receive-only, non-certified, experimental-only, loud DISCLAIMER. |

---

## 11. Repo Layout (proposed)
```
L:\Dev\ADSBin\
├─ IMPLEMENTATION_PLAN.md      (this file)
├─ LICENSE.md                  (PolyForm Noncommercial 1.0.0 — see GPL note in THIRD_PARTY)
├─ DISCLAIMER.md               (experimental / receive-only / not collision-avoidance)
├─ THIRD_PARTY.md              (provenance + licenses of borrowed code)
├─ licenses\                   (verbatim upstream license texts)
├─ tools\                      (.bat deploy scripts + Python bench harness — §3.1)
├─ CMakeLists.txt
├─ sdkconfig.defaults          (P4 target, USB host, dual-core)
├─ partitions.csv
├─ main\                       (app entry, task wiring, core pinning)
└─ components\
   ├─ usb_rtlsdr\              (§4.1)
   ├─ demod1090\              (§4.2)
   ├─ modes_decode\           (§4.3)
   ├─ traffic\                (§4.4)
   ├─ sinks\                  (§4.5: debug, gdl90[, tis, wifi])
   ├─ ownship\               (§4.6)
   ├─ config\                (§4.7)
   └─ status\                (§4.8)
```

---

## 12. Definition of Done (MVP)
- Powers from USB-C; dongle hosted directly off the P4 HS port.
- Receives real 1090ES traffic and decodes ICAO / altitude / velocity / position.
- Maintains an aging traffic table; emits valid **GDL90** (validated by a PC decoder)
  plus a readable debug table.
- Survives a field test near an airport with target counts comparable to a reference
  receiver, within thermal spec, **no fan**.
- One firmware binary; cleanly extensible to 978 + WiFi + TIS + audio without rework.
