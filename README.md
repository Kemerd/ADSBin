# ADSBin

**A software-defined ADS-B receiver built from scratch on the dual-core RISC-V ESP32-P4.** USB
High-Speed SDR ingest, real-time Mode-S / CPR decode, and GDL90 emission over WiFi to ForeFlight or
hardwired to a panel display — the full *receive → decode → serve* chain on a **$23 microcontroller**,
for a complete build **under $100**. One firmware image, no app subscriptions, no license gates.

> **The whole signal chain lives in firmware.** A commodity RTL-SDR delivers 2.4 Msps of raw 8-bit
> I/Q over USB 2.0 High-Speed; the P4 demodulates 1090ES on a hard-real-time core (magnitude LUT →
> preamble correlation → PPM bit-slice), runs Mode-S CRC and global/local CPR position solving on the
> second core, ages a per-aircraft traffic table, and re-emits standard GDL90. No PC, no cloud, no
> vendor app — just an MCU doing DSP that usually wants a laptop.

| Tier | What you get | Total |
|---|---|---|
| **Tier 1** | Traffic only (1090ES) | **≈ $81** |
| **Tier 2** | Traffic + Weather (1090 + 978 UAT/FIS-B) | **≈ $128** |
| **Tier 3** | Traffic + Weather + GPS ownship | **≈ $143** |

ADSBin decodes 1090 MHz ADS-B broadcasts from nearby aircraft using an RTL-SDR dongle and re-emits
them as GDL90 traffic for an EFB (ForeFlight, Garmin Pilot) or a panel display. The RTL-SDR is a
receive-only radio that cannot transmit on 1090/978 MHz — the only RF it emits is the WiFi link that
delivers GDL90 to your tablet. The name is the aviation term *ADS-B In* (receiving), as opposed to
*ADS-B Out* (transmitting), which ADSBin does not and cannot do.

---

## Overview

ADS-B is a surveillance protocol that most aircraft broadcast in the clear on 1090 MHz: ICAO
address, GPS-derived position, barometric altitude, velocity, and callsign. ADSBin receives that
broadcast, demodulates and decodes Mode-S / ADS-B (DF17/DF18), resolves target positions via CPR,
maintains an aging traffic table, and emits GDL90 — the traffic format EFBs and panel displays
consume.

What makes it interesting as an engineering exercise: **none of this normally runs on a
microcontroller.** ADS-B decode is typically a laptop-class workload (dump1090 on a Raspberry Pi or
PC). ADSBin moves the entire pipeline onto a $23 dual-core RISC-V part by treating the radio path as
hard real-time:

- **USB 2.0 High-Speed SDR host** — the P4 acts as USB host to a commodity RTL2832U + R820T2 dongle
  and streams **2.4 Msps of 8-bit I/Q (~4.8 MB/s)** in over bulk-IN. High-Speed is a hard
  requirement; that rate exceeds the Full-Speed bandwidth on smaller ESP32 parts, which is why the P4
  is the floor.
- **Real-time DSP, core-pinned.** Sample ingest and the 1090ES demodulator (magnitude LUT → 8 µs
  preamble correlation → PPM bit-slice) own Core 0 and are never preempted; Mode-S CRC, DF17/18
  parse, and global/local CPR position solving run on Core 1 behind lock-free queues.
- **Two ways out.** GDL90 goes to ForeFlight / Garmin Pilot over **WiFi** (on-board ESP32-C6 SoftAP +
  UDP broadcast) *or* hardwired to a panel display — the same transport-agnostic encoder feeds both.

The hardware is an ESP32-P4 (dual-core RISC-V, USB 2.0 High-Speed host) driving one or two RTL2832U +
R820T2 dongles; the second, when present, is auto-assigned to 978 MHz UAT for free FIS-B weather.

## Form factor and power

ADSBin is the size of a thumb drive — about **3 in long × 1 in wide**, tapering from **½ in tall at
its thinnest to 1 in at its highest**. It fits in a pocket or clips out of the way on the panel.

The cutting-edge ESP32-P4 runs cool enough to stay passively cooled at this size, so there's no fan
and no heat to manage. Power it however the cockpit allows:

- **USB port** — plug it into an aircraft USB outlet and it runs off ship power.
- **Battery bank** — any USB power bank makes it fully portable, no aircraft wiring required.

Antenna is just as flexible: use the **built-in antenna** for a self-contained receiver, or wire up a
**permanent external antenna** for better range and a fixed install.

## Rationale

Commercial ADS-B receivers are comparatively expensive, and some gate decode features behind an app
subscription despite the hardware being capable. The decode path is well-documented and the radio is
a commodity RTL-SDR, so the full receive → decode → output chain can be implemented on low-cost
hardware. ADSBin does that from a **~$73 traffic build up to a ~$128 traffic + weather build** (see
the [bill of materials](#bill-of-materials)), with source available for inspection and modification
under a noncommercial license.

Design constraints carried throughout:

- **One firmware image.** Features are enabled by *detected hardware* (dongle count, bands present),
  not by build flags or license gates.
- **No GPS required.** Aircraft broadcast their own absolute position; ADSBin resolves it with global
  CPR (even/odd pairing) and emits absolute lat/lon. A reference position is only needed for relative
  output formats (a later phase).
- **Receive-only, non-certified.** Intended for experimental aircraft and bench use. See
  [DISCLAIMER.md](DISCLAIMER.md).

---

## Architecture

The hard-real-time radio path is pinned to Core 0; everything else runs on Core 1, so the sample
ingest is never preempted by decode or output work.

```
                 ┌──────────────────────────── ESP32-P4 ────────────────────────────┐
                 │                                                                    │
 [1090 antenna]  │  CORE 0  (RX + DSP, hard real-time)   CORE 1  (decode + serve)     │
       │         │  ┌───────────────┐   ┌─────────────┐   ┌──────────────┐            │
   [NESDR Nano]──┼─►│ usb_rtlsdr    │──►│  IQ ring    │──►│ demod1090    │──┐         │
   RTL2832U +    │  │ USB-HS host   │   │ (lock-free) │   │ mag-LUT +    │  │ frames  │
   R820T2        │  │ bulk-IN       │   └─────────────┘   │ preamble +   │  │ (queue) │
                 │  └───────────────┘                     │ bit-slice    │  ▼         │
                 │                                         └──────────────┘ ┌────────┐ │
                 │                                                          │modes_  │ │
                 │                          ┌───────────────────────────── │decode  │ │
                 │                          ▼   adsb_msg (queue)            │CRC/CPR │ │
                 │                   ┌─────────────┐                        └────────┘ │
                 │                   │  traffic    │  merge by ICAO, age, range/alt cull │
                 │                   └──────┬──────┘                                    │
                 │                          │ snapshot                                  │
                 │              ┌───────────┴───────────┐                               │
                 │              ▼                       ▼                               │
                 │        [sink_debug]            [sink_gdl90]   (+ future WiFi / TIS)  │
                 │         USB-CDC                  USB-CDC                              │
                 └────────────────────────────────────────────────────────────────────┘
```

| Stage | Component | Core | Function |
|---|---|---|---|
| 1 | `usb_rtlsdr` | 0 | USB-HS host bring-up; tune R820T2 to 1090 MHz @ 2.4 Msps; stream raw I/Q into a lock-free ring. |
| 2 | `demod1090` | 0 | Magnitude LUT → 8 µs preamble correlation → PPM bit-slice → 56/112-bit candidate frames. Non-blocking. |
| 3 | `modes_decode` | 1 | Mode-S CRC, DF17/18 parse, ICAO, callsign, altitude, velocity, and CPR position resolution (global + local) → absolute lat/lon. |
| 4 | `traffic` | 1 | One record per aircraft, keyed by ICAO. Merge updates, age out stale targets, optional range/altitude/sanity culling. |
| 5 | `sinks` | 1 | Pluggable outputs: a debug table and a GDL90 stream (Heartbeat + Traffic Report), both over USB-CDC. |

Supporting components: `config` (NVS-backed settings), `ownship` (optional reference position),
`status` (LEDs + internal die-temperature watchdog), `common` (the shared type contract every
component compiles against). Full design rationale: [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).

---

## Bill of materials

The firmware is **one binary that auto-tiers by the hardware you populate** — plug one dongle and
you have a traffic receiver; plug a second and the box auto-detects it, assigns it to 978 MHz, and
adds free FIS-B weather. Build the cheapest tier now and upgrade later without reflashing. All prices
are approximate and from the linked vendors at the time of writing.

### Shared base (every tier)

| Part | Specification | Cost | Link |
|---|---|---|---|
| Compute | **Waveshare ESP32-P4-WIFI6** dev board. Dual-core RISC-V, USB 2.0 HS host, on-board ESP32-C6 (WiFi → ForeFlight). | **$23.03** | [Amazon B0FKN8GCW6](https://www.amazon.com/dp/B0FKN8GCW6) |
| USB splitter | **1-to-4 USB-A expansion board** (breaks the P4's single HS host port out to multiple ports for the dongle(s)). | **$1.66** | [AliExpress](https://www.aliexpress.us/item/3256804070458646.html) |
| USB-A ports | USB-A connectors (5-pack; you need ~2). | **$1.54 / 5** | [AliExpress](https://www.aliexpress.us/item/3256808331104442.html) |
| Enclosure | **3D-printed PA612-CF** (carbon-fibre nylon; RF-transparent enough for the internal radio, stiff and heat-tolerant). | **~$1** filament | print it yourself |
| Fasteners | M2 self-tapping screws (case assembly). | **<$1** | hardware store |
| Power | USB-C, 5 V, ≥ 1.5 A headroom. | — | — |

> **Assembly note:** requires a **soldering iron** (USB-A ports → expansion board) and basic M2
> tapping. No exotic tools.

### Tier 1 — Traffic only (1090ES)

Shared base **+ one** receiver. One dongle is always assigned the **1090 traffic** role, in either
port.

| Part | Cost | Link |
|---|---|---|
| Nooelec NESDR Nano 3 (RTL2832U + R820T2, 0.5 ppm TCXO) | **$44.95** | [nooelec.com](https://www.nooelec.com/store/nesdr-nano-three.html) |
| Nooelec 1090MHz Antenna | **$7.95** | [nooelec.com](https://www.nooelec.com/store/sdr/sdr-addons/antennas/1090mhz-ads-b-antenna-5dbi-sma.html)

**Tier 1 total: ≈ $80.95** (base ≈ $28 + one Nano 3).

### Tier 2 — Traffic + Weather (1090 + 978 UAT/FIS-B)

Shared base **+ two** receivers. The firmware auto-assigns the first dongle to **1090 traffic** and
the second to **978 weather**; weather streams to ForeFlight as GDL90 Uplink (FIS-B) frames.

**Easiest path — one bundle covers both dongles and every antenna:**

| Part | Cost | Link |
|---|---|---|
| **Nooelec Stratux Nano 3 bundle** — two NESDR Nano 3 receivers **plus all the antennas you need** (both 1090 + 978 bands). One SKU, no loose parts to source. | **$99.95** | [Stratux Nano 3 bundle](https://www.nooelec.com/store/stratux-bundle-nano-3.html) |

**Tier 2 total (bundle): ≈ $128** (base ≈ $28 + Stratux bundle $99.95).

**Or source the parts loose:**

| Part | Cost | Link |
|---|---|---|
| 2× NESDR Nano 3 | **$89.90** (2×$44.95) | [nooelec.com](https://www.nooelec.com/store/nesdr-nano-three.html) |
| Antennas — the bands are tuned differently, so a 978 antenna is required. The **ADS-B Discovery 5 dBi dual-band antenna bundle** covers both 1090 + 978. | **$19.95** | [antenna bundle](https://www.nooelec.com/store/ads-b-discovery-antenna-bundle-5dbi.html) |

**Tier 2 total (loose): ≈ $138** (base ≈ $28 + two Nano 3 ≈ $90 + antenna bundle ≈ $20).

### Tier 3 — Traffic + Weather + GPS (ownship)

Tier 2 **+** a GPS module for live ownship position (enables relative formats and range/altitude
filtering). *GPS firmware support is not yet implemented — listed so you can buy ahead.*

| Part | Cost | Link |
|---|---|---|
| GPS / NMEA module | **~$15** | [AliExpress](https://www.aliexpress.us/item/3256810616275036.html) |
| External GPS antenna *(optional)* — for a fixed mount / better sky view than the on-module patch. | **$2.94** | [AliExpress](https://www.aliexpress.us/item/2251832872907030.html) |
| IPEX → SMA adapter *(required by the external antenna)* — 2-pack. | **$2.71 / 2** | [AliExpress](https://www.aliexpress.us/item/3256809083730864.html) |

**Tier 3 total: ≈ $143** (Tier 2 bundle ≈ $128 + GPS ≈ $15). Add **≈ $6** for the optional external GPS antenna + IPEX→SMA adapter.

> A single 1090-only dongle is expected to be passively coolable. With two dongles + WiFi the box
> works harder; the firmware samples and logs peak die temperature for thermal validation before the
> enclosure is sealed.

---

## Wiring

The P4 exposes two distinct USB interfaces:

| Port | Use |
|---|---|
| USB-C | Power and debug — flashing, logs, and the GDL90/debug output, over the on-board USB-UART. |
| USB-HS host | The RTL-SDR dongle(s), via the 1-to-4 USB expansion board. A separate interface dedicated to sample ingest. |

The single HS host port feeds the **1-to-4 USB expansion board**, which breaks out the ports the
dongles plug into:

```
                                 ┌── port ──► [NESDR Nano #1]── SMA ── 1090 MHz antenna  (traffic)
ESP32-P4 ── HS host ── [1→4 USB expansion board] ──┤
                                 └── port ──► [NESDR Nano #2]── SMA ── 978 MHz antenna   (weather)
  VBUS (5 V) powers the board + dongles (host mode, ~300–500 mA each)
```

### Auto-role assignment (zero config)

- **One dongle plugged (either port): always 1090 traffic.**
- **Two dongles plugged: the first is 1090 traffic, the second is 978 UAT weather** (FIS-B).
- Roles are bound to the **physical hub-port position** (so the correctly-tuned antenna stays on the
  right band), with the dongle's USB serial recorded as a stable label. Live hotplug is supported —
  add or pull the weather dongle while running and traffic never interrupts.

> **Antennas are band-specific.** A 1090 antenna (~68 mm) is not a 978 antenna (~77 mm); the dual-band
> [Discovery 5 dBi bundle](https://www.nooelec.com/store/ads-b-discovery-antenna-bundle-5dbi.html)
> covers both. Keep the two antennas physically separated to avoid desense.

### Staged testing / role override

Getting the dongles one at a time? Force a **single** dongle to the weather role to test 978 before
the second arrives, over the USB-CDC console:

```
+ROLE 978      → force the lone dongle to 978 weather
+ROLE 1090     → force it to 1090 traffic (the default)
+ROLE auto     → back to count-based auto-assignment
```

The override persists in NVS. With two dongles it is ignored — count-based assignment takes over.

ADS-B is a weak signal; minimize noise coupling: keep the antenna elements away from the P4,
switching regulators, and USB lines; keep coax short (1090/978 MHz is lossy); use a plastic enclosure
(PA612-CF) for internal antennas. Verify the 5 V rail can source ~300–500 mA **per dongle** on top of
the P4, and place a bulk capacitor near each dongle's VBUS to handle inrush.
Full RF/placement notes: [IMPLEMENTATION_PLAN.md §8](IMPLEMENTATION_PLAN.md).

### GPS (u-blox MAX-M10S) — optional live ownship

A GPS module gives the box a **live ownship position** (the blue "you are here" airplane in
ForeFlight) and a **GPS-disciplined clock**. It's entirely optional: with no module wired the
firmware runs exactly as before and no ownship is sent. Wire the module's 5 pins to the P4 header:

| MAX-M10S pin | P4 pin | Kconfig symbol | Required? |
|---|---|---|---|
| **VCC** | 3.3 V | — | ✅ |
| **GND** | GND | — | ✅ |
| **TX** | **GPIO20** | `ADSBIN_GPS_UART_RX_GPIO` | ✅ — only wire needed for ownship |
| **RX** | **GPIO21** | `ADSBIN_GPS_UART_TX_GPIO` | optional — UBX config burst |
| **PPS** | **GPIO22** | `ADSBIN_GPS_PPS_GPIO` | optional — precise 1PPS clock layer |

The module's **TX** goes to the P4's **RX** and vice-versa (UART is crossed). GPIO20/21/22 are free,
non-strapping pins clear of the UART0 console, the C6 SDIO block (GPIO14–19, reset GPIO54), and I2C
(GPIO7/8). **Verify they're broken out on your board's 2×20 header** before wiring; if your board
exposes a different free trio, just change the three Kconfig defaults — no code change. Set any pin to
`-1` in `menuconfig → ADSBin` to disable that wire; `ADSBIN_GPS_UART_RX_GPIO = -1` disables GPS
entirely. The module is 3.3 V native — do **not** feed 5 V to its RX pin. Keep the GPS power/ground
return away from the RTL-SDR USB-HS lines so digital noise doesn't desense the GNSS front-end.

The clock auto-degrades through five quality levels (PPS-disciplined → holdover → NMEA fix →
free-running → off) and promotes/demotes itself as the signal comes and goes — no configuration.

**External antenna (optional).** The MAX-M10S ships with an on-board patch antenna and also exposes a
**u.FL / IPEX** socket (same connector, two names). For a fixed install, run an
[IPEX → SMA adapter](https://www.aliexpress.us/item/3256809083730864.html) from that socket out to the
enclosure wall and screw on the [external GPS antenna](https://www.aliexpress.us/item/2251832872907030.html)
— a clear sky view gives a faster, more reliable fix than the patch buried inside the case. No firmware
change; the module doesn't care which antenna feeds it. Keep the antenna lead clear of the RTL-SDR
USB-HS lines so digital noise doesn't desense the GNSS front-end.

---

## Toolchain setup

Builds with ESP-IDF v6.0.1 (RISC-V GCC 15.2), target `esp32p4`.

1. Install ESP-IDF v6.0.1 and verify the toolchain (`idf.py --version` in an ESP-IDF terminal).
2. Open an ESP-IDF terminal so `idf.py` is on `PATH`, or source the export script:
   ```powershell
   . C:\esp\v6.0.1\esp-idf\export.ps1
   ```
3. Set the target once (regenerates `sdkconfig` from `sdkconfig.defaults`):
   ```
   tools\set-target.bat        # idf.py set-target esp32p4
   ```

The USB Host Library was removed from ESP-IDF core in v6.0. ADSBin declares it as the managed
component `espressif/usb` in `components/usb_rtlsdr/idf_component.yml`; the component manager fetches
it on the first build (network access required for that build).

Flash size defaults to 16 MB. Adjust the `CONFIG_ESPTOOLPY_FLASHSIZE_*` line in `sdkconfig.defaults`
for other boards; the partition layout (dual 3 MB OTA app slots) also fits in 8 MB.

---

## Build and flash

Deploy scripts in [`tools/`](tools/), run from an ESP-IDF terminal:

| Command | Action |
|---|---|
| `tools\set-target.bat` | One-time: set chip to ESP32-P4. |
| `tools\build.bat` | Build only. |
| `tools\bfm.bat [COMx]` | Build, flash, and monitor. Omit `COMx` to auto-detect. |
| `tools\flash.bat [COMx]` | Flash and monitor (no rebuild). |
| `tools\monitor.bat [COMx]` | Serial monitor (`Ctrl-]` to exit). |

Initial bring-up:

```
tools\set-target.bat
tools\bfm.bat COM5          # P4 USB-C serial port
```

Expected output: the boot banner (chip, cores, IDF version), both cores reporting in, and — with no
dongle attached — a 1 Hz heartbeat. Attaching the dongle starts the decode pipeline.

---

## Output

Over the USB-C serial link, ADSBin emits two streams:

`sink_debug` — a human-readable traffic table, one line per aircraft:

```
=== ADSBIN TRAFFIC 3 @ ... ===
ICAO=A1B2C3 CS=UAL123 LAT=37.6189 LON=-122.3750 ALT=12000 GS=320 TRK=095 VR=-640 MSGS=42 SEEN=120
=== END ===
```

`sink_gdl90` — a GDL90 byte stream (Heartbeat + Traffic Report) consumable directly by an EFB.

Both use USB-CDC in the current build, allowing full validation from a PC before WiFi exists. The
GDL90 encoder is transport-agnostic; WiFi delivery (UDP broadcast to ForeFlight / Garmin Pilot)
reuses the same encoder and changes only the transport. The wire format is specified in
[`tools/bench/WIRE_CONTRACT.md`](tools/bench/WIRE_CONTRACT.md).

---

## Bench validation (no radio required)

The Python harness in [`tools/bench/`](tools/bench/) communicates over USB-CDC and can inject canned
frames into the live decode path:

```
cd tools\bench
python -m pip install -r requirements.txt          # pyserial only

python adsbin_bench.py list-ports                   # enumerate serial ports
python adsbin_bench.py validate-gdl90 --port COM5   # parse and CRC-check live GDL90 frames
python adsbin_bench.py dump-debug --port COM5       # print the live traffic table
python adsbin_bench.py inject --port COM5 <hex>     # inject one raw Mode-S frame
python adsbin_bench.py inject-verify --port COM5    # inject the canned corpus and assert decode output
python adsbin_bench.py list-canned                  # list built-in test frames + ground truth
```

`inject-verify` is the acceptance gate: it pushes known frames (including matched CPR even/odd pairs
with ground-truth positions) through the firmware decode path and verifies the output.

Pure host unit tests (CPR math, GDL90 CRC/framing, debug-table round-trip) run without hardware:

```
cd tools\bench\tests
run_tests.bat          # 44 tests
```

---

## Configuration

Runtime settings persist in NVS ([`components/config`](components/config/)): tuner gain, optional
manual reference position, range/altitude filters, band map, and enabled sinks. Defaults: 49.6 dB
fixed gain (AGC off), debug + GDL90 enabled.

Build-time options (`idf.py menuconfig → ADSBin`):

- `ADSBIN_STATUS_LED_GPIO` — heartbeat LED GPIO. Default `-1` (disabled); board-specific. The console
  heartbeat is independent of this pin.
- `ADSBIN_STATUS_LED_ACTIVE_LOW` — set for active-low LEDs.
- `ADSBIN_GPS_UART_RX_GPIO` — P4 RX ← GPS TX. Default `20`. **Set to `-1` to disable GPS entirely.**
- `ADSBIN_GPS_UART_TX_GPIO` — P4 TX → GPS RX (UBX config wire). Default `21`; `-1` skips the config burst.
- `ADSBIN_GPS_PPS_GPIO` — P4 ← GPS 1PPS. Default `22`; `-1` disables the precise clock-discipline layer.
- `ADSBIN_GPS_UART_NUM` / `ADSBIN_GPS_BAUD` — GPS UART controller (default `1`) and baud (default `9600`).
  See the [GPS wiring section](#gps-u-blox-max-m10s--optional-live-ownship) for the pin map.

---

## Repository layout

```
ADSBin/
├─ IMPLEMENTATION_PLAN.md     design and rationale
├─ THIRD_PARTY.md             provenance and license of all borrowed code
├─ LICENSE.md  DISCLAIMER.md  license and safety terms
├─ licenses/                  verbatim upstream license texts
├─ tools/                     deploy scripts + Python bench harness (bench/, bench/tests/)
├─ sdkconfig.defaults  partitions.csv   target config / dual-OTA layout
├─ main/                      app entry, pipeline wiring, core pinning, +INJECT console
└─ components/
   ├─ common/                 shared type ABI (adsbin_types.h)
   ├─ usb_rtlsdr/             RTL-SDR USB-HS host driver     (Core 0)
   ├─ demod1090/              1090ES demodulator             (Core 0)
   ├─ modes_decode/           Mode-S / ADS-B parser + CPR    (Core 1)
   ├─ traffic/                traffic table manager          (Core 1)
   ├─ sinks/                  debug + GDL90 outputs          (Core 1)
   ├─ ownship/                optional reference position
   ├─ config/                 NVS-backed settings
   └─ status/                 LEDs + temperature watchdog
```

---

## Status

| Item | State |
|---|---|
| Firmware build (ESP-IDF v6.0.1 / esp32p4) | Builds clean. `adsbin.bin` ≈ 835 KB; fits the 3 MB app slot (73% free). |
| Host unit tests (CPR / GDL90 / RS-FEC / UAT demod+decode / debug format) | All passing (incl. the new 978/UAT + GDL90 0x07 suites). |
| Full 1090 pipeline (USB → DSP → decode → traffic → sinks) | Flashed and running on the P4; GDL90 stream CRC-clean; injected CPR pairs resolve to ground truth. |
| Dual-dongle / auto-role (1090 traffic + 978 weather) | Implemented and on-silicon: the driver allocates a per-role IQ ring and assigns roles by hub-port position + serial. |
| 978 UAT / FIS-B weather (demod978 + uat_decode + GDL90 0x07 sink) | Implemented; live-hotplugs in/out without interrupting traffic. Awaiting a second dongle for on-air weather validation. |
| `+ROLE` single-dongle override | Verified on hardware (`+OK`), persists to NVS — force a lone dongle to 978 for staged weather testing. |
| GDL90 NIC/NACp fields; raw-frame debug (`RAW=`) | Not yet wired through `adsb_msg_t`; does not block the build. |

Builds, flashes, boots, and runs on the P4: the 1090 traffic chain is verified end-to-end on
silicon (CRC-clean GDL90, injected positions resolve correctly). The 978 weather chain is
algorithmically verified by host unit tests (Reed-Solomon FEC across all three UAT codes, FSK
sync/demod on synthesized waveforms, UAT→`adsb_msg_t`); on-air weather validation is pending a
second (978) dongle.

---

## Roadmap

- **978 MHz UAT + FIS-B weather** — *implemented.* A second auto-detected dongle adds UAT traffic
  (merged into the same table) and FIS-B weather relayed to ForeFlight as GDL90 Uplink (0x07).
  Pending on-air validation with a real 978 dongle.
- **WiFi GDL90** — *implemented.* The on-board ESP32-C6 SoftAP + UDP :4000 broadcast to tablets,
  same encoder as the wired path.
- **RS-232 / Garmin TIS-A** — panel-display output (e.g. G3X Touch); requires a reference position.
- **GPS / NMEA ownship** — live reference for relative formats and range/altitude filtering
  (hardware listed in the BOM; firmware support not yet implemented).

---

## License and attribution

Source is available under the [PolyForm Noncommercial License 1.0.0](LICENSE.md) — free for
noncommercial use. Commercial use requires a separate license from Novabox.Works.

ADSBin uses permissive sources only. It adapts the BSD-licensed antirez dump1090 (DSP, Mode-S CRC,
CPR decoder structure; notices preserved) and clean-rooms the RTL2832U / R820T2 tuner bring-up and
the GDL90 encoder from public datasheets and the Garmin GDL90 specification. No GPLv2 librtlsdr and
no GPLv3 dump1090 fork is used. All borrowed code is tracked in [THIRD_PARTY.md](THIRD_PARTY.md),
with full texts in [`licenses/`](licenses/).

**Safety:** ADSBin is a receive-only, advisory traffic-awareness aid for experimental, non-certified
aircraft. It is not certified, not a collision-avoidance system, and not a substitute for
see-and-avoid or pilot judgment. See [DISCLAIMER.md](DISCLAIMER.md).
