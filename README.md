<div align="center">

# ✈️ ADSBin

### A small, cheap, receive-only ADS‑B box for general aviation.

**It listens to the planes around you and tells your tablet where they are.**
No subscription. No transmitter. No nonsense.

`ESP32‑P4` · `RTL‑SDR` · `1090 MHz / 1090ES` · `GDL90 out` · `~$55–80 in parts`

</div>

---

> **ADSBin = "ADS‑B *In*."** In aviation, *ADS‑B In* means **receiving** traffic — as opposed to
> *ADS‑B Out*, which **transmits** your position. This box is electrically incapable of
> transmitting. That's the whole point: it's the part that keeps you aware, not the part that
> talks to ATC. Receive‑only is what keeps it legal, cheap, and liability‑light.

A Novabox product, sibling to LidarAGL. Built for **experimental / non‑certified** aircraft and
the bench. **Not** a collision‑avoidance system — see [the disclaimer](DISCLAIMER.md).

---

## 🧭 Table of contents

- [Why ADSBin](#-why-adsbin)
- [How it works](#-how-it-works)
- [What you need (hardware + cost)](#-what-you-need)
- [Wiring it up](#-wiring-it-up)
- [Software setup](#-software-setup)
- [Build & flash](#-build--flash)
- [Using it](#-using-it)
- [Testing without a radio (the bench harness)](#-testing-without-a-radio)
- [Configuration](#-configuration)
- [Project layout](#-project-layout)
- [Status](#-status)
- [Roadmap](#-roadmap)
- [License & attribution](#-license--attribution)

---

## 💡 Why ADSBin

Commercial ADS‑B receivers are either expensive, locked behind app subscriptions, or both. ADSBin
is the opposite of that:

- **🔓 No software feature‑locks.** You get every decode feature the hardware can do. Tiering is by
  *what you plug in*, never by a paywall. (The decode core is derived from open ADS‑B work — locking
  it would be both wrong and legally impossible.)
- **🧩 One binary, many boxes.** The firmware **auto‑detects** how many dongles and which bands are
  present and lights up features accordingly. Build the superset, sell the subset. Same image on the
  glareshield today, in the panel tomorrow.
- **🛰️ No GPS required.** ADS‑B targets broadcast their *own* absolute position; ADSBin resolves it
  with global CPR (even/odd pairing) and hands your tablet absolute lat/lon. Your EFB already knows
  where *you* are. (Ownship is only needed for relative formats — that's a later phase.)
- **⚡ Genuinely real‑time.** A dual‑core RISC‑V design where Core 0 does nothing but ingest IQ and
  run DSP, and Core 1 does everything else. 2.4 Msps of 8‑bit I/Q (~4.8 MB/s) streams over USB
  High‑Speed — a rate a smaller ESP32 physically cannot carry. This is *why* the P4 exists here.
- **🪶 Cheap and hackable.** ~$55–80 in parts, a plastic box, a wire antenna, and a USB‑C cable.

It's small, it's fast, and it does exactly one job extremely well.

---

## ⚙️ How it works

ADSBin pins the hard‑real‑time radio path to **Core 0** and everything else to **Core 1**, so the
~4.8 MB/s sample firehose never competes with decoding or output.

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
                 │                   │  table      │                                    │
                 │                   └──────┬──────┘                                    │
                 │                          │ snapshot                                  │
                 │              ┌───────────┼───────────┐                               │
                 │              ▼           ▼           ▼                               │
                 │        [sink_debug] [sink_gdl90]  [future: WiFi / TIS]              │
                 │         USB-CDC      USB-CDC                                          │
                 └────────────────────────────────────────────────────────────────────┘
```

The pipeline, stage by stage:

| Stage | Component | Core | What it does |
|---|---|---|---|
| 1 | **usb_rtlsdr** | 0 | Brings up the USB‑HS host, tunes the R820T2 to 1090 MHz @ 2.4 Msps, streams raw I/Q into a lock‑free ring it owns. |
| 2 | **demod1090** | 0 | Magnitude LUT → 8 µs preamble correlation → PPM bit‑slice → 56/112‑bit candidate frames. Never blocks; drops‑with‑counter under load. |
| 3 | **modes_decode** | 1 | 24‑bit Mode‑S CRC, DF17/18 parse, ICAO, callsign, altitude, velocity, and **CPR position resolution** (global even/odd + local). Emits absolute lat/lon. |
| 4 | **traffic** | 1 | One record per aircraft keyed by ICAO. Merges updates, ages out stale targets, optional range/altitude/sanity culling. |
| 5 | **sinks** | 1 | Pluggable outputs: a human‑readable **debug table** and a spec‑correct **GDL90** stream (Heartbeat + Traffic Report), both over USB‑CDC for the MVP. Same encoder later drives WiFi. |

Supporting cast: **config** (NVS settings), **ownship** (optional reference position), **status**
(LEDs + die‑temperature watchdog), **common** (the shared type contract every component compiles
against). Full design rationale lives in [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).

---

## 🛒 What you need

The MVP is one compute board, one dongle, one antenna, and a cable.

| Part | What | ~Cost |
|---|---|---|
| **Compute** | ESP32‑P4 dev board (ideally with an on‑board ESP32‑C6 for future WiFi). Dual‑core RISC‑V ~400 MHz, USB 2.0 **High‑Speed** OTG host. | ~$30–40 |
| **Receiver** | Nooelec **NESDR Nano 3** (RTL2832U + **R820T2**). A generic R820T2 dongle works too. | ~$30–45 (or ~$15–20 generic) |
| **Antenna** | 1090 MHz λ/4 whip (**~68 mm**) on SMA + a small ground plane/counterpoise. DIY is totally fine. | ~$2–3 |
| **Power** | USB‑C, 5 V. Budget ≥ **1.5 A** headroom (P4 + dongle + future C6). | — |
| **Enclosure** | Small **plastic/composite** box (RF‑transparent — metal blocks an internal antenna). | ~$10 |
| **Status** | 1–2 LEDs (power + "hearing traffic" heartbeat). | — |
| | **MVP total** | **≈ $55–80** |

> 🔥 **Thermals:** one dongle + 1090‑only is almost certainly **passive‑coolable** — likely **no
> fan**. Keep a thermal pad to the enclosure wall and let the firmware's temp watchdog log
> worst‑case temps during field testing before you commit.

---

## 🔌 Wiring it up

The P4 has **two** USB interfaces. Don't conflate them:

| P4 port | Used for |
|---|---|
| **USB‑C** | **Power + debug.** Flashing, logs, and the MVP GDL90/debug output all ride this one cable (USB‑Serial/JTAG). |
| **USB‑HS OTG host** | **The dongle.** A separate interface dedicated to the RTL‑SDR. |

Wire the NESDR Nano's USB‑A plug **directly** to the P4's HS host port:

```
NESDR Nano (RTL2832U)            ESP32-P4 (USB-HS host)
  D+  ───────────────────────────►  HS host D+   (keep the pair short, matched, away from noise)
  D-  ───────────────────────────►  HS host D-
  VBUS (5V) ◄─────────────────────  5V rail      (host mode: the P4 powers the dongle, ~300–500 mA)
  GND ───────────────────────────►  GND          (common ground)

  SMA ── 1090 MHz λ/4 whip (~68 mm) + ground plane
```

Then:

- **Antenna placement matters.** Keep the element away from the P4, switching regulators, and USB —
  ADS‑B is weak and those raise your noise floor. Short coax (1090 MHz is lossy). Plastic box for an
  internal antenna; metal box ⇒ external antenna.
- **Confirm your 5 V rail** can source the dongle's ~300–500 mA *on top of* the P4 itself, and add a
  bulk cap near the dongle's VBUS tap to survive inrush.
- **Status LEDs** are board‑specific — you pick the GPIOs in config (see [Configuration](#-configuration)).

Full RF/placement notes are in [IMPLEMENTATION_PLAN.md §8](IMPLEMENTATION_PLAN.md).

---

## 🧰 Software setup

ADSBin builds with **ESP‑IDF v6.0.1** (RISC‑V GCC 15.2) targeting `esp32p4`.

1. **Install ESP‑IDF v6.0.1** (the standard Espressif installer or the VS Code extension). Confirm
   the toolchain + Python venv are actually installed (`idf.py --version` works in an ESP‑IDF
   terminal).
2. **Open an ESP‑IDF terminal** so `idf.py` is on `PATH`. On Windows, that's the "ESP‑IDF
   PowerShell/CMD" shortcut, or source the export script:
   ```powershell
   . C:\esp\v6.0.1\esp-idf\export.ps1
   ```
3. **Set the target once** (regenerates `sdkconfig` from `sdkconfig.defaults`):
   ```
   tools\set-target.bat          # == idf.py set-target esp32p4
   ```

The USB Host Library left ESP‑IDF core in v6.0 — ADSBin pulls it back in automatically as the
managed component `espressif/usb` (declared in `components/usb_rtlsdr/idf_component.yml`). The
component manager fetches it on first build; just be online for that build.

> **Flash size** defaults to **16 MB** (typical P4 dev board). Different board? Change the one
> `CONFIG_ESPTOOLPY_FLASHSIZE_*` line in `sdkconfig.defaults`. The partition layout (dual 3 MB OTA
> app slots) fits in 8 MB too.

---

## 🚀 Build & flash

One‑click `.bat` helpers live in [`tools/`](tools/) (run them yourself from an ESP‑IDF terminal):

| Command | Does |
|---|---|
| `tools\set-target.bat` | One‑time: set chip to ESP32‑P4. |
| `tools\build.bat` | Build only (fast compile check). |
| `tools\bfm.bat [COMx]` | **Build → flash → monitor** in one shot (the main loop). Omit `COMx` to auto‑detect. |
| `tools\flash.bat [COMx]` | Flash + monitor (no rebuild). |
| `tools\monitor.bat [COMx]` | Serial monitor only (`Ctrl‑]` to exit). |

Typical first run:

```
tools\set-target.bat
tools\bfm.bat COM5          # ← your P4's USB-C serial port
```

You should see the boot banner (chip + cores + IDF version), the two cores report in, and — with no
dongle yet — a 1 Hz "alive" heartbeat. Plug in the NESDR and traffic starts flowing.

---

## 📡 Using it

Once flashed and connected to an antenna with traffic overhead, ADSBin emits **two streams over the
USB‑C serial link**:

1. **A human‑readable traffic table** (`sink_debug`) — one line per aircraft:
   ```
   === ADSBIN TRAFFIC 3 @ 19h... ===
   ICAO=A1B2C3 CS=UAL123 LAT=37.6189 LON=-122.3750 ALT=12000 GS=320 TRK=095 VR=-640 MSGS=42 SEEN=120
   ...
   === END ===
   ```
2. **A GDL90 byte stream** (`sink_gdl90`) — Heartbeat + Traffic Report frames an EFB understands.

In the MVP both ride **USB‑CDC** so you can validate everything from a PC before any WiFi exists. The
GDL90 encoder is transport‑agnostic — the *same* encoder later broadcasts over WiFi to ForeFlight /
Garmin Pilot, changing only the transport.

The exact wire format (debug tokens, GDL90 constants, the `+INJECT` test command) is frozen in
[`tools/bench/WIRE_CONTRACT.md`](tools/bench/WIRE_CONTRACT.md).

---

## 🧪 Testing without a radio

You don't need the sky (or even a dongle) to prove the decode/output is correct. The Python bench
harness in [`tools/bench/`](tools/bench/) talks to the box over USB‑CDC and can inject canned
frames.

```
cd tools\bench
python -m pip install -r requirements.txt        # just pyserial

python adsbin_bench.py list-ports                 # find the P4's serial port
python adsbin_bench.py validate-gdl90 --port COM5 # parse & CRC-check live GDL90 frames
python adsbin_bench.py dump-debug --port COM5     # pretty-print the live traffic table
python adsbin_bench.py inject --port COM5 <hex>   # feed one raw Mode-S frame into the decoder
python adsbin_bench.py inject-verify --port COM5  # inject the canned corpus, assert the decode
python adsbin_bench.py list-canned                # show the built-in test frames + ground truth
```

`inject-verify` is the **acceptance gate**: it pushes known ADS‑B frames (including matched CPR
even/odd pairs with ground‑truth positions) through the *real* firmware decode path and checks the
output. Deterministic, repeatable, no airplanes required.

**Pure host unit tests** (CPR math, GDL90 CRC/framing, debug‑table round‑trip) need no hardware at
all:

```
cd tools\bench\tests
run_tests.bat          # 44 tests: all green
```

---

## 🔧 Configuration

Settings persist in **NVS** ([`components/config`](components/config/)): tuner gain, an optional
manual reference position, range/altitude filters, the active band map, and which output sinks are
enabled. Defaults are sane out of the box (49.6 dB fixed gain / AGC off, debug + GDL90 enabled).

Build‑time options via `idf.py menuconfig` → **ADSBin**:

- **`ADSBIN_STATUS_LED_GPIO`** — GPIO for the heartbeat LED. Defaults to `-1` (disabled) because the
  LED pin differs per board; set it to your board's user LED. Console heartbeat works regardless.
- **`ADSBIN_STATUS_LED_ACTIVE_LOW`** — flip if your LED is active‑low.

> The dedicated `status` component drives a power LED + a traffic‑heartbeat LED and samples the P4's
> internal die temperature. Its default pins are placeholders — set your real board pinout before a
> field build.

---

## 🗂️ Project layout

```
ADSBin/
├─ README.md                  ← you are here
├─ IMPLEMENTATION_PLAN.md     full design + rationale
├─ LICENSE.md                 PolyForm Noncommercial 1.0.0
├─ DISCLAIMER.md              receive-only / experimental / not collision-avoidance
├─ THIRD_PARTY.md             provenance + licenses of every borrowed line
├─ licenses/                  verbatim upstream license texts
├─ tools/                     .bat deploy scripts + Python bench harness
│  └─ bench/                  USB-CDC validator, GDL90 decoder, +INJECT, canned corpus, tests/
├─ sdkconfig.defaults         P4 target, dual-core, USB-Serial/JTAG console
├─ partitions.csv             dual-OTA app slots (3 MB each)
├─ main/                      app entry, pipeline wiring, core pinning, +INJECT console
└─ components/
   ├─ common/                 the frozen shared-type ABI (adsbin_types.h)
   ├─ usb_rtlsdr/             RTL-SDR USB-HS host driver        (Core 0)
   ├─ demod1090/              1090ES demodulator                (Core 0)
   ├─ modes_decode/           Mode-S/ADS-B parser + CPR         (Core 1)
   ├─ traffic/                traffic table manager             (Core 1)
   ├─ sinks/                  debug + GDL90 outputs             (Core 1)
   ├─ ownship/                reference position (optional)
   ├─ config/                 NVS-backed settings
   └─ status/                 LEDs + temperature watchdog
```

---

## ✅ Status

| Thing | State |
|---|---|
| Firmware build (ESP‑IDF v6.0.1 / esp32p4) | ✅ **Builds clean.** `adsbin.bin` ≈ 333 KB, fits the 3 MB app slot with **89 % free**. |
| Host unit tests (CPR / GDL90 / debug format) | ✅ **44 / 44 passing.** |
| Full pipeline wired (USB → DSP → decode → traffic → sinks) | ✅ Implemented + linked. |
| On‑hardware bring‑up (real dongle, live traffic) | ⏳ **Not yet flown** — needs the P4 + NESDR flashed. |
| GDL90 integrity fields (NIC/NACp) + raw‑frame debug (`RAW=`) | ⏳ Known gaps; don't block the build, polish pending. |

In short: **it compiles, links, fits, and passes every host‑side test.** Real‑silicon validation
(and the §6 phase gates — frame yield vs. dump1090, target counts vs. a reference receiver) is the
next milestone.

---

## 🛣️ Roadmap

Designed‑for from day one, built incrementally:

- **978 MHz UAT** — a second dongle, auto‑detected, adds free US weather (FIS‑B) + UAT traffic.
- **WiFi GDL90 → EFB tablets** — bring up the on‑board ESP32‑C6 and UDP‑broadcast GDL90 to
  ForeFlight / Garmin Pilot. Reuses the existing encoder; only the transport changes.
- **RS‑232 + Garmin TIS‑A** — drive a panel display (e.g. G3X Touch). Relative format → needs ownship.
- **GPS / NMEA ownship** — live reference for relative formats + range/altitude filtering.

---

## ⚖️ License & attribution

ADSBin is offered under the **[PolyForm Noncommercial License 1.0.0](LICENSE.md)** — free for any
noncommercial use (hobby, study, non‑profit, public‑safety). Commercial use? Ask Novabox.Works.

To keep that license clean, ADSBin deliberately uses **permissive sources only**: it adapts the
**BSD‑licensed antirez dump1090** (DSP, Mode‑S CRC, CPR structure — notices preserved) and
**clean‑rooms** the RTL2832U/R820T2 tuner bring‑up and the GDL90 encoder from public datasheets and
the Garmin spec. **No GPLv2 librtlsdr and no GPLv3 dump1090 fork is used.** Every borrowed line is
tracked in [THIRD_PARTY.md](THIRD_PARTY.md), with full texts in [`licenses/`](licenses/).

> ⚠️ **Safety:** ADSBin is a **receive‑only, advisory** traffic‑awareness aid for **experimental,
> non‑certified** aircraft. It is **not** certified, **not** a collision‑avoidance system, and
> **not** a substitute for see‑and‑avoid or pilot judgment. Read [DISCLAIMER.md](DISCLAIMER.md).
> Use at your own risk.

---

<div align="center">

**Built by [Novabox.Works](https://novabox.works/).** Receive‑only. Experimental. Yours to hack.

*Hear everything. Transmit nothing.*

</div>
