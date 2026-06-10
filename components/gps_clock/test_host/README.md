# gps_clock host unit test

Validates the **pure** NMEA-0183 parser (`../gps_clock_nmea.c`) on a development
host — no ESP-IDF, no hardware. The parser pulls in `adsbin_types.h`
(→ `esp_timer.h`); the `stub_esp_timer.h` here satisfies that include for a host
compile only. It is **not** part of the firmware build (the component CMakeLists
lists the real sources; this directory is test-only).

The Layer-3 PPS file (`../gps_clock_pps.c`) drives ESP-IDF GPTimer/ETM/PCNT
peripherals and is therefore exercised on-target, not here. This host test covers
the correctness-critical, hardware-independent parser.

## Run

```sh
# from this directory
cp stub_esp_timer.h esp_timer.h
gcc -std=c11 -Wall -Wextra -I. -I../include -I../../common/include \
    -I../../ownship/include test_gps_nmea.c -lm -o test_gps_nmea
./test_gps_nmea
rm -f esp_timer.h test_gps_nmea        # don't leave a shadowing copy around
```

(The test `#include`s `../gps_clock_nmea.c` directly so it can reach the parser's
file-static state; there is no separate object to link.)

## What it checks

- A good **GGA + RMC** pair (multi-constellation `GN` talker) decodes latitude,
  longitude, MSL altitude, ground speed, track, and an absolute UTC timestamp —
  with the two sentences **merged by time-of-day**, so a GGA arriving before its
  RMC still contributes its altitude (the common real-world ordering).
- A **bad XOR checksum** sentence is ignored (no fresh fix is published).
- The **null-island** `(0, 0)` position is rejected as invalid even with RMC
  status `A`.
- **Southern/Western** hemispheres produce correctly-signed coordinates.
- RMC status **`V`** (navigation receiver warning) yields an invalid fix.
- `byte_count` advances for every consumed byte (the module-presence signal the
  supervisor's ladder uses to leave the `NONE` state).

The parsed fix flows, unchanged, into the supervisor's clock ladder, which
publishes the position through `ownship_update(source = OWNSHIP_SOURCE_GPS)` once
it reaches the `NMEA_FIX` rung.
