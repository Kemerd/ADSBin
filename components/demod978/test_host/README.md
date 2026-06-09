# demod978 host unit tests (UAT FEC + FSK demod)

Validate the **pure** parts of the 978 MHz UAT path on a development host — no
ESP-IDF, no hardware:

- `../uat_fec.c` — the clean-room Reed-Solomon FEC over GF(256).
- the pure DSP core of `../demod978.c` (discriminator + 36-bit sync + bit slicing
  + FEC dispatch), compiled with `-DUAT_HOST_TEST` so its FreeRTOS task shell is
  excluded.

The single `stub_esp_timer.h` here satisfies `adsbin_types.h`'s `esp_timer.h`
include for a host compile; it is **not** part of the firmware build (the
component CMakeLists lists only `demod978.c` + `uat_fec.c`).

## Run

```sh
# from this directory
cp stub_esp_timer.h esp_timer.h

# 1) Reed-Solomon FEC: RS(30,18) / RS(48,34) / RS(92,72) + uplink interleave.
gcc -std=c11 -Wall -Wextra -I../include test_uat_fec.c ../uat_fec.c -o test_uat_fec
./test_uat_fec

# 2) FSK demod core end-to-end on a synthesized 2-FSK waveform.
gcc -std=c11 -Wall -Wextra -DUAT_HOST_TEST \
    -I. -I../include -I.. -I../../common/include \
    test_uat_sync.c ../demod978.c ../uat_fec.c -lm -o test_uat_sync
./test_uat_sync

rm -f esp_timer.h test_uat_fec test_uat_sync   # don't leave shadows around
```

## What it checks

`test_uat_fec.c` (the highest-risk algorithm — a wrong constant = silent
no-weather):
- GF(256) field built from the UAT primitive polynomial `0x187`.
- For each code (RS(30,18) basic, RS(48,34) long, RS(92,72) uplink), 2000+
  randomized trials: exact recovery of the original message at 0..t byte errors,
  and the reported corrected-count equals the injected count.
- No FALSE successes at t+1 errors (the decoder fails or genuinely restores —
  never silently returns the wrong message as if correct).
- Full uplink frame: 6 RS(92,72) codewords byte-interleaved, corrupted within t,
  deinterleaved + decoded, asserting the 432-byte payload is restored exactly;
  and an uncorrectable block is flagged, not silently passed.

`test_uat_sync.c` (the demod, end-to-end with FEC):
- Synthesizes a 2-FSK I/Q waveform at 2.4 Msps for a known sync word + RS-encoded
  payload, then runs the production discriminator + `uat_core_process`.
- Basic ADS-B (18 B), Long ADS-B (34 B), and Uplink (432 B) payloads each recover
  byte-for-byte through discriminator → 36-bit sync → fractional-sample bit
  timing → RS FEC.
- Pure noise produces no false frames.
