# demod1090 host unit test (preamble + PPM slicer)

Validates the **pure DSP core** of `../demod1090.c` on a development host — no
ESP-IDF, no hardware — compiled with `-DDEMOD1090_HOST_TEST` so its FreeRTOS
task shell is excluded (same arrangement as `demod978`'s host tests).

This is the stage the bench `+INJECT` path **bypasses** (injection enters at
`modes_decode`), so until this test existed the preamble detector and bit
slicer had never been exercised by anything: the synthesized bursts here sweep
the full range of sub-sample arrival phases at 2.4 Msps, with band-limited
pulse edges and a realistic noise floor.

The stub headers here satisfy `adsbin_types.h` / `demod1090.h` includes for a
host compile; they are **not** part of the firmware build (the component
CMakeLists lists only `demod1090.c`).

## Run

```sh
# from this directory
cp stub_esp_timer.h esp_timer.h
cp stub_esp_err.h   esp_err.h

gcc -std=c11 -Wall -Wextra -O2 -DDEMOD1090_HOST_TEST \
    -I. -I../include -I.. -I../../common/include \
    test_preamble.c ../demod1090.c -lm -o test_preamble
./test_preamble

rm -f esp_timer.h esp_err.h test_preamble   # don't leave shadows around
```

## What it checks

- **Phase sweep (the regression test):** a DF17 burst must decode bit-exact at
  ≥97% of 240 uniformly-spread sub-sample arrival phases. A single-template
  point-sampling detector fails a large slice of phases outright — this is the
  in-flight "strong traffic overhead, nothing decodes" failure mode.
- **Short frames:** DF11 (56-bit) across the same sweep.
- **Weak signal:** graceful degradation (≥60% at ~14 dB SNR), no cliff.
- **Pure noise:** with a hot front end (49.6 dB gain noise floor) the pre-gate +
  correlator must keep the false-candidate rate bounded — this bounds the
  garbage load on Core 0/Core 1 (CRC kills the survivors downstream).
- **Two bursts per block:** the post-frame advance logic must not swallow a
  following burst.

The synthesizer applies a one-pole envelope filter (~1-sample rise time) so
pulse edges look like the RTL2832U's band-limited output, and a random carrier
phase per burst so envelope detection is genuinely I/Q-agnostic.
