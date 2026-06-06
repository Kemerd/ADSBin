# sinks host unit test (gdl90_encoder)

Validates the **pure** GDL90 encoder (`../gdl90_encoder.c`) on a development
host — no ESP-IDF, no hardware. The encoder pulls in `adsbin_types.h`
(→ `esp_timer.h`) and `esp_err.h`; the two `stub_*.h` files here satisfy those
includes for a host compile only. They are **not** part of the firmware build
(the component CMakeLists lists only the five firmware `.c` files).

## Run

```sh
# from this directory
cp stub_esp_timer.h esp_timer.h
cp stub_esp_err.h   esp_err.h
gcc -std=c11 -Wall -Wextra -I. -I../include -I../../common/include \
    test_gdl90.c ../gdl90_encoder.c -lm -o test_gdl90
./test_gdl90
rm -f esp_timer.h esp_err.h        # don't leave shadowing copies around
```

## What it checks

- CRC-16 table fold == an independent table reference, **and** matches the
  published GDL90 spec heartbeat vector (`0x8BB3`).
- Heartbeat (0x00) frame is byte-for-byte identical to an independent framer.
- Traffic (0x14) frame: lat/lon semicircle packing, 12-bit pressure altitude,
  12-bit horizontal + signed vertical velocity, track, callsign padding.
- Byte-stuffing: no interior raw `0x7E`; every `0x7D` escape is well-formed.
- Buffer-overflow returns a negative `esp_err`.
