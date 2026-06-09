# uat_decode host unit test

Validates the **pure** UAT message decoder (`../uat_decode.c`) on a development
host — no ESP-IDF, no hardware. The decoder pulls in `adsbin_types.h`
(→ `esp_timer.h`) and `esp_err.h`; the two `stub_*.h` files here satisfy those
includes for a host compile only. They are **not** part of the firmware build
(the component CMakeLists lists only `uat_decode.c`).

## Run

```sh
# from this directory
cp stub_esp_timer.h esp_timer.h
cp stub_esp_err.h   esp_err.h
gcc -std=c11 -Wall -Wextra -I. -I../include -I../../common/include \
    test_uat_decode.c ../uat_decode.c -lm -o test_uat_decode
./test_uat_decode
rm -f esp_timer.h esp_err.h        # don't leave shadowing copies around
```

## What it checks

- Long and short UAT ADS-B frames decode the 24-bit address, ABSOLUTE lat/lon
  (no CPR), and altitude (barometric and geometric) into the shared `adsb_msg_t`,
  tagged `downlink_format = 0` (UAT-origin).
- The all-zero lat/lon "position unavailable" sentinel yields `has_position = false`
  while still decoding the address.
- Bad payload length and NULL payload are rejected.
- FIS-B uplink: a 432-byte payload validates (safe to relay) and parses the
  position-valid flag; a wrong-length uplink is rejected.

The decode results flow, unchanged, onto the same `msg_queue → traffic_ingest`
path as 1090 traffic — so UAT traffic merges into the one traffic table.
