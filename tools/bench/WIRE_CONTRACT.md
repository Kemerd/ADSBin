# ADSBin USB-CDC Wire Contract (FROZEN)

This file is the **frozen ABI** between the firmware's USB-CDC output (the
`sink_debug` table and the `+INJECT` console) and the host-side Python bench
harness (`tools/bench/`). Both sides MUST conform byte-for-byte. Do not change a
token name or delimiter without bumping every consumer.

> Transport: the ESP-IDF console link on the P4's USB-C port (the same link that
> carries flashing and logs). On boards that break out the native USB-Serial/JTAG
> controller this is `303A:1001`; on boards whose single USB-C jack is wired
> through an on-board USB-UART bridge (e.g. a CH343) it is that bridge's COM port
> instead — point the bench at it with `--port COMx`. Either way the GDL90 byte
> stream and this text share the one link; the harness deframes GDL90 by its
> `0x7E` flags and reads debug lines as UTF-8 text.

---

## 1. `sink_debug` traffic-table format

Every publish cycle emits one block. Lines are UTF-8, `\n`-terminated.

```
=== ADSBIN TRAFFIC <count> @ <now_us> ===
ICAO=<6hex> [CS=<callsign>] [LAT=<deg>] [LON=<deg>] [ALT=<ft>] [GS=<kt>] [TRK=<deg>] [VR=<fpm>] [CAT=<n>] [NIC=<n>] [NACp=<n>] [RNG=<nm>] [BRG=<deg>] MSGS=<n> SEEN=<age_ms>
... one line per live target ...
=== END ===
```

### Token rules
- The parser keys on lines that **start with `ICAO=`**. The header/footer lines
  (`=== ... ===`) are optional context and may be ignored by the parser.
- Tokens are **space-separated `KEY=VALUE`**, order as listed above.
- **Optional tokens are OMITTED entirely** when the field is invalid (e.g. no
  `LAT=`/`LON=` until position is resolved). `ICAO=`, `MSGS=`, `SEEN=` are always
  present.
- Value formats:
  - `ICAO` : 6 uppercase hex digits, zero-padded (e.g. `A1B2C3`).
  - `CS`   : 1–8 chars, no spaces; emitted only if a callsign is known.
  - `LAT`/`LON` : signed decimal degrees, ≥4 fractional digits.
  - `ALT`/`VR`  : signed integer feet / feet-per-minute.
  - `GS`/`TRK`/`BRG` : unsigned integer knots / degrees.
  - `RNG`  : decimal nautical miles (emitted only when ownship is set).
  - `CAT`  : emitter category integer (matches `adsb_emitter_category_t`).
  - `NIC`/`NACp` : integers 0..11.
  - `SEEN` : milliseconds since the target was last heard.

### Verbose per-message line (only when `sink_debug.verbose`)
```
MSG ICAO=<6hex> DF=<n> TC=<n> RAW=<14hex|28hex>
```

---

## 2. `+INJECT` console command (deterministic decode testing)

Lets the bench feed canned ADS-B frames into the decoder over USB-CDC without
live air (à la dump5892 terminal injection, plan S3.1). Owned firmware-side by
the `main` debug console (see the parallel-build plan).

- **Request** (host → device), one per line:
  ```
  +INJECT <hex>
  ```
  `<hex>` is a raw Mode-S frame: 14 hex chars (56-bit/short) or 28 hex chars
  (112-bit/long), case-insensitive, no spaces.
- **Response** (device → host):
  ```
  +OK                  (frame accepted and pushed into the decode path)
  +ERR <reason>        (e.g. BADLEN, BADHEX)
  ```
- Injected frames flow through the *real* `modes_decode` path, so the resulting
  target appears in the next `sink_debug` block and GDL90 output.

---

## 3. GDL90 framing constants (must match `gdl90_encoder.h`)

| Constant | Value | Meaning |
|---|---|---|
| Flag byte | `0x7E` | Start/end of every frame |
| Escape byte | `0x7D` | Byte-stuffing escape |
| Escape XOR | `0x20` | Stuffed byte = original XOR 0x20 |
| Heartbeat id | `0x00` | Heartbeat message |
| Uplink id | `0x07` | Uplink Data (FIS-B weather; id + 3-byte big-endian Time of Reception + up to 432-byte UAT uplink payload) |
| Ownship id | `0x0A` | Ownship Report |
| Traffic id | `0x14` | Traffic Report |
| CRC | CRC-16 (CCITT table variant) | Over UNstuffed id+payload, little-endian on the wire |

### Deframe procedure (host)
1. Split the byte stream on `0x7E` flags.
2. Un-stuff: replace `0x7D <b>` with `<b> XOR 0x20`.
3. Last 2 bytes = CRC-16 (LE); verify against CRC of the preceding id+payload.
4. First byte = message id; parse payload per the GDL90 spec.

Lat/lon are packed as 24-bit two's-complement semicircles
(`value = round(deg * 2^23 / 180)`); pressure altitude is the 12-bit
`(alt_ft + 1000) / 25` encoding (`0xFFF` = invalid).
