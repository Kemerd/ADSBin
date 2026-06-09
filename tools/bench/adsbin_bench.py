# -*- coding: utf-8 -*-
"""
adsbin_bench.py — Host-PC bench harness CLI for the ADSBin ESP32-P4 receiver.

One command-line entry point that ties the bench modules together against the
frozen USB-CDC wire contract (``tools/bench/WIRE_CONTRACT.md``) and the frozen
``gdl90_encoder.h`` / ``cpr.h`` field layouts.

Subcommands
-----------
  list-ports     Enumerate serial ports and flag likely ADSBin devices.
  validate-gdl90 Capture the live GDL90 byte stream and CRC-check every frame.
  dump-debug     Capture and pretty-print the sink_debug traffic table.
  inject         Push one raw Mode-S frame (hex or a named canned frame).
  inject-verify  Inject a frame/pair, then verify the device's decode against
                 the corpus ground truth (sink_debug and/or GDL90).

Everything routes through cdc_link.CdcLink for I/O, gdl90.py for the independent
reference decode, sink_debug_parse.py for the text grammar, inject.py for the
+INJECT handshake, and canned_msgs.py for verified test vectors.

Run ``python adsbin_bench.py <subcommand> --help`` for per-command options.
UTF-8 / Windows-console safe (stdout is reconfigured to UTF-8 at startup).
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Tuple

import gdl90
import canned_msgs
import sink_debug_parse
from inject import inject_frame, normalize_frame_hex
import cdc_link


# Position match tolerance (degrees). ~0.0005° ≈ 55 m, comfortably tighter than
# the device's own CPR quantisation yet loose enough to absorb float rounding.
_POS_TOL_DEG = 5e-4


# ─────────────────────────────────────────────────────────────────────────────
#  Small console helpers — keep output clean and Windows-safe.
# ─────────────────────────────────────────────────────────────────────────────
def _setup_console() -> None:
    """Force UTF-8 on stdout/stderr so box glyphs never mojibake on Windows."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8")
        except Exception:
            # Pre-3.7 or a redirected stream that can't reconfigure — ignore.
            pass


def _ok(msg: str) -> None:
    """Print a success line."""
    print(f"[ OK ] {msg}")


def _fail(msg: str) -> None:
    """Print a failure line to stderr."""
    print(f"[FAIL] {msg}", file=sys.stderr)


def _info(msg: str) -> None:
    """Print an informational line."""
    print(f"[INFO] {msg}")


# ═════════════════════════════════════════════════════════════════════════════
#  Subcommand: list-ports
# ═════════════════════════════════════════════════════════════════════════════
def cmd_list_ports(args: argparse.Namespace) -> int:
    """Enumerate serial ports, flagging probable ADSBin devices by VID:PID."""
    ports = cdc_link.list_ports()

    # Nothing plugged in at all — say so plainly rather than printing an empty list.
    if not ports:
        _info("No serial ports found.")
        return 0

    # Tabular listing; the [ADSBin] tag marks an auto-detectable device.
    print(f"{'PORT':<14}{'VID:PID':<12}{'SERIAL':<20}DESCRIPTION")
    for p in ports:
        vidpid = (f"{p.vid:04X}:{p.pid:04X}"
                  if p.vid is not None else "----:----")
        tag = "  [ADSBin]" if p.is_adsbin else ""
        serial = p.serial_number or "-"
        print(f"{p.device:<14}{vidpid:<12}{serial:<20}{p.description}{tag}")

    # Report the auto-detect verdict so the user knows what an option-less run does.
    auto = cdc_link.auto_detect_port()
    if auto:
        _info(f"Auto-detect would use: {auto}")
    else:
        _info("Auto-detect is ambiguous; pass --port explicitly.")
    return 0


# ═════════════════════════════════════════════════════════════════════════════
#  Subcommand: validate-gdl90
# ═════════════════════════════════════════════════════════════════════════════
def cmd_validate_gdl90(args: argparse.Namespace) -> int:
    """
    Capture the live GDL90 stream for a window and CRC-check every frame.

    Splits the captured bytes on 0x7E flags, un-stuffs, recomputes the CRC-16,
    and tallies pass/fail per message id. A non-zero exit means at least one
    frame failed CRC (or none were seen), which is the bench's go/no-go signal.
    """
    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    with link:
        _info(f"Capturing GDL90 for {args.duration:.1f}s ...")
        link.drain()
        stream = link.collect_bytes(args.duration)

    # Deframe the whole capture in one pass; partial frames are skipped cleanly.
    frames = gdl90.deframe_stream(stream)
    if not frames:
        _fail(f"No GDL90 frames seen in {len(stream)} bytes. Is the device "
              f"streaming on this port?")
        return 1

    # Per-id counters so the summary shows the message mix, not just a total.
    seen: dict = {}
    bad = 0
    for f in frames:
        name = _gdl90_id_name(f.msg_id)
        slot = seen.setdefault(name, [0, 0])   # [total, crc_fail]
        slot[0] += 1
        if not f.crc_ok:
            slot[1] += 1
            bad += 1

        # On request, decode and print each frame's headline fields.
        if args.show:
            _print_frame(f)

    # Summary table by message id.
    print("\nGDL90 frame summary:")
    for name in sorted(seen):
        total, fails = seen[name]
        status = "OK" if fails == 0 else f"{fails} CRC FAIL"
        print(f"  {name:<12} {total:>5} frames   {status}")

    print(f"\nTotal: {len(frames)} frames, {bad} CRC failures, "
          f"{len(stream)} bytes captured.")

    if bad:
        _fail(f"{bad} frame(s) failed CRC — device/host CRC mismatch or line "
              f"corruption.")
        return 1

    _ok("All GDL90 frames passed CRC.")
    return 0


def _gdl90_id_name(msg_id: int) -> str:
    """Human label for a GDL90 message id."""
    return {
        gdl90.GDL90_ID_HEARTBEAT: "Heartbeat",
        gdl90.GDL90_ID_OWNSHIP:   "Ownship",
        gdl90.GDL90_ID_TRAFFIC:   "Traffic",
    }.get(msg_id, f"id=0x{msg_id:02X}")


def _print_frame(frame: gdl90.Gdl90Frame) -> None:
    """Decode and print one GDL90 frame's headline fields."""
    crc = "crc-OK" if frame.crc_ok else "CRC-FAIL"
    parsed = gdl90.parse_frame(frame)

    # Heartbeat: timestamp + counts.
    if isinstance(parsed, gdl90.Heartbeat):
        print(f"  Heartbeat   {crc}  ts={parsed.timestamp_s}s "
              f"gps={parsed.gps_pos_valid} "
              f"basic/long={parsed.msg_count_basic_long}")
    # Traffic / Ownship: the bits an operator wants at a glance.
    elif isinstance(parsed, gdl90.TrafficReport):
        kind = "Ownship " if parsed.is_ownship else "Traffic "
        alt = "----" if parsed.alt_press_ft is None else f"{parsed.alt_press_ft}ft"
        print(f"  {kind}    {crc}  ICAO={parsed.icao:06X} "
              f"lat={parsed.lat_deg:.5f} lon={parsed.lon_deg:.5f} {alt} "
              f"gs={parsed.h_velocity_kt}kt cs={parsed.callsign!r}")
    else:
        print(f"  {_gdl90_id_name(frame.msg_id):<12}{crc}  (not decoded)")


# ═════════════════════════════════════════════════════════════════════════════
#  Subcommand: dump-debug
# ═════════════════════════════════════════════════════════════════════════════
def cmd_dump_debug(args: argparse.Namespace) -> int:
    """
    Capture the sink_debug text table and print the parsed targets.

    Reads newline-delimited lines over the window, parses them per the frozen
    grammar, and prints each target's fields. Useful for eyeballing live decode
    and confirming the firmware emits the contract token format.
    """
    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    with link:
        _info(f"Capturing sink_debug for {args.duration:.1f}s ...")
        link.drain()
        lines = link.read_lines(args.duration)

    # Optionally echo the raw text exactly as received for low-level debugging.
    if args.raw:
        print("--- raw lines ---")
        for ln in lines:
            print(ln)
        print("--- end raw ---\n")

    # Parse into blocks and report the freshest one (or all with --all).
    blocks = sink_debug_parse.parse_lines(lines)
    if not blocks:
        _fail("No sink_debug content parsed. Is the debug sink enabled and "
              "publishing on this port?")
        return 1

    to_show = blocks if args.all else blocks[-1:]
    for block in to_show:
        _print_debug_block(block)

    total_targets = sum(len(b.targets) for b in blocks)
    _ok(f"Parsed {len(blocks)} block(s), {total_targets} target line(s).")
    return 0


def _print_debug_block(block: sink_debug_parse.DebugBlock) -> None:
    """Pretty-print one parsed sink_debug block."""
    # Header context, when the device sent it.
    if block.count is not None:
        print(f"=== TRAFFIC count={block.count} @ {block.now_us}us ===")

    # One readable line per target, omitting fields the device omitted.
    for t in block.targets:
        parts = [f"ICAO={t.icao}"]
        if t.callsign is not None:
            parts.append(f"CS={t.callsign}")
        if t.lat_deg is not None and t.lon_deg is not None:
            parts.append(f"POS={t.lat_deg:.5f},{t.lon_deg:.5f}")
        if t.alt_ft is not None:
            parts.append(f"ALT={t.alt_ft}ft")
        if t.gs_kt is not None:
            parts.append(f"GS={t.gs_kt}kt")
        if t.trk_deg is not None:
            parts.append(f"TRK={t.trk_deg}")
        if t.vr_fpm is not None:
            parts.append(f"VR={t.vr_fpm}")
        if t.rng_nm is not None:
            parts.append(f"RNG={t.rng_nm:.1f}nm")
        parts.append(f"MSGS={t.msgs}")
        parts.append(f"SEEN={t.seen_ms}ms")
        print("  " + " ".join(parts))

    # Verbose per-message lines, if the sink emitted them.
    for m in block.msgs:
        print(f"  MSG ICAO={m.icao} DF={m.df} TC={m.tc} RAW={m.raw_hex}")


# ═════════════════════════════════════════════════════════════════════════════
#  Subcommand: inject
# ═════════════════════════════════════════════════════════════════════════════
def cmd_inject(args: argparse.Namespace) -> int:
    """
    Push one raw Mode-S frame into the decoder via +INJECT.

    The frame is either an explicit ``--hex`` value or a corpus entry named with
    ``--name``. We print the device's +OK / +ERR reply verbatim.
    """
    frame_hex = _resolve_frame_hex(args)
    if frame_hex is None:
        return 2

    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    with link:
        result = inject_frame(link, frame_hex, timeout=args.timeout)

    # Report the handshake outcome.
    if result.ok:
        _ok(f"+INJECT {frame_hex} -> {result.raw_reply}")
        return 0

    _fail(f"+INJECT {frame_hex} -> {result.raw_reply or result.reason}")
    return 1


def _resolve_frame_hex(args: argparse.Namespace) -> Optional[str]:
    """
    Resolve the frame hex from --hex / --name, validating it host-side.

    @return  Validated uppercase hex, or None after printing a diagnostic.
    """
    # Named corpus frame takes precedence when given.
    if getattr(args, "name", None):
        frame = canned_msgs.get_frame(args.name)
        if frame is None:
            _fail(f"Unknown canned frame {args.name!r}. Known: "
                  f"{', '.join(canned_msgs.all_frame_names())}")
            return None
        return frame.hex

    # Otherwise an explicit hex string; validate the same way the device will.
    if getattr(args, "hex", None):
        try:
            return normalize_frame_hex(args.hex)
        except ValueError as e:
            _fail(f"Bad frame hex: {e} (need 14 or 28 hex chars).")
            return None

    _fail("Provide a frame with --hex <hex> or --name <canned>.")
    return None


# ═════════════════════════════════════════════════════════════════════════════
#  Subcommand: inject-verify
# ═════════════════════════════════════════════════════════════════════════════
def cmd_inject_verify(args: argparse.Namespace) -> int:
    """
    Inject a frame or CPR pair, then verify the device's decode vs ground truth.

    For a single frame: inject it and check the next sink_debug block's target
    for the expected callsign / altitude / velocity. For a CPR pair: inject even
    then odd and check the resolved position against either anchoring within
    tolerance. A non-zero exit means a mismatch (or no target appeared).
    """
    # A pair name and a frame are mutually exclusive selection modes.
    if args.pair:
        return _verify_pair(args)
    return _verify_single(args)


def _verify_single(args: argparse.Namespace) -> int:
    """Inject one named/explicit frame and verify its decoded fields."""
    # Resolve the frame; for verification we strongly prefer a named corpus entry
    # because that's what carries ground truth.
    frame = None
    if args.name:
        frame = canned_msgs.get_frame(args.name)
        if frame is None:
            _fail(f"Unknown canned frame {args.name!r}. Known: "
                  f"{', '.join(canned_msgs.all_frame_names())}")
            return 2
        frame_hex = frame.hex
    else:
        frame_hex = _resolve_frame_hex(args)
        if frame_hex is None:
            return 2

    # Without ground truth there is nothing to verify against.
    if frame is None:
        _fail("inject-verify needs a canned --name (ground truth) or use --pair.")
        return 2

    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    with link:
        # Inject the frame and confirm the device accepted it.
        link.drain()
        res = inject_frame(link, frame_hex, timeout=args.timeout)
        if not res.ok:
            _fail(f"Device rejected +INJECT: {res.raw_reply or res.reason}")
            return 1
        _ok(f"Injected {frame.name}: {res.raw_reply}")

        # Capture the next debug block(s) and find our aircraft.
        lines = link.read_lines(args.duration)

    blocks = sink_debug_parse.parse_lines(lines)
    icao = frame.truth.icao
    if icao is None:
        _info("Frame carries no ICAO (non-ADS-B); accepted by device, nothing "
              "further to verify.")
        return 0

    tgt = sink_debug_parse.find_target(blocks, icao)
    if tgt is None:
        _fail(f"ICAO {icao:06X} never appeared in sink_debug after injection.")
        return 1

    # Compare each ground-truth field the frame populates.
    ok = _compare_target(frame, tgt)
    if ok:
        _ok(f"Decode of {frame.name} matches ground truth.")
        return 0
    return 1


def _verify_pair(args: argparse.Namespace) -> int:
    """Inject an even+odd CPR pair and verify the resolved position."""
    pair = canned_msgs.get_pair(args.pair)
    if pair is None:
        _fail(f"Unknown CPR pair {args.pair!r}. Known: "
              f"{', '.join(canned_msgs.all_pair_names())}")
        return 2

    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    with link:
        link.drain()

        # Inject even then odd so the device's pairing cache completes a global
        # decode; the odd is the most-recent frame, anchoring the longitude zone.
        for label, fhex in (("even", pair.even_hex), ("odd", pair.odd_hex)):
            r = inject_frame(link, fhex, timeout=args.timeout)
            if not r.ok:
                _fail(f"Device rejected {label} frame: {r.raw_reply or r.reason}")
                return 1
            _ok(f"Injected {label}: {r.raw_reply}")

        # Let the decode + publish cycle run, then capture the table.
        lines = link.read_lines(args.duration)

    blocks = sink_debug_parse.parse_lines(lines)
    tgt = sink_debug_parse.find_target(blocks, pair.icao)
    if tgt is None:
        _fail(f"ICAO {pair.icao:06X} never appeared after pair injection.")
        return 1
    if tgt.lat_deg is None or tgt.lon_deg is None:
        _fail(f"ICAO {pair.icao:06X} appeared but no position was resolved "
              f"(CPR pairing failed on device?).")
        return 1

    # Accept a match against EITHER anchoring — the device may have closed the
    # pair on the even or the odd depending on arrival/processing order.
    got = (tgt.lat_deg, tgt.lon_deg)
    for label, truth in (("odd-anchored", pair.odd_anchored),
                         ("even-anchored", pair.even_anchored)):
        if _pos_close(got, truth):
            _ok(f"Resolved position {got[0]:.5f},{got[1]:.5f} matches "
                f"{label} ground truth {truth[0]:.5f},{truth[1]:.5f}.")
            return 0

    _fail(f"Resolved position {got[0]:.5f},{got[1]:.5f} matches neither "
          f"anchoring (even {pair.even_anchored}, odd {pair.odd_anchored}).")
    return 1


def _compare_target(frame: canned_msgs.CannedFrame,
                    tgt: sink_debug_parse.DebugTarget) -> bool:
    """
    Compare a decoded sink_debug target against a frame's ground truth.

    Only the fields the frame's truth actually populates are checked; each
    mismatch is reported. Returns True iff every checked field agrees.
    """
    truth = frame.truth
    ok = True

    # Callsign (exact, case-sensitive — the device emits it verbatim).
    if truth.callsign is not None:
        if tgt.callsign != truth.callsign:
            _fail(f"  CS: got {tgt.callsign!r}, want {truth.callsign!r}")
            ok = False
        else:
            _info(f"  CS = {tgt.callsign!r} (match)")

    # Emitter category.
    if truth.emitter_category is not None and tgt.cat is not None:
        if tgt.cat != truth.emitter_category:
            _fail(f"  CAT: got {tgt.cat}, want {truth.emitter_category}")
            ok = False
        else:
            _info(f"  CAT = {tgt.cat} (match)")

    # Altitude (exact feet).
    if truth.altitude_ft is not None:
        if tgt.alt_ft != truth.altitude_ft:
            _fail(f"  ALT: got {tgt.alt_ft}, want {truth.altitude_ft}")
            ok = False
        else:
            _info(f"  ALT = {tgt.alt_ft}ft (match)")

    # Ground speed (allow ±1 kt for the device's rounding choice).
    if truth.ground_speed_kt is not None and tgt.gs_kt is not None:
        if abs(tgt.gs_kt - truth.ground_speed_kt) > 1:
            _fail(f"  GS: got {tgt.gs_kt}, want {truth.ground_speed_kt}")
            ok = False
        else:
            _info(f"  GS = {tgt.gs_kt}kt (match)")

    # Track (allow ±1° for rounding).
    if truth.track_deg is not None and tgt.trk_deg is not None:
        if abs(tgt.trk_deg - truth.track_deg) > 1:
            _fail(f"  TRK: got {tgt.trk_deg}, want {truth.track_deg}")
            ok = False
        else:
            _info(f"  TRK = {tgt.trk_deg} (match)")

    # Vertical rate (allow ±64 fpm — one encoding LSB).
    if truth.vertical_rate_fpm is not None and tgt.vr_fpm is not None:
        if abs(tgt.vr_fpm - truth.vertical_rate_fpm) > 64:
            _fail(f"  VR: got {tgt.vr_fpm}, want {truth.vertical_rate_fpm}")
            ok = False
        else:
            _info(f"  VR = {tgt.vr_fpm}fpm (match)")

    return ok


def _pos_close(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
    """True if two lat/lon points agree within the position tolerance."""
    return (abs(a[0] - b[0]) <= _POS_TOL_DEG
            and abs(a[1] - b[1]) <= _POS_TOL_DEG)


# ═════════════════════════════════════════════════════════════════════════════
#  Argument parsing / dispatch
# ═════════════════════════════════════════════════════════════════════════════
def _add_link_args(p: argparse.ArgumentParser) -> None:
    """Attach the shared port/baud options used by every device-touching command."""
    p.add_argument("--port", default=None,
                   help="Serial port (e.g. COM7 / /dev/ttyACM0). "
                        "Default: auto-detect by VID:PID.")
    p.add_argument("--baud", type=int, default=cdc_link.DEFAULT_BAUD,
                   help="Line rate (ignored by USB-CDC). Default: 115200.")


def build_parser() -> argparse.ArgumentParser:
    """Construct the full argparse tree for the bench CLI."""
    parser = argparse.ArgumentParser(
        prog="adsbin_bench",
        description="ADSBin ESP32-P4 host bench harness (USB-CDC / GDL90).")
    sub = parser.add_subparsers(dest="command", required=True)

    # list-ports — no link options needed; it enumerates everything.
    p_list = sub.add_parser("list-ports",
                            help="List serial ports and flag ADSBin devices.")
    p_list.set_defaults(func=cmd_list_ports)

    # validate-gdl90
    p_val = sub.add_parser("validate-gdl90",
                           help="Capture and CRC-check the live GDL90 stream.")
    _add_link_args(p_val)
    p_val.add_argument("--duration", type=float, default=3.0,
                       help="Capture window in seconds. Default: 3.0.")
    p_val.add_argument("--show", action="store_true",
                       help="Print each decoded frame, not just the summary.")
    p_val.set_defaults(func=cmd_validate_gdl90)

    # dump-debug
    p_dbg = sub.add_parser("dump-debug",
                           help="Capture and parse the sink_debug traffic table.")
    _add_link_args(p_dbg)
    p_dbg.add_argument("--duration", type=float, default=3.0,
                       help="Capture window in seconds. Default: 3.0.")
    p_dbg.add_argument("--raw", action="store_true",
                       help="Also echo the raw captured text lines.")
    p_dbg.add_argument("--all", action="store_true",
                       help="Print every block, not just the latest.")
    p_dbg.set_defaults(func=cmd_dump_debug)

    # inject
    p_inj = sub.add_parser("inject",
                           help="Push one raw Mode-S frame via +INJECT.")
    _add_link_args(p_inj)
    g = p_inj.add_mutually_exclusive_group(required=True)
    g.add_argument("--hex", help="Raw frame hex (14 or 28 chars).")
    g.add_argument("--name", help="A canned frame name (see list-canned).")
    p_inj.add_argument("--timeout", type=float, default=2.0,
                       help="Seconds to await +OK/+ERR. Default: 2.0.")
    p_inj.set_defaults(func=cmd_inject)

    # inject-verify
    p_ver = sub.add_parser(
        "inject-verify",
        help="Inject a frame/pair and verify the decode vs ground truth.")
    _add_link_args(p_ver)
    gv = p_ver.add_mutually_exclusive_group(required=True)
    gv.add_argument("--name", help="A canned frame name (carries ground truth).")
    gv.add_argument("--hex", help="Raw frame hex (no ground-truth checks).")
    gv.add_argument("--pair", help="A canned CPR pair name (verifies position).")
    p_ver.add_argument("--timeout", type=float, default=2.0,
                       help="Seconds to await +OK/+ERR. Default: 2.0.")
    p_ver.add_argument("--duration", type=float, default=3.0,
                       help="Seconds to capture sink_debug after inject. "
                            "Default: 3.0.")
    p_ver.set_defaults(func=cmd_inject_verify)

    # spoof — synthesize live traffic around a point and stream it via +INJECT.
    p_spoof = sub.add_parser(
        "spoof",
        help="Spawn moving fake traffic around you (or --lat/--lon) for ForeFlight.")
    _add_link_args(p_spoof)
    p_spoof.add_argument("--lat", type=float, default=None,
                         help="Scenario centre latitude (deg). Default: locate by IP.")
    p_spoof.add_argument("--lon", type=float, default=None,
                         help="Scenario centre longitude (deg). Default: locate by IP.")
    p_spoof.add_argument("--interval", type=float, default=1.0,
                         help="Seconds between position updates. Default: 1.0.")
    p_spoof.add_argument("--timeout", type=float, default=2.0,
                         help="Seconds to await each +OK/+ERR. Default: 2.0.")
    p_spoof.add_argument("--verbose", action="store_true",
                         help="Print per-cycle injection diagnostics.")
    p_spoof.set_defaults(func=cmd_spoof)

    # list-canned — a convenience that needs no device.
    p_can = sub.add_parser("list-canned",
                           help="List the built-in canned frames and CPR pairs.")
    p_can.set_defaults(func=cmd_list_canned)

    return parser


def _resolve_center(args: argparse.Namespace) -> Optional[Tuple[float, float]]:
    """
    Resolve the scenario centre: explicit --lat/--lon, else IP geolocation.

    @return  (lat, lon) in degrees, or None after printing a diagnostic.
    """
    # Explicit coordinates always win - no network, fully deterministic.
    if args.lat is not None and args.lon is not None:
        return (args.lat, args.lon)

    # Otherwise approximate the host's location by IP (city-level; plenty close
    # for "put some planes near me"). Kept optional so the bench's core has no
    # network dependency - only this convenience path reaches out.
    _info("No --lat/--lon given; locating you by IP ...")
    try:
        import json
        import urllib.request
        with urllib.request.urlopen(
                "http://ip-api.com/json/?fields=status,lat,lon,city,regionName",
                timeout=10) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        if data.get("status") == "success":
            _ok(f"Located near {data.get('city')}, {data.get('regionName')} "
                f"({data['lat']:.4f}, {data['lon']:.4f})")
            return (float(data["lat"]), float(data["lon"]))
        _fail(f"IP geolocation failed: {data.get('status')}")
    except Exception as e:
        _fail(f"IP geolocation error: {e}")
    _fail("Pass --lat <deg> --lon <deg> explicitly.")
    return None


def cmd_spoof(args: argparse.Namespace) -> int:
    """
    Synthesize live ADS-B traffic around a point and stream it via +INJECT.

    Builds a few low/slow aircraft around the resolved centre, then loops:
    advance each target by the publish interval, render its position/velocity/
    id frames, inject them, and sleep - so the targets crawl across the
    ForeFlight map in real time over the device's WiFi GDL90 broadcast.
    """
    import time as _time
    import spoof_traffic

    center = _resolve_center(args)
    if center is None:
        return 2
    clat, clon = center

    # Build the scenario. Only "low-slow" is shipped today; the flag is here so
    # more scenarios can slot in without changing the CLI surface.
    targets = spoof_traffic.low_slow_scenario(clat, clon)
    _info(f"Spoofing {len(targets)} target(s) around ({clat:.4f}, {clon:.4f}); "
          f"Ctrl-C to stop.")
    for t in targets:
        _info(f"  {t.callsign} ICAO={t.icao:06X} alt={t.alt_ft}ft "
              f"gs={t.gs_kt:.0f}kt trk={t.track_deg:.0f}")

    try:
        link = cdc_link.open_link(args.port, args.baud)
    except RuntimeError as e:
        _fail(str(e))
        return 2

    # Cadence: one update per --interval seconds. Each cycle we send, per target,
    # an identification frame (so ForeFlight shows the callsign), a velocity
    # frame, and a position frame (alternating even/odd so a CPR pair completes
    # within two cycles and ForeFlight can globally resolve the fix).
    sent = 0
    with link:
        try:
            while True:
                for t in targets:
                    for fhex in (t.frame_ident(), t.frame_velocity(),
                                 t.frame_position()):
                        res = inject_frame(link, fhex, timeout=args.timeout)
                        if res.ok:
                            sent += 1
                        elif args.verbose:
                            _fail(f"{t.callsign}: {res.raw_reply or res.reason}")
                    # Move the aircraft for next cycle.
                    t.step(args.interval)
                if args.verbose:
                    _info(f"cycle done; {sent} frames injected total")
                _time.sleep(args.interval)
        except KeyboardInterrupt:
            print()
            _ok(f"Stopped. Injected {sent} frames across {len(targets)} targets.")
    return 0


def cmd_list_canned(args: argparse.Namespace) -> int:
    """Print the corpus of canned frames and CPR pairs with their ground truth."""
    print("Canned frames:")
    for f in canned_msgs.CANNED_FRAMES:
        print(f"  {f.name:<22} {f.hex}")
        print(f"      {f.note}")

    print("\nCPR pairs (even+odd with verified position):")
    for p in canned_msgs.CPR_PAIRS:
        print(f"  {p.name:<22} ICAO {p.icao:06X}")
        print(f"      even {p.even_hex}")
        print(f"      odd  {p.odd_hex}")
        print(f"      pos  even-anchored={p.even_anchored} "
              f"odd-anchored={p.odd_anchored}")
        print(f"      {p.note}")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    """CLI entry point. Returns a process exit code."""
    _setup_console()
    parser = build_parser()
    args = parser.parse_args(argv)
    # Every subparser sets `func`; dispatch to it.
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
