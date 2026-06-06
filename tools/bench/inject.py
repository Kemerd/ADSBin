# -*- coding: utf-8 -*-
"""
inject.py — Drive the firmware's ``+INJECT`` console command over USB-CDC.

``+INJECT`` lets the bench push a canned raw Mode-S frame into the device's *real*
``modes_decode`` path without live RF (WIRE_CONTRACT.md §2, plan S3.1). The
injected frame then surfaces in the next ``sink_debug`` block and GDL90 output,
so we can verify the on-device decoder deterministically.

Wire protocol (frozen, WIRE_CONTRACT.md §2)::

    host -> device:   +INJECT <hex>        (14 or 28 hex chars, no spaces)
    device -> host:   +OK                  (accepted, pushed into decode)
                      +ERR <reason>        (e.g. BADLEN, BADHEX)

This module is the thin request/response layer over a :class:`cdc_link.CdcLink`:
it validates the hex the same way the firmware will (so we fail fast on the host),
sends the line, and reads back the single ``+OK`` / ``+ERR`` reply. Position and
field verification live one layer up in adsbin_bench.py — this file only owns the
inject handshake.

Pure protocol over an injected link object — unit-testable with a fake link.
Windows-console / UTF-8 safe.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


# Valid raw-frame hex lengths the firmware accepts (short 56-bit, long 112-bit).
_VALID_HEX_LENS = (14, 28)


@dataclass
class InjectResult:
    """Outcome of a single ``+INJECT`` exchange."""
    ok: bool                    # True iff the device replied "+OK".
    reason: Optional[str]       # The "+ERR <reason>" text, or None on success.
    raw_reply: Optional[str]    # The exact reply line, for diagnostics/logging.


def normalize_frame_hex(frame_hex: str) -> str:
    """
    Validate and normalise a raw-frame hex string the way the firmware does.

    Mirrors the device-side BADLEN / BADHEX checks so the host rejects a bad
    frame BEFORE it hits the wire, giving a clearer error than a round-trip.

    @param frame_hex  Candidate frame hex (any case, no spaces).
    @return           Uppercase, validated hex.
    @raises ValueError  "BADLEN" if not 14/28 chars; "BADHEX" if non-hex. The
                        message text matches the firmware's ``+ERR`` reasons so
                        callers can present one consistent vocabulary.
    """
    # Strip incidental whitespace a user may have pasted; the wire form has none.
    s = frame_hex.strip().replace(" ", "")

    # Length gate first — the firmware keys length off the hex char count.
    if len(s) not in _VALID_HEX_LENS:
        raise ValueError("BADLEN")

    # Every character must be a hex digit; int() with base 16 is the cleanest
    # exact check and rejects sign/underscore tricks that bytes.fromhex allows.
    try:
        int(s, 16)
    except ValueError:
        raise ValueError("BADHEX")

    return s.upper()


def inject_frame(link, frame_hex: str, timeout: float = 2.0) -> InjectResult:
    """
    Send one ``+INJECT <hex>`` and read back the device's ``+OK`` / ``+ERR``.

    @param link       A live :class:`cdc_link.CdcLink` (or any object exposing
                      ``write_line`` and ``read_line(timeout)``).
    @param frame_hex  Raw Mode-S frame hex (14 or 28 chars, any case).
    @param timeout    Seconds to wait for the reply line.
    @return           An :class:`InjectResult`. Host-side validation failures are
                      returned as ``ok=False`` with the firmware's reason code, so
                      the caller handles local and remote rejects identically.
    """
    # Validate locally; surface the firmware's own reason vocabulary on failure.
    try:
        normalized = normalize_frame_hex(frame_hex)
    except ValueError as e:
        return InjectResult(ok=False, reason=str(e), raw_reply=None)

    # Clear any stale text so we don't mistake a previous reply for ours, then
    # send the command line. (drain() is a no-op on a fake link without it.)
    if hasattr(link, "drain"):
        link.drain()
    link.write_line(f"+INJECT {normalized}")

    # The reply may be preceded by unrelated console/log lines on the shared
    # link, so we scan up to a few lines for the first +OK / +ERR token.
    reply = _read_inject_reply(link, timeout)
    if reply is None:
        return InjectResult(ok=False, reason="TIMEOUT", raw_reply=None)

    # Parse the frozen reply grammar.
    if reply.startswith("+OK"):
        return InjectResult(ok=True, reason=None, raw_reply=reply)
    if reply.startswith("+ERR"):
        # "+ERR <reason>" — the reason is everything after the keyword.
        reason = reply[len("+ERR"):].strip() or "UNKNOWN"
        return InjectResult(ok=False, reason=reason, raw_reply=reply)

    # An unexpected line that isn't +OK/+ERR — treat as a protocol error but keep
    # the raw text so the operator can see what the device actually said.
    return InjectResult(ok=False, reason="UNEXPECTED", raw_reply=reply)


def _read_inject_reply(link, timeout: float) -> Optional[str]:
    """
    Read lines until a ``+OK`` / ``+ERR`` appears or the timeout elapses.

    The USB-CDC link interleaves the firmware console with our reply, so a few
    log lines can land between our command and its ``+OK``. We skip non-reply
    lines and return the first one that starts with the reply prefix.
    """
    import time

    deadline = time.monotonic() + timeout

    # Keep pulling lines (each with a short per-line timeout) until we either see
    # our reply or run out the overall budget.
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        line = link.read_line(timeout=remaining)
        if line is None:
            break

        stripped = line.strip()
        # Only +OK / +ERR are our concern; everything else is console noise.
        if stripped.startswith("+OK") or stripped.startswith("+ERR"):
            return stripped

    return None
