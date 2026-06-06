# Upstream license texts

This directory holds the **verbatim** license file of every piece of third-party
code incorporated into ADSBin, one file per dependency
(e.g. `dump1090-antirez-LICENSE.txt`, `librtlsdr-LICENSE.txt`).

It is intentionally empty in the scaffold: **nothing has been borrowed yet.**
Before any external code lands in the tree, follow the procedure in
[`THIRD_PARTY.md`](../THIRD_PARTY.md):

1. Drop the upstream license here as `licenses/<name>-LICENSE.txt`.
2. Preserve the upstream copyright/attribution notices in the borrowed source.
3. Add/flip the matching row in `THIRD_PARTY.md` to the verified license + status.

See the **license-compatibility warning** at the top of `THIRD_PARTY.md` — the
PolyForm-Noncommercial vs GPLv3 question must be resolved before any copyleft
ADS-B code is incorporated into a distributed build.
