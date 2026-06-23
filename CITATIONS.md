# Algorithm Citations & Clean-Room Provenance

This file records the **published, non-GPL sources** from which ADSBin's signal-processing
algorithms were derived, and documents the **clean-room process** used so that no copyleft
(GPL) source code lineage enters the project.

It complements [`THIRD_PARTY.md`](THIRD_PARTY.md), which tracks *incorporated code* and its
licenses. **This file tracks *ideas/algorithms*** — the math and techniques we implemented
ourselves from public literature.

> **Why this matters.** Copyright protects the *expression* of an idea (the actual source
> code), **not** the idea, algorithm, mathematical method, or system of operation it
> describes (see 17 U.S.C. § 102(b) — "In no case does copyright protection ... extend to
> any idea, procedure, process, system, method of operation ..."). An algorithm described
> in a textbook or paper may therefore be implemented in original code without inheriting
> the license of any *other* program that also implements it. ADSBin's demodulators are
> written from the published *math*, not transcribed from any GPL program.

---

## Clean-room rule (followed for every algorithm below)

1. **Permitted inputs:** academic papers, textbooks, standards documents, datasheets, and
   public technical writeups describing the *math/technique* — cited below with author,
   title, and license/access terms.
2. **Forbidden inputs:** the source code of any GPL/copyleft ADS-B program (dump1090 and its
   forks, dump978, the SoftRF/dump5892 lineage, librtlsdr's GPL files, etc.) was **NOT**
   copied, transcribed, paraphrased line-by-line, or adapted into ADSBin's code.
3. **Derivation recorded in-code:** each algorithm's source file carries comments deriving
   the implementation from the cited math (e.g. matched-filter taps derived from the pulse
   shape and sample geometry), so a reviewer can check the derivation against the citation
   rather than against another program.

> Note on prior BSD lineage: ADSBin's *original* 2.0 Msps preamble/slicer
> (`components/demod1090`) carries, in its own header, a BSD-2-Clause attribution to
> S. Sanfilippo's (antirez) original dump1090, which is **permissively** licensed (BSD), not
> GPL. That attribution is preserved there. The **multi-phase 2.4 Msps** work described below
> is independent of it and is derived from the published math cited here.

---

## A. Mode-S / ADS-B physical layer (preamble, PPM, framing, CRC)

**Implemented in:** `components/demod1090/`, `components/modes_decode/`

**Primary citation — open textbook (Creative-Commons-style open access):**
- Junzi Sun, *The 1090 Megahertz Riddle: A Guide to Decoding Mode S and ADS-B Signals*
  (2nd edition, 2021). TU Delft. Open-access online edition at
  <https://mode-s.org/decode> (a.k.a. mode-s.org / junzis.com).
  Used for: preamble pulse timing (0.0, 1.0, 3.5, 4.5 µs pulses in an 8 µs preamble), the
  PPM bit convention (a `1` = pulse in the first 0.5 µs half-bit, `0` = pulse in the second),
  the 56-/112-bit frame structure (5-bit DF header + 24-bit parity), and the CRC-24
  generator polynomial **0xFFF409** with the "remainder == 0 for a valid frame" check.

These are also defined by the governing **standards** (independent of any source code):
- ICAO Annex 10, Volume IV — Aeronautical Telecommunications (SSR Mode S, 1090 MHz).
- RTCA DO-260B / EUROCAE ED-102A — *Minimum Operational Performance Standards for 1090 MHz
  Extended Squitter ADS-B*. (The PPM modulation, preamble, and CRC are specified here.)

---

## B. Multi-phase (sub-sample) PPM demodulation at 2.4 Msps

**Implemented in:** `components/demod1090/` (the phase-correction path)

**The problem (published, well-known):** at exactly 2× oversampling (2.0 Msps for the 1 Mbps
PPM signal), one bit is exactly two samples and a simple "first-half vs second-half"
magnitude comparison is exact. At higher oversampling (e.g. 2.4 Msps) the symbol/sample
phase is fractional and slides, so a single-phase slicer recovers wrong bits even though the
(tolerant) preamble correlator still locks. The robust remedy is to estimate the sampling
phase from the signal itself and correct for it before slicing.

**Primary citation — public DSP writeup (non-GPL prose/technique, not code):**
- John Gentile, "ADS-B" signal-processing notes, <https://john-gentile.com/kb/dsp/ADS-B.html>.
  Used for: the **zero-mean preamble correlation template** (subtract the template mean so
  noise correlates near zero), and the **energy-leakage phase-correction** method — estimate
  whether sampling is early/late from the energy that leaks into the samples adjacent to the
  preamble's on-time peaks (peaks at sample indices {0, 2, 7, 9} at 2 MHz), form a fractional
  leakage ratio α, and rescale each data sample up/down by a factor derived from α and the
  neighbouring bit's pulse side before re-slicing and re-checking CRC.

**Underlying theory (textbook, license-irrelevant):**
- Matched-filter / correlation receiver theory and fractional-delay sampling — standard
  digital-communications material, e.g. J. G. Proakis & M. Salehi, *Digital Communications*
  (matched filter as the optimal detector for a known pulse in AWGN). The per-phase
  correlation weights used for sub-sample slicing are the **matched-filter taps for the
  half-symbol pulse evaluated at each sample-phase hypothesis**, derived directly from the
  pulse shape and the 2.4 Msps sample geometry — not taken from any program.

**Clean-room note for §B:** the GPL-licensed dump1090 `demod_2400.c` is *known to exist* and
*known to use* a multi-phase approach, but its source was **not** used as the implementation
reference. ADSBin's correlation taps are re-derived from the matched-filter definition and
the sample geometry above, and the phase-correction is implemented from Gentile's described
energy-leakage method. Any numerical similarity in correlation coefficients is the
unavoidable consequence of both implementations approximating the *same optimal matched
filter* for the *same physical pulse* — which is exactly the uncopyrightable math, not
copied expression.

---

## C. CPR position decoding

**Implemented in:** `components/modes_decode/cpr.c`

- Junzi Sun, *The 1090 Megahertz Riddle* (above) — Compact Position Reporting (CPR) global
  and local decoding equations.
- RTCA DO-260B / ICAO Annex 10 Vol IV — the normative CPR algorithm (NL table, dlat/dlon,
  even/odd pairing).

---

## How to extend this file

When you implement a new algorithm from published material:
1. Add a section here with the **author, title, URL/DOI, and license/access terms** of the
   source(s).
2. State plainly that **no GPL program source was used** as the implementation reference.
3. Put the **derivation in the code's comments** so a reviewer can verify the algorithm came
   from the cited math.
