# Disclaimer of Warranty, Liability, and Fitness for Use

**ADSBin — Copyright 2026 D Everett Hinton — Novabox.Works (https://novabox.works/)**

> **READ THIS BEFORE BUILDING, INSTALLING, OR OPERATING ADSBIN.**
>
> This disclaimer is in addition to, and not in place of, the warranty and
> liability terms of the [PolyForm Noncommercial License 1.0.0](LICENSE.md) under
> which the software, firmware, hardware designs, documentation, and all related
> files (collectively, the **"Project"**) are made available. Choosing,
> purchasing, building, flashing, installing, or operating any part of the Project,
> or any device incorporating it, constitutes your acceptance of this disclaimer in
> full. If you do not accept it, do not use the Project.

---

## 1. Experimental and Advisory Use Only

The Project is an **experimental, advisory traffic-awareness aid** intended solely for
use in **amateur-built / experimental category aircraft** by qualified persons who
accept full responsibility for its use. It is a **receive-only** device that listens
for ADS-B (and, in later configurations, UAT) broadcasts from *some* nearby aircraft
and presents their approximate, supplementary positions to a separate display.

The Project is **NOT** a flight instrument, **NOT** a navigation system, **NOT** a
collision-avoidance system, **NOT** a TCAS / ACAS / TAS / TCAD or any other traffic
advisory or resolution system, **NOT** a substitute for air traffic control, and
**NOT** a substitute for the pilot's see-and-avoid responsibility. It is **advisory
only**. Its outputs are convenience cues and nothing more.

## 2. Receive-Only — Not ADS-B Out — No Equipage Credit

The Project is **receive-only ("ADS-B In")**. It **does not transmit**, it does
**not** make your aircraft visible to ATC or to any other aircraft, and it does
**NOT** satisfy any ADS-B Out equipage requirement or mandate (including, without
limitation, 14 CFR §§ 91.225 / 91.227 or any equivalent rule in your jurisdiction).
Do not treat the presence of this device as meeting any equipment requirement of any
kind.

## 3. Not Certified — No Airworthiness Approval

The Project is **NOT** certified, approved, qualified, or authorized by the Federal
Aviation Administration (FAA), the European Union Aviation Safety Agency (EASA), or
any other civil aviation authority, regulatory body, or standards organization. It
has **not** been developed, tested, or documented in accordance with any aviation
software, hardware, environmental, or design-assurance standard (including, without
limitation, DO-178C, DO-254, DO-160, DO-260, DO-282, TSO, ETSO, or any AC/AMC
guidance).

No representation is made that the Project complies with any airworthiness
requirement. Installation or operation of the Project in any aircraft may be subject
to regulation in your jurisdiction, and **you alone** are responsible for determining
and ensuring compliance with all applicable laws, regulations, operating limitations,
and the airworthiness requirements governing your aircraft. Installation in
type-certificated aircraft is **not** intended or supported.

## 4. No Warranty of Any Kind

**TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, THE PROJECT IS PROVIDED
"AS IS" AND "AS AVAILABLE," WITH ALL FAULTS AND WITHOUT WARRANTY OR CONDITION OF
ANY KIND, WHETHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE.** The licensor and any
contributors expressly disclaim all warranties and conditions, including, without
limitation, any implied warranties or conditions of **merchantability, satisfactory
quality, fitness for a particular purpose, accuracy, reliability, availability,
title, and non-infringement**, and any warranties arising out of course of dealing,
course of performance, usage, or trade practice.

Without limiting the foregoing, **no warranty is made** that the Project will meet
your requirements; that it will be accurate, timely, complete, uninterrupted, secure,
or error-free; that any traffic target, position, altitude, or other output will be
correct, present, complete, or on time; or that defects will be corrected. **A wrong,
missing, late, ghost, or misleading target may occur at any time and without
warning, and aircraft may be entirely absent from the display.** You assume the
entire risk as to the quality, performance, accuracy, completeness, and timeliness of
the Project and any device built from it.

## 5. Limitation and Exclusion of Liability

**TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, IN NO EVENT SHALL THE LICENSOR,
THE AUTHOR(S), NOVABOX.WORKS, OR ANY CONTRIBUTOR, SUPPLIER, OR DISTRIBUTOR BE LIABLE
TO YOU OR TO ANY THIRD PARTY FOR ANY DAMAGES OF ANY KIND** arising out of or in any
way related to the Project or its use, inability to use, installation, operation, or
performance — including, without limitation, any **direct, indirect, incidental,
special, exemplary, punitive, or consequential damages**; loss of life, personal
injury, or bodily harm; property damage, including damage to or loss of any aircraft;
mid-air collision or loss of separation; loss of use, loss of data, loss of profits,
business interruption, or cost of substitute goods or services — **whether based in
contract, warranty, tort (including negligence), strict liability, product liability,
or any other legal or equitable theory, and whether or not the licensor has been
advised of, or could have foreseen, the possibility of such damages, and even if any
remedy is found to have failed of its essential purpose.**

Where applicable law does not permit the exclusion or limitation of certain damages
or of implied warranties, the disclaimers and limitations in this document shall
apply to the **fullest extent permitted by that law**, and the licensor's aggregate
liability shall be limited to the **greater of the amount you paid (if any) for the
specific copy of the Project giving rise to the claim, or zero**.

## 6. No Substitute for See-and-Avoid and Pilot Judgment

The Project is **never** a substitute for the pilot's **continuous outside visual
scan and see-and-avoid responsibility**, for air traffic control services, for
properly installed and approved traffic systems, for established procedures, for
adequate training and currency, or for the pilot's own situational awareness — most
especially in the traffic pattern, during taxi, takeoff, and approach, and in busy or
non-towered airspace.

**Do not maneuver, and do not fail to maneuver, on the basis of an ADSBin display.**
Treat every target — and every *apparently empty* piece of sky — as unverified,
supplementary information that may be wrong, incomplete, or absent. The pilot in
command is at all times solely responsible for traffic separation and the safe
operation of the aircraft and must be fully prepared to operate safely with **no
reliance whatsoever** on the Project.

## 7. Inherent ADS-B and Operational Limitations

The Project shows only what it can receive and decode, and is subject to inherent and
unavoidable limitations, including, without limitation:

- **It cannot see non-cooperative traffic.** Aircraft without ADS-B Out, with it
  switched off or inoperative, military or other exempt aircraft, gliders, balloons,
  birds, drones, and obstacles may be **completely invisible**. There is **no primary
  radar** in this device.
- **TIS-B / ADS-R rebroadcast traffic** depends on ground-station coverage and on a
  suitably equipped nearby client; it may be partial, delayed, or absent, and may
  show duplicate or "ghost" targets.
- **Reception is imperfect:** dropped, late, or garbled messages; antenna shadowing
  during maneuvering; range, terrain, and line-of-sight limits; interference and a
  raised noise floor; CPR/decoding ambiguity or error; and **position latency** mean a
  target's shown position may lag or differ from reality.
- **Configuration and environment:** range/altitude filtering, single-band operation
  (1090-only configurations do not see UAT-only traffic, and vice-versa), mounting,
  antenna, installation, or configuration error; electrical noise, brown-out, reboot,
  or power interruption; and software defects.

Under any of these conditions the Project may report nothing, omit real aircraft, or
report incorrectly, **without warning**.

## 8. Assumption of Risk and Indemnification

**You knowingly and voluntarily assume all risks** associated with building,
installing, operating, and relying on the Project. **Aviation is inherently
dangerous; experimental aircraft and experimental avionics compound that risk.** You
represent that you are qualified to make your own independent assessment of the
Project's suitability for your intended use.

To the maximum extent permitted by applicable law, you agree to **indemnify, defend,
and hold harmless** the licensor, the author(s), Novabox.Works, and all contributors,
suppliers, and distributors from and against any and all claims, demands, suits,
actions, losses, liabilities, damages, costs, and expenses (including reasonable
attorneys' fees) arising out of or related to your use, installation, distribution,
or operation of the Project, or any device incorporating it, or your breach of this
disclaimer or the [License](LICENSE.md).

## 9. Relationship to the License; Severability

This disclaimer supplements the **[PolyForm Noncommercial License 1.0.0](LICENSE.md)**.
In the event of any conflict between this disclaimer and that license with respect to
warranty or liability, the broadest disclaimer and the greatest limitation of
liability permitted by applicable law shall control. If any provision of this
disclaimer is held to be unenforceable or invalid, that provision shall be modified
or severed to the minimum extent necessary, and the remaining provisions shall remain
in full force and effect.

---

*By using ADSBin you acknowledge that you have read, understood, and agree to this
Disclaimer of Warranty, Liability, and Fitness for Use, and to the
[PolyForm Noncommercial License 1.0.0](LICENSE.md).*
