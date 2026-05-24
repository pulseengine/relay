# Falcon v1.0 dossier — credit-bundle crosswalks

This directory is the **assessor-facing index** for falcon's safety-
domain credit-bundles. One Markdown file per domain; each file is a
crosswalk that maps every standard's relevant objective to the
falcon evidence (rivet artifact, Verus / Kani / miri / witness
property, test, build target) that supports it — or to an explicit
**GAP** entry naming what's still missing.

| Domain                          | Status        | File                                            |
|---------------------------------|---------------|-------------------------------------------------|
| DO-178C DAL-A (avionics)        | v0.12 scaffold | [DO-178C-DAL-A-mapping.md](DO-178C-DAL-A-mapping.md) |
| ISO 26262 ASIL-D (automotive)   | v0.13 scaffold | [ISO-26262-ASIL-D-mapping.md](ISO-26262-ASIL-D-mapping.md) |
| IEC 61508 SIL-4 (general E/E/PE)| v0.13 scaffold | [IEC-61508-SIL-4-mapping.md](IEC-61508-SIL-4-mapping.md) |
| IEC 62304 Class C (medical)     | v0.13 scaffold | [IEC-62304-Class-C-mapping.md](IEC-62304-Class-C-mapping.md) |
| ECSS-Q-ST-80C Cat A (space)     | v0.13 scaffold | [ECSS-Q-ST-80C-Cat-A-mapping.md](ECSS-Q-ST-80C-Cat-A-mapping.md) |
| EN 50128 SIL-4 (rail)           | v0.13 scaffold | [EN-50128-SIL-4-mapping.md](EN-50128-SIL-4-mapping.md) |

## Tool qualification records (v0.14.3)

Cross-cutting documents under [`tool-qualification/`](tool-qualification/README.md)
— one per verification tool — close the "tool qualification" gap
that appears in every domain crosswalk simultaneously.

| Tool | TQ record                                       |
|------|--------------------------------------------------|
| Verus  | [Verus-TQ.md](tool-qualification/Verus-TQ.md)   |
| Kani   | [Kani-TQ.md](tool-qualification/Kani-TQ.md)     |
| miri   | [miri-TQ.md](tool-qualification/miri-TQ.md)     |
| witness| [witness-TQ.md](tool-qualification/witness-TQ.md) |
| spar   | [spar-TQ.md](tool-qualification/spar-TQ.md)     |

## How to read a crosswalk

Two-column table. Left = standard's objective ID + description. Right
= falcon evidence or **GAP**. v1.0 ships when every cell is either
populated or carries a deliberate, documented deferral.

The **gap list** at the bottom of each file is the single most
useful section for an assessor — it names what is *not yet* in the
package, with the work needed to close each gap.

## How the same evidence supports multiple domains

Falcon's verification stack is technique-class–coherent:
**Verus** (deductive FM) + **Kani** (bounded model checking) +
**miri** (robust testing with UB detection) + **witness** (MC/DC on
Wasm). Each standard recognises some subset of these under its own
naming convention. The crosswalks point all six domains at the
**same** underlying artifacts; the work is in naming the mapping,
not in re-running the verification.
