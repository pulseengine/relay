# Tool qualification — falcon verification toolchain

Five tool-qualification (TQ) records, one per verification tool
falcon relies on. Each TQ document classifies the tool per the four
safety standards that demand a formal qualification record
(IEC 61508-3 §7.4.4.7, ISO 26262-8 §11, ECSS-Q-ST-80C §5.4.8,
EN 50128 §6.7.4) and records the qualification approach falcon takes.

Closes one explicit **GAP** entry in every dossier crosswalk
simultaneously (the "tool qualification" gap appearing in
[DO-178C-DAL-A](../DO-178C-DAL-A-mapping.md),
[ISO-26262-ASIL-D](../ISO-26262-ASIL-D-mapping.md),
[IEC-61508-SIL-4](../IEC-61508-SIL-4-mapping.md),
[IEC-62304-Class-C](../IEC-62304-Class-C-mapping.md),
[ECSS-Q-ST-80C-Cat-A](../ECSS-Q-ST-80C-Cat-A-mapping.md),
[EN-50128-SIL-4](../EN-50128-SIL-4-mapping.md)).

| Tool   | TQ document                       | Use in falcon                                              |
|--------|-----------------------------------|------------------------------------------------------------|
| Verus  | [Verus-TQ.md](Verus-TQ.md)        | Formal proof (deductive verification) of engine contracts  |
| Kani   | [Kani-TQ.md](Kani-TQ.md)          | Bounded model checking over arbitrary input domains        |
| miri   | [miri-TQ.md](miri-TQ.md)          | Dynamic UB detection + robust testing                      |
| witness| [witness-TQ.md](witness-TQ.md)    | MC/DC structural coverage on Wasm                          |
| spar   | [spar-TQ.md](spar-TQ.md)          | AADL architectural modelling + EMV2 fault-tree analysis    |

## Cross-standard classification framework

Each TQ document uses one consistent table:

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | T1 / T2 / T3            | …         |
| ISO 26262-8 §11           | TCL1 / TCL2 / TCL3      | …         |
| ECSS-Q-ST-80C §5.4.8      | Category A / B / C / D  | …         |
| EN 50128 §6.7.4           | T1 / T2 / T3            | …         |

Each row is the result of two questions per the standards:

1. **Does the tool's output influence the safety-related software?**
2. **Can errors in the tool's output be detected by other independent means?**

For a verifier (Verus / Kani / miri / witness / spar), the answers
are usually:
- Output influences the safety-related software (the proof / counter-
  example / coverage report informs whether code is accepted) → not T1.
- Errors in the tool's output detected by independent means? — IF
  every claim a single tool makes is *cross-confirmed* by another
  technique class (e.g. Verus proves a property + Kani exhaustively
  enumerates the same property + miri runs it concretely), then
  yes — the technique-class diversity is itself the independent
  detection mechanism. This is what lets falcon stay at the lower
  tool-class for each individual tool, even at SIL-4 / ASIL-D / Cat A.

The TQ records below name the cross-confirmation for each tool's
claims explicitly.

## What qualification means at this stage

These TQ records are **draft-level documents** — they record falcon's
classification reasoning + the qualification approach (operational
history, technique-class cross-confirmation, version pinning) but do
not constitute a formal assessor-signed Tool Qualification Plan or
Report. v1.0 is when an external assessor signs these off; v0.14.3
is when they exist as auditable starting points.
