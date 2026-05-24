# miri — Tool Qualification record (falcon v0.14.3)

**Tool:** miri  (rustc mid-level IR interpreter with UB detection)
**Version pinned:** nightly toolchain channel (matches
`rustup component add miri --toolchain nightly`)
**Source:** https://github.com/rust-lang/miri  (in-tree with rustc)
**Operating mode:** concrete interpretation with Tree Borrows
(default), `-Zmiri-disable-isolation` for tests that touch the
filesystem.

## Use in falcon

Concrete interpretation of cargo unit tests, looking for:

- **Undefined behaviour** (uninitialised reads, dangling pointers,
  use-after-free, data races, alignment violations)
- **Integer overflow** in checked / debug paths
- **Out-of-bounds slice access**
- **Pointer-aliasing violations** (Stacked / Tree Borrows)

Falcon runs miri on the v0.12 geofence + bench tests:

- 5/5 `relay-lc` Geofence unit tests miri-clean
  (geofence_inside_does_not_trip, geofence_outside_n_trips_once,
  geofence_outside_e_trips, geofence_outside_d_trips,
  geofence_boundary_inclusive)
- 2/2 `falcon-hitl-rfspoof` stub-backend tests miri-clean
- 6/6 `falcon-hitl-rfspoof` mavlink-backend tests miri-clean

→ 13 tests miri-clean as of v0.12.0 (FV-FALCON-GEO-003).

## Cross-standard classification

| Standard                  | Falcon's classification | Rationale |
|---------------------------|-------------------------|-----------|
| IEC 61508-3 §7.4.4.7      | **T2** | Test execution tool that affects verification confidence; output (clean run vs UB report) is review-able and reproducible. |
| ISO 26262-8 §11           | **TCL2** — generates verification artifacts; errors detected by re-running and by comparison against cargo test results (UB-free under miri implies clean under cargo). |
| ECSS-Q-ST-80C §5.4.8      | **Category B**. |
| EN 50128 §6.7.4           | **T2**. |

**Honest categorisation note:** miri is a *dynamic concrete
interpreter*, not abstract interpretation in the textbook DO-178C
sense. Falcon's substitution of miri for "abstract interpretation"
in the rollout is recorded honestly in FV-FALCON-GEO-003.

## Qualification approach

| miri claim                   | Independent confirmation                                  |
|------------------------------|-----------------------------------------------------------|
| No UB in Geofence tests      | Verus + Kani prove the *function* never panics; miri runs the test invocations end-to-end with the actual concrete inputs. |
| No alignment violations      | `#![forbid(unsafe_code)]` on every verified crate         |
| No integer overflow          | Verus contracts on integer ranges where applicable        |
| Pointer-aliasing clean       | Cross-confirmed by the absence of unsafe code             |

miri runs orthogonal to Verus / Kani — Verus proves at the SMT
level, Kani enumerates symbolic states, miri runs the actual
compiled code with bit-precise UB sentinels. A miri failure on
verified code would mean either (a) the implementation diverges
from the verified spec, or (b) miri itself has a soundness bug —
the diversity rules out single-tool failure modes.

## Validation evidence

- `MIRIFLAGS=-Zmiri-disable-isolation cargo +nightly miri test -p
  relay-lc --lib geofence` — 5/5 pass.
- `MIRIFLAGS=-Zmiri-disable-isolation cargo +nightly miri test -p
  falcon-hitl-rfspoof --bin falcon-hitl-rfspoof stub::tests` — 2/2.
- `MIRIFLAGS=-Zmiri-disable-isolation cargo +nightly miri test -p
  falcon-hitl-rfspoof --bin falcon-hitl-rfspoof mavlink::tests` — 6/6.
- Recipe pinned in FV-FALCON-GEO-003 `steps:` field.
- miri ships in-tree with rustc and is validated by the Rust
  project's CI (we rely on upstream validation for the tool itself).

## Honestly out of scope

- A formal TQR. v1.0 work.
- miri runs on a *subset* of the test suite (proptest-driven tests
  trip on miri's filesystem isolation even with
  `-Zmiri-disable-isolation`). Each FV artifact names the subset
  it validates against.
- miri does not cover branches the tests don't exercise — that's
  what witness MC/DC is for (FV-FALCON-COV-001/002/003).
