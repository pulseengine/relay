# relay-iekf — Invariant EKF design (v0.21 keystone)

The full-state navigation filter that replaces the attitude-only Mahony
complementary filter (`relay-ekf`). Fixes RC#3 by predicting attitude
through the rigid-body dynamics and correcting with GPS/baro, instead of
trusting the accelerometer as an instantaneous gravity reference.

Frame: NED (North-East-Down). Gravity `g = [0, 0, +9.81]` (down +).

## Nominal state (the extended pose + IMU biases)

```
X = (R, v, p, b_g, b_a)
  R   ∈ SO(3)   body→NED rotation        (stored as unit quaternion)
  v   ∈ R³      NED velocity
  p   ∈ R³      NED position
  b_g ∈ R³      gyro bias
  b_a ∈ R³      accelerometer bias
```

(R, v, p) is the "extended pose" on the matrix Lie group SE₂(3).

## IMU propagation (high rate, dt)

With measured body rate `ω_m` and specific force `a_m`:

```
ω = ω_m − b_g
a = a_m − b_a
R⁺ = R · Exp(ω · dt)                     (SO(3) exponential)
v⁺ = v + (R·a + g) · dt
p⁺ = p + v·dt + ½ (R·a + g) · dt²
b_g⁺ = b_g ,  b_a⁺ = b_a                  (random-walk nominal)
```

Note `v̇ = R·a + g` is the rigid-body dynamics: the accelerometer is a
**dynamics input**, never a gravity oracle. This is the structural fix
for RC#3.

## Invariant error (right-invariant) — why it's an IEKF, not an EKF

The error is defined through the **group**, not additively. For the
right-invariant convention, the extended-pose error is

```
η = X · X̂⁻¹        (group error)        ξ = Log(η) ∈ R⁹  (+ bias errors → R¹⁵)
```

The decisive property (Barrau & Bonnabel): for this error, the
linearized error propagation

```
ξ̇ = A · ξ + noise
```

has an `A` matrix whose pose block is **independent of the state
estimate trajectory** ("group-affine"). A *naive* error-state EKF has an
`A` that depends on the current estimate, which is the source of its
inconsistency (false convergence). The IEKF's state-independent `A` is
what gives:
- **consistency** (the covariance never lies about its own uncertainty),
  testable by NEES staying inside its χ²(15) envelope; and
- a clean, state-independent object that Lean/Rocq prove well — the
  invariant IS a geometric symmetry.

For the right-invariant IMU error the continuous `A` (pose block,
ordering [δθ, δv, δp]) is

```
        δθ      δv      δp        b_g          b_a
δθ̇ [   0       0       0      −R̂        0    ]
δv̇ [  [g]×     0       0       0       −R̂    ]      ([g]× = skew(g))
δṗ [   0       I       0       0        0    ]
```

— the pose block (top-left 9×9) has no dependence on R̂'s *value* beyond
the fixed `[g]×` and identity couplings; the only `R̂` terms are in the
bias-input columns. Contrast the naive EKF, whose δv̇ row carries a
`−R̂[a]×` term that drags the whole trajectory in.

Covariance: `P⁺ = Φ P Φᵀ + Q`, `Φ = Exp(A·dt) ≈ I + A·dt` (15×15).

## Measurement updates (right-invariant observations)

**GPS / NED position** `z = p + n`,  `n ~ N(0, R_gps)`:
For the right-invariant error, the position innovation `y = z − p̂`
has a **constant** measurement Jacobian `H = [0  0  I  0  0]` (3×15) —
state-independent, the matching half of the group-affine story.

**Baro / altitude** `z = p_down + n`: `H = [0…0  1  0…0]` (selects p[2]).

Kalman update (3×3 innovation cov is cheaply invertible):
```
S = H P Hᵀ + R_meas        (3×3)
K = P Hᵀ S⁻¹               (15×3)
ξ = K · y                  (15)
```
**Injection** (retraction back onto the group — NOT additive on R):
```
R̂ ← Exp(ξ_θ) · R̂          (right-invariant injection)
v̂ ← v̂ + ξ_v ,  p̂ ← p̂ + ξ_p
b_g ← b_g + ξ_bg ,  b_a ← b_a + ξ_ba
P  ← (I − K H) P
```

## Verification plan (defense-in-depth)

- **proptest invariants:** `‖q‖ = 1` after any propagate/update; `P`
  symmetric + PSD; no NaN/∞ under adversarial IMU/measurement streams.
- **Kani totality:** one propagate+update step is panic-free and finite
  for bounded inputs (bit-blast the f32 like relay-mix-quad / relay-arm).
- **Consistency (the IEKF claim):** NEES = ξᵀ P⁻¹ ξ vs gz ground truth
  stays inside the χ²(15) 95% envelope over a bench run. This is the
  mechanical oracle that an *error-state* EKF would fail and the IEKF
  passes — the whole reason for the architecture choice.
- **gz closed loop:** in the `hover` scenario, `est_tilt` now tracks
  `true_tilt` (< few °) instead of diverging to 51° → position-hold
  closes. This is the v0.20 RC#3 evidence, inverted.
- **Lean (later, shared with v0.23):** the group-affine consistency
  argument and the SE(3) geometry are the same foundation the geometric
  controller's Lyapunov proof rests on.

## Build order (incremental, multi-step)

1. Nominal state + SO(3) helpers (quat exp/log/mult/normalize) +
   IMU propagation. proptest: ‖q‖=1, no NaN.    ← foundation
2. 15×15 covariance + group-affine Φ propagation. proptest: P symmetric.
3. Position + baro updates with group injection. proptest: P PSD.
4. gz integration: replace relay-ekf in the cascade; measure est vs true
   tilt; NEES consistency oracle.
5. Kani totality harness; tune Q/R; close position-hold.
