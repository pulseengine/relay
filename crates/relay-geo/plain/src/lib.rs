//! Relay geometric SE(3) attitude controller (Lee, Leok, McClamroch 2010).
//!
//! Replaces the Euler-based attitude path (relay-att's quaternion error on
//! a `euler_to_quaternion` setpoint), which **couples tilt into yaw** once
//! heading ≠ 0 — the v0.22 finding that the body yawed freely under the
//! legacy cascade. The geometric controller works directly on SO(3): the
//! desired attitude is built from the thrust direction + heading
//! geometrically (no Euler decomposition), and the attitude error is the
//! intrinsic `e_R = ½(R_dᵀR − RᵀR_d)^∨`. No singularities, no cross-coupling.
//!
//! Control law (setpoint regulation, desired body rate Ω_d = 0):
//! ```text
//!   e_R = ½ (R_dᵀ R − Rᵀ R_d)^∨          (attitude error, body frame)
//!   e_Ω = Ω − Rᵀ R_d Ω_d = Ω             (Ω_d = 0)
//!   M   = −k_R e_R − k_Ω e_Ω + Ω × J Ω    (body moment / torque)
//! ```
//!
//! Stability (the differentiator): almost-global exponential stability
//! with the explicit Lyapunov function `V = ½ e_Ω·J·e_Ω + k_R Ψ(R,R_d)
//! (+ c e_R·e_Ω)`, where `Ψ(R,R_d) = ½ tr(I − R_dᵀR)`. The measure-zero
//! excluded set is the true SO(3) topological obstruction (no continuous
//! controller is globally stabilising) — we claim "almost-global", never
//! "global". The Lyapunov proof targets Lean/rules_lean4; here `psi` and
//! the decrease test are the in-crate evidence. See
//! docs/research/v0.23-control-sota.md.

#![no_std]
#![allow(clippy::needless_range_loop)]

pub type Vec3 = [f32; 3];
/// Row-major 3×3 matrix.
pub type Mat3 = [[f32; 3]; 3];

/// Standard gravity in NED (down +), m/s².
pub const GRAVITY_NED: Vec3 = [0.0, 0.0, 9.81];

// ───────────────────────────── vector / matrix ────────────────────────

#[inline]
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
#[inline]
fn dot(a: Vec3, b: Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
#[inline]
fn norm(a: Vec3) -> f32 {
    libm::sqrtf(dot(a, a))
}
#[inline]
fn normalize(a: Vec3) -> Option<Vec3> {
    let n = norm(a);
    if n.is_finite() && n > 1e-9 {
        Some([a[0] / n, a[1] / n, a[2] / n])
    } else {
        None
    }
}
/// 3×3 identity.
#[inline]
pub fn identity3() -> Mat3 {
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
}
#[inline]
fn transpose3(m: &Mat3) -> Mat3 {
    [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]
}
#[inline]
fn matmul3(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut o = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0;
            for k in 0..3 {
                s += a[i][k] * b[k][j];
            }
            o[i][j] = s;
        }
    }
    o
}
/// vee map: skew-symmetric matrix → 3-vector (inverse of hat).
#[inline]
fn vee(m: &Mat3) -> Vec3 {
    // m ≈ [[0,-c,b],[c,0,-a],[-b,a,0]] → [a,b,c]
    [
        0.5 * (m[2][1] - m[1][2]),
        0.5 * (m[0][2] - m[2][0]),
        0.5 * (m[1][0] - m[0][1]),
    ]
}

/// Body→NED rotation matrix from a unit quaternion (Hamilton, w-first).
pub fn quat_to_rotmat(q: [f32; 4]) -> Mat3 {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

// ───────────────────────── geometric construction ─────────────────────

/// Build the desired attitude `R_d` (body→NED) from the desired body-down
/// (thrust) axis `b3_d` and the desired NED heading `yaw`, geometrically —
/// NO Euler angles, hence no tilt-yaw coupling. Columns are the desired
/// body axes [b1, b2, b3] in NED. Returns `None` if the geometry is
/// degenerate (thrust axis parallel to the heading vector).
pub fn desired_attitude(b3_d: Vec3, yaw: f32) -> Option<Mat3> {
    let b3 = normalize(b3_d)?;
    // Desired heading direction in the horizontal NED plane (body-x points
    // at the heading). NED yaw is CW from North.
    let b1_des = [libm::cosf(yaw), libm::sinf(yaw), 0.0];
    // b2 ⟂ b3 and b1_des; b1 = b2 × b3 (re-orthogonalised).
    let b2 = normalize(cross(b3, b1_des))?;
    let b1 = cross(b2, b3);
    // Columns [b1 b2 b3].
    Some([
        [b1[0], b2[0], b3[0]],
        [b1[1], b2[1], b3[1]],
        [b1[2], b2[2], b3[2]],
    ])
}

/// Desired body-down (thrust) axis in NED for a commanded NED acceleration
/// `a_cmd`. The specific force is `a_cmd − g`; thrust acts along −body_z
/// (up), so body-z (down) aligns with `g − a_cmd`. At hover (a_cmd = 0)
/// this is `[0,0,1]` (level). Returns `None` if degenerate (free-fall).
pub fn thrust_axis_ned(a_cmd: Vec3) -> Option<Vec3> {
    normalize([
        GRAVITY_NED[0] - a_cmd[0],
        GRAVITY_NED[1] - a_cmd[1],
        GRAVITY_NED[2] - a_cmd[2],
    ])
}

/// Differential-flatness **feedforward body rate** (v0.27): the angular
/// velocity that rotates the desired thrust axis as a flat trajectory
/// demands, from the commanded NED acceleration `a_cmd` and jerk `j_cmd`.
/// With the desired body-down axis `b3 = (g − a)/c`, `c = ‖g − a‖`, the
/// world-frame thrust-axis rate is `ω_w = −(b3 × (−j))/c = (b3 × j)/c`;
/// the yaw-rate `ψ̇` adds rotation about `b3`. Returned in the BODY frame
/// of `R_d` (what the geometric controller / ADRC track as Ω_d
/// feedforward). `None` if the thrust axis is degenerate (free-fall).
///
/// Validated against the finite-difference of `R_d` along a trajectory
/// (`flatness_feedforward_matches_attitude_derivative`).
pub fn flatness_omega_ff(a_cmd: Vec3, j_cmd: Vec3, yaw: f32, yaw_rate: f32) -> Option<Vec3> {
    let tf = [
        GRAVITY_NED[0] - a_cmd[0],
        GRAVITY_NED[1] - a_cmd[1],
        GRAVITY_NED[2] - a_cmd[2],
    ];
    let c = norm(tf);
    if !c.is_finite() || c <= 1e-3 {
        return None;
    }
    let b3 = [tf[0] / c, tf[1] / c, tf[2] / c];
    // ω_world ⊥ b3 = −(b3 × j)/c  (d/dt(g−a) = −j; ω = b3 × ḃ3; sign pinned
    // by flatness_feedforward_matches_attitude_derivative).
    let bxj = cross(b3, j_cmd);
    let w_world = [-bxj[0] / c, -bxj[1] / c, -bxj[2] / c];
    let r_d = desired_attitude(b3, yaw)?;
    // ω_body = R_dᵀ ω_world, plus the yaw rate about body-z (b3).
    let mut w_body = [
        r_d[0][0] * w_world[0] + r_d[1][0] * w_world[1] + r_d[2][0] * w_world[2],
        r_d[0][1] * w_world[0] + r_d[1][1] * w_world[1] + r_d[2][1] * w_world[2],
        r_d[0][2] * w_world[0] + r_d[1][2] * w_world[1] + r_d[2][2] * w_world[2],
    ];
    w_body[2] += yaw_rate;
    Some(sanitise3(w_body))
}

// ────────────────────────────── controller ────────────────────────────

/// Geometric attitude controller gains + diagonal inertia.
#[derive(Clone, Copy)]
pub struct GeoGains {
    /// Attitude proportional gain (per axis).
    pub k_r: Vec3,
    /// Angular-rate gain (per axis).
    pub k_omega: Vec3,
    /// Diagonal body inertia (kg·m²) for the Ω × JΩ feed-forward.
    pub j: Vec3,
}

impl GeoGains {
    /// Defaults for the gz falcon-quad (2 kg, J≈0.0217). The almost-global
    /// proof requires k_R, k_Ω > 0; these are positive and roll/pitch >
    /// yaw (weak yaw authority).
    pub const FALCON_QUAD: Self = Self {
        // Yaw is rate-DOMINANT: low k_R (the attitude term uses the
        // lagged estimate and, driven hard, chases estimate noise into a
        // spin) but firm k_Ω (the rate term uses the fast raw gyro — pure
        // damping, no lag). Roll/pitch are gravity-anchored, so attitude-
        // dominant is fine there.
        k_r: [0.6, 0.6, 0.15],
        k_omega: [0.12, 0.12, 0.35],
        j: [0.0217, 0.0217, 0.04],
    };
}

pub struct GeoAtt {
    gains: GeoGains,
}

impl GeoAtt {
    pub fn new(gains: GeoGains) -> Self {
        GeoAtt { gains }
    }

    /// The configuration error function Ψ(R,R_d) = ½ tr(I − R_dᵀR) ≥ 0,
    /// zero iff R = R_d. The Lyapunov function's potential term — used by
    /// the decrease test (and the future Lean proof).
    pub fn psi(r: &Mat3, r_d: &Mat3) -> f32 {
        let rdt_r = matmul3(&transpose3(r_d), r);
        let tr = rdt_r[0][0] + rdt_r[1][1] + rdt_r[2][2];
        0.5 * (3.0 - tr)
    }

    /// Attitude error `e_R = (R_dᵀR − RᵀR_d)^∨` (body frame). NOTE: this is
    /// the FULL vee, i.e. **2× Lee's `½(R_dᵀR−RᵀR_d)^∨`** — the ½ is folded
    /// into the gain `k_R`. Consequence for the Lyapunov analysis: the
    /// Ψ-derivative identity is `Ψ̇ = ½ e_R·Ω` and the potential coefficient
    /// is `2 k_R` (so V = ½ Ω·JΩ + 2 k_R Ψ). See the
    /// `lyapunov_decrease_certificate` test.
    pub fn attitude_error(r: &Mat3, r_d: &Mat3) -> Vec3 {
        let rdt_r = matmul3(&transpose3(r_d), r);
        let rt_rd = matmul3(&transpose3(r), r_d);
        let mut diff = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                diff[i][j] = rdt_r[i][j] - rt_rd[i][j];
            }
        }
        vee(&diff)
    }

    /// **Reduced-attitude (S²) error** for fault-tolerant / thrust-axis
    /// control (v0.26). The body-frame angular error that aligns the
    /// body-down (thrust) axis to `b3_d`, with **no yaw component** —
    /// rotation about b3 is unconstrained. On rotor loss a quad cannot hold
    /// full attitude (Mueller & D'Andrea), so the controller drives only
    /// this 2-DoF tilt and relinquishes yaw. `e = e_z × (Rᵀ b3_d)` (its
    /// z-component is identically 0); ×2 to match the `attitude_error`
    /// convention so the same `k_R` applies. Zero iff b3 = b3_d.
    pub fn attitude_error_reduced(r: &Mat3, b3_d: Vec3) -> Vec3 {
        // Rᵀ·b3_d : the desired thrust axis expressed in the body frame.
        let v = [
            r[0][0] * b3_d[0] + r[1][0] * b3_d[1] + r[2][0] * b3_d[2],
            r[0][1] * b3_d[0] + r[1][1] * b3_d[1] + r[2][1] * b3_d[2],
            r[0][2] * b3_d[0] + r[1][2] * b3_d[1] + r[2][2] * b3_d[2],
        ];
        // v × e_z = [v_y, −v_x, 0], scaled ×2. (Sign matches the
        // `attitude_error` convention so −k_R·e drives b3 → b3_d, verified
        // by `reduced_attitude_drives_thrust_axis_to_target`.)
        [2.0 * v[1], -2.0 * v[0], 0.0]
    }

    /// Reduced-attitude (S²) moment for **single-rotor-out** flight: drive
    /// the thrust axis to `b3_d` (roll/pitch) and **relinquish yaw** (no yaw
    /// torque — the body spins freely about its near-vertical axis, which
    /// `mix_rotor_out` also drops). `M = −k_R e_reduced − k_Ω Ω_xy + Ω×JΩ`,
    /// yaw component 0.
    pub fn moment_reduced(&self, r: &Mat3, omega: Vec3, b3_d: Vec3) -> Vec3 {
        let e = Self::attitude_error_reduced(r, b3_d); // e[2] = 0
        let g = &self.gains;
        let j_omega = [g.j[0] * omega[0], g.j[1] * omega[1], g.j[2] * omega[2]];
        let gyro = cross(omega, j_omega);
        [
            -g.k_r[0] * e[0] - g.k_omega[0] * omega[0] + gyro[0],
            -g.k_r[1] * e[1] - g.k_omega[1] * omega[1] + gyro[1],
            0.0, // yaw relinquished
        ]
    }

    /// Body moment (torque) for setpoint regulation toward `r_d` with
    /// desired body rate zero: `M = −k_R e_R − k_Ω Ω + Ω × J Ω`.
    pub fn moment(&self, r: &Mat3, omega: Vec3, r_d: &Mat3) -> Vec3 {
        let e_r = Self::attitude_error(r, r_d);
        let g = &self.gains;
        let j_omega = [g.j[0] * omega[0], g.j[1] * omega[1], g.j[2] * omega[2]];
        let gyro = cross(omega, j_omega); // Ω × JΩ
        [
            -g.k_r[0] * e_r[0] - g.k_omega[0] * omega[0] + gyro[0],
            -g.k_r[1] * e_r[1] - g.k_omega[1] * omega[1] + gyro[1],
            -g.k_r[2] * e_r[2] - g.k_omega[2] * omega[2] + gyro[2],
        ]
    }

    /// Outer attitude loop producing a DESIRED BODY RATE (rad/s) instead
    /// of a torque — for a robust inner loop (ADRC/INDI) to track. The
    /// geometric attitude error maps to a rate command `Ω_d = −k_R e_R`.
    /// This is the v0.25 split: geometry decides *where to point* (the
    /// well-conditioned part), the inner loop handles *how to get the
    /// torque there* through the actuator dynamics (the part that broke
    /// the direct-torque yaw loop).
    pub fn desired_rate(&self, q_est: [f32; 4], a_cmd_ned: Vec3, yaw_d: f32) -> Vec3 {
        let r = quat_to_rotmat(q_est);
        let r_d = thrust_axis_ned(sanitise3(a_cmd_ned))
            .and_then(|b3| desired_attitude(b3, yaw_d));
        match r_d {
            Some(r_d) => {
                let e = Self::attitude_error(&r, &r_d);
                let g = &self.gains;
                sanitise3([-g.k_r[0] * e[0], -g.k_r[1] * e[1], -g.k_r[2] * e[2]])
            }
            None => [0.0; 3],
        }
    }

    /// One control tick: from the estimate quaternion + body rate, the
    /// commanded NED acceleration (horizontal manoeuvre), and the desired
    /// heading, build `R_d` geometrically and return the body torque.
    /// Falls back to a pure rate-damping torque if the desired-attitude
    /// geometry is degenerate (keeps the output total/finite).
    pub fn tick(&self, q_est: [f32; 4], omega_body: Vec3, a_cmd_ned: Vec3, yaw_d: f32) -> Vec3 {
        let r = quat_to_rotmat(q_est);
        let omega = sanitise3(omega_body);
        let r_d = thrust_axis_ned(sanitise3(a_cmd_ned))
            .and_then(|b3| desired_attitude(b3, yaw_d));
        match r_d {
            Some(r_d) => sanitise3(self.moment(&r, omega, &r_d)),
            None => {
                let g = &self.gains;
                sanitise3([
                    -g.k_omega[0] * omega[0],
                    -g.k_omega[1] * omega[1],
                    -g.k_omega[2] * omega[2],
                ])
            }
        }
    }
}

#[inline]
fn sanitise3(v: Vec3) -> Vec3 {
    [
        if v[0].is_finite() { v[0] } else { 0.0 },
        if v[1].is_finite() { v[1] } else { 0.0 },
        if v[2].is_finite() { v[2] } else { 0.0 },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rot_z(yaw: f32) -> Mat3 {
        let (c, s) = (libm::cosf(yaw), libm::sinf(yaw));
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    }
    fn rot_x(roll: f32) -> Mat3 {
        let (c, s) = (libm::cosf(roll), libm::sinf(roll));
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    }

    /// At hover (level thrust axis, zero yaw) the desired attitude is
    /// identity, and a level body has zero attitude error.
    #[test]
    fn hover_desired_attitude_is_identity() {
        let r_d = desired_attitude([0.0, 0.0, 1.0], 0.0).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((r_d[i][j] - want).abs() < 1e-5, "R_d[{i}][{j}]={}", r_d[i][j]);
            }
        }
        let e = GeoAtt::attitude_error(&identity3(), &r_d);
        assert!(norm(e) < 1e-5, "zero error at hover, got {e:?}");
        assert!(GeoAtt::psi(&identity3(), &r_d) < 1e-6);
    }

    /// The thrust axis tilts toward a horizontal acceleration command:
    /// commanding +north accel tilts the desired body-down axis northward.
    #[test]
    fn thrust_axis_tilts_toward_accel() {
        let b3 = thrust_axis_ned([2.0, 0.0, 0.0]).unwrap();
        assert!(b3[0] < 0.0, "north accel ⇒ body-down tilts -north: {b3:?}");
        assert!(b3[2] > 0.0, "still mostly down");
    }

    /// THE decoupling property (the whole point of v0.23): a pure YAW
    /// difference produces an attitude error with ONLY a yaw (body-z)
    /// component — no spurious roll/pitch. And a pure tilt produces no
    /// yaw component. Euler setpoints fail exactly this.
    #[test]
    fn attitude_error_does_not_cross_couple() {
        // Pure yaw error: R = I, R_d = Rz(20°).
        let r_d_yaw = rot_z(20f32.to_radians());
        let e_yaw = GeoAtt::attitude_error(&identity3(), &r_d_yaw);
        assert!(e_yaw[0].abs() < 1e-4 && e_yaw[1].abs() < 1e-4, "yaw err leaked to roll/pitch: {e_yaw:?}");
        assert!(e_yaw[2].abs() > 0.05, "yaw err should be present: {e_yaw:?}");

        // Pure roll error at a non-zero heading: R = Rz(90°), R_d = Rz(90°)Rx(15°).
        let r = rot_z(90f32.to_radians());
        let r_d = matmul3(&r, &rot_x(15f32.to_radians()));
        let e = GeoAtt::attitude_error(&r, &r_d);
        assert!(e[2].abs() < 1e-3, "tilt at heading=90° leaked into YAW (the Euler bug): {e:?}");
        assert!(e[0].abs() > 0.05, "roll error should be present: {e:?}");
    }

    /// Closed-loop stability: simulate a rigid body (diagonal J) driven by
    /// the geometric moment from a tilted initial attitude — the Lyapunov
    /// Ψ must strictly decrease to ~0 (almost-global exp. stability).
    #[test]
    fn geometric_control_drives_psi_to_zero() {
        let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
        let r_d = identity3(); // target level
        // Start tilted 40° in roll, at rest.
        let mut r = rot_x(40f32.to_radians());
        let mut omega = [0.0f32; 3];
        let j = GeoGains::FALCON_QUAD.j;
        let dt = 0.002f32;
        let psi0 = GeoAtt::psi(&r, &r_d);
        for _ in 0..3000 {
            let m = ctrl.moment(&r, omega, &r_d);
            // ω̇ = J⁻¹ (M − ω × Jω)
            let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
            let gyro = cross(omega, jo);
            for i in 0..3 {
                omega[i] += dt * (m[i] - gyro[i]) / j[i];
            }
            // R ← R · exp(ω̂ dt) via first-order + re-orthonormalisation.
            r = integrate_rotation(&r, omega, dt);
        }
        let psi1 = GeoAtt::psi(&r, &r_d);
        assert!(psi1 < psi0, "Ψ must decrease: {psi0} -> {psi1}");
        assert!(psi1 < 1e-2, "should converge to level: Ψ={psi1}");
        assert!(norm(omega) < 0.05, "should settle: |ω|={}", norm(omega));
    }

    /// v0.27 differential-flatness feedforward: the analytic ω_ff from
    /// (a, jerk) matches the finite-difference of the desired attitude R_d
    /// along a trajectory — `ω_ff ≈ vee(R_dᵀ Ṙ_d)`. This is the oracle that
    /// pins the feedforward's SIGN (the class of bug behind the yaw saga).
    #[test]
    fn flatness_feedforward_matches_attitude_derivative() {
        let yaw = 0.0f32;
        let dt = 1e-4f32;
        let accel = |t: f32| [0.5 * libm::sinf(t), 0.3 * libm::cosf(0.7 * t), 0.2 * t];
        let jerk = |t: f32| [0.5 * libm::cosf(t), -0.21 * libm::sinf(0.7 * t), 0.2];
        for k in 1..20 {
            let t = k as f32 * 0.15;
            let (a, jc) = (accel(t), jerk(t));
            let rd = desired_attitude(thrust_axis_ned(a).unwrap(), yaw).unwrap();
            let rdb = desired_attitude(thrust_axis_ned(accel(t + dt)).unwrap(), yaw).unwrap();
            let mut rdot = [[0.0f32; 3]; 3];
            for i in 0..3 {
                for jj in 0..3 {
                    rdot[i][jj] = (rdb[i][jj] - rd[i][jj]) / dt;
                }
            }
            let w_fd = vee(&matmul3(&transpose3(&rd), &rdot));
            let w_ff = flatness_omega_ff(a, jc, yaw, 0.0).unwrap();
            for i in 0..3 {
                assert!(
                    (w_ff[i] - w_fd[i]).abs() < 0.05,
                    "axis {i} at t={t}: ff {} vs fd {}",
                    w_ff[i], w_fd[i]
                );
            }
        }
    }

    /// Reduced-attitude (S²) error: zero at alignment, nonzero roll/pitch
    /// under tilt, and EXACTLY zero yaw component (yaw is relinquished).
    #[test]
    fn reduced_error_zero_at_alignment_no_yaw() {
        let e0 = GeoAtt::attitude_error_reduced(&identity3(), [0.0, 0.0, 1.0]);
        assert!(e0[0].abs() < 1e-6 && e0[1].abs() < 1e-6 && e0[2] == 0.0,
            "zero at alignment: {e0:?}");
        let e1 = GeoAtt::attitude_error_reduced(&rot_x(0.3), [0.0, 0.0, 1.0]);
        assert_eq!(e1[2], 0.0, "no yaw component: {e1:?}");
        assert!(e1[0].abs() + e1[1].abs() > 0.1, "tilt produces error: {e1:?}");
    }

    /// v0.26 reduced-attitude control: a tilted body under `moment_reduced`
    /// drives its THRUST AXIS (body-down) to the target while commanding NO
    /// yaw torque — the rotor-loss control law (the residual spin is left
    /// free). Mirrors the full-attitude convergence test but on S².
    #[test]
    fn reduced_attitude_drives_thrust_axis_to_target() {
        let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
        let b3_d = [0.0f32, 0.0, 1.0]; // level thrust axis (hover)
        let mut r = rot_x(40f32.to_radians()); // tilted 40° in roll
        let mut omega = [0.0f32; 3];
        let j = GeoGains::FALCON_QUAD.j;
        let dt = 0.002f32;
        let tilt0 = libm::acosf(r[2][2].clamp(-1.0, 1.0));
        for _ in 0..3000 {
            let m = ctrl.moment_reduced(&r, omega, b3_d);
            assert_eq!(m[2], 0.0, "yaw torque must be relinquished (0)");
            let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
            let gyro = cross(omega, jo);
            for i in 0..3 {
                omega[i] += dt * (m[i] - gyro[i]) / j[i];
            }
            r = integrate_rotation(&r, omega, dt);
        }
        let tilt1 = libm::acosf(r[2][2].clamp(-1.0, 1.0));
        assert!(tilt1 < tilt0, "thrust-axis tilt must decrease: {tilt0} -> {tilt1}");
        assert!(tilt1 < 0.05, "thrust axis should align to target: tilt={tilt1}");
    }

    /// Integrate R by the body rate over dt, with Gram-Schmidt
    /// re-orthonormalisation to keep R ∈ SO(3) over many steps.
    fn integrate_rotation(r: &Mat3, omega: Vec3, dt: f32) -> Mat3 {
        // body-frame increment: R ← R (I + ω̂ dt)
        let wdt = [omega[0] * dt, omega[1] * dt, omega[2] * dt];
        let incr = [
            [1.0, -wdt[2], wdt[1]],
            [wdt[2], 1.0, -wdt[0]],
            [-wdt[1], wdt[0], 1.0],
        ];
        let m = matmul3(r, &incr);
        orthonormalize(&m)
    }

    fn orthonormalize(m: &Mat3) -> Mat3 {
        // Columns of m.
        let c0 = [m[0][0], m[1][0], m[2][0]];
        let c1 = [m[0][1], m[1][1], m[2][1]];
        let e0 = normalize(c0).unwrap();
        let p1 = [c1[0] - dot(e0, c1) * e0[0], c1[1] - dot(e0, c1) * e0[1], c1[2] - dot(e0, c1) * e0[2]];
        let e1 = normalize(p1).unwrap();
        let e2 = cross(e0, e1);
        [[e0[0], e1[0], e2[0]], [e0[1], e1[1], e2[1]], [e0[2], e1[2], e2[2]]]
    }

    /// v0.23 — RUNNABLE Lyapunov certificate (the oracle backing the Lean
    /// algebraic proof in proofs/lean/GeometricLyapunov.lean). For the
    /// canonical SCALAR-gain Lee controller, the closed-loop Lyapunov
    /// function V = ½ Ω·JΩ + k_R Ψ satisfies **V̇ = −k_Ω‖Ω‖² ≤ 0** on
    /// Ψ < 2. The decrease rests on two facts we verify NUMERICALLY over a
    /// grid (attitudes angle<180° ⇒ Ψ<2, several body rates), the rest
    /// being exact algebra:
    ///   FACT 1 (the moment realises the closed loop): M − Ω×JΩ = −k_R e_R
    ///          − k_Ω Ω. A sign bug in moment() breaks this — the crux.
    ///   FACT 2 (the Ψ-derivative identity): d/dt Ψ = ½ e_R·Ω, checked by
    ///          finite-difference of the real psi() along the flow. (The ½
    ///          is because this crate's e_R = (R_dᵀR−RᵀR_d)^∨ = 2× Lee's;
    ///          the potential coefficient is correspondingly 2 k_R.)
    /// Then V̇ = Ω·(M−Ω×JΩ) + 2k_R Ψ̇ = Ω·(−k_R e_R − k_Ω Ω) + k_R(e_R·Ω)
    ///        = −k_Ω‖Ω‖² ≤ 0  — asserted exactly from the real moment/e_R.
    #[test]
    fn lyapunov_decrease_certificate() {
        let kr = 8.0f32;
        let kw = 2.0f32;
        let j = [0.0217f32, 0.0217, 0.04];
        let ctrl = GeoAtt::new(GeoGains { k_r: [kr; 3], k_omega: [kw; 3], j });
        let r_d = identity3();
        let dt = 1e-4f32;

        let angles = [0.2f32, 0.8, 1.5, 2.4, 2.9]; // up to ~166° (Ψ<2)
        let omegas = [
            [1.0f32, 0.0, 0.0], [0.0, 2.0, -1.0],
            [3.0, -2.0, 4.0], [-5.0, 1.0, 2.0],
        ];
        let mut checked = 0;
        for &ax in &angles {
            for &az in &angles {
                let r = matmul3(&rot_x(ax), &rot_z(az));
                let psi = GeoAtt::psi(&r, &r_d);
                assert!(psi < 2.0, "grid point outside Ψ<2: {psi}");
                let e_r = GeoAtt::attitude_error(&r, &r_d);
                for &omega in &omegas {
                    // FACT 1: M − Ω×JΩ == −k_R e_R − k_Ω Ω (exact, f32).
                    let m = ctrl.moment(&r, omega, &r_d);
                    let jo = [j[0]*omega[0], j[1]*omega[1], j[2]*omega[2]];
                    let j_omega_dot = {
                        let g = cross(omega, jo);
                        [m[0]-g[0], m[1]-g[1], m[2]-g[2]]
                    };
                    for i in 0..3 {
                        let want = -kr * e_r[i] - kw * omega[i];
                        assert!((j_omega_dot[i] - want).abs() < 1e-3,
                            "FACT1 axis {i}: JΩ̇={} vs {want}", j_omega_dot[i]);
                    }
                    // FACT 2: Ψ̇ == ½ e_R·Ω (finite-diff of real psi).
                    let r1 = integrate_rotation(&r, omega, dt);
                    let psi_dot = (GeoAtt::psi(&r1, &r_d) - psi) / dt;
                    let half_e_dot_w = 0.5 * dot(e_r, omega);
                    let tol2 = 0.02 + 0.03 * half_e_dot_w.abs();
                    assert!((psi_dot - half_e_dot_w).abs() <= tol2,
                        "FACT2: Ψ̇={psi_dot} vs ½e_R·Ω={half_e_dot_w}; Ψ={psi}");
                    // Assembled V̇ from the REAL moment + e_R.
                    let vdot = dot(omega, j_omega_dot) + kr * dot(e_r, omega);
                    let expected = -kw * dot(omega, omega);
                    assert!(vdot <= 0.0,
                        "V̇ must be ≤ 0: {vdot} at Ψ={psi}, ω={omega:?}");
                    assert!((vdot - expected).abs() < 1e-2,
                        "V̇ ({vdot}) must equal −k_Ω‖Ω‖² ({expected}); Ψ={psi}");
                    checked += 1;
                }
            }
        }
        assert!(checked >= 80, "grid too small: {checked}");
    }

    proptest::proptest! {
        /// The controller is total: finite torque for any finite inputs,
        /// including degenerate accel commands and non-unit quaternions.
        #[test]
        fn geo_tick_is_total(
            qw in -2.0f32..2.0, qx in -2.0f32..2.0, qy in -2.0f32..2.0, qz in -2.0f32..2.0,
            wx in -50.0f32..50.0, wy in -50.0f32..50.0, wz in -50.0f32..50.0,
            ax in -100.0f32..100.0, ay in -100.0f32..100.0, az in -100.0f32..100.0,
            yaw in -10.0f32..10.0,
        ) {
            let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
            let t = ctrl.tick([qw, qx, qy, qz], [wx, wy, wz], [ax, ay, az], yaw);
            proptest::prop_assert!(t.iter().all(|c| c.is_finite()));
        }
    }
}
