//! Relay ModExtra — extended flight-mode setpoint generators.
//!
//! Rounds out the autonomy modes to PX4 breadth (the existing relay-fsm/relay-pos
//! handle takeoff/loiter/mission/RTL/land). Each function generates the position/
//! yaw SETPOINT for one mode; the verified cascade flies it. All are total and
//! NaN-safe — a garbage input yields a safe, finite setpoint, never a panic or a
//! NaN flown into the controller.
//!
//!  * **`poi_yaw`** — guided point-of-interest: the heading to face a point of
//!    interest from the vehicle (PX4 Guided yaw / ROI).
//!  * **`follow_setpoint`** — follow-me: a position offset behind/around a moving
//!    target's pose.
//!  * **`land_in_place`** — descend at the current horizontal position (the
//!    alternative to RTL when returning home is unwise).
//!  * **`rtl_setpoint`** — return to launch with an altitude override (climb/
//!    descend to a safe return altitude rather than the current one).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// NED position.
pub type Ned = [f32; 3];

#[inline]
fn fin(x: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        0.0
    }
}

#[inline]
fn fin3(p: Ned) -> Ned {
    [fin(p[0]), fin(p[1]), fin(p[2])]
}

/// Guided point-of-interest yaw: the NED heading (rad, atan2(east, north)) to
/// face the POI from the vehicle. If the vehicle is essentially ON the POI
/// (degenerate horizontal separation) the previous `hold_yaw` is returned so the
/// heading doesn't spin. Total / NaN-safe.
pub fn poi_yaw(vehicle: Ned, poi: Ned, hold_yaw: f32) -> f32 {
    let v = fin3(vehicle);
    let p = fin3(poi);
    let dn = p[0] - v[0];
    let de = p[1] - v[1];
    if dn * dn + de * de < 1e-6 {
        return if hold_yaw.is_finite() { hold_yaw } else { 0.0 };
    }
    relay_math::atan2f(de, dn)
}

/// Follow-me setpoint: the target's pose plus a fixed NED offset (e.g. behind and
/// above). Total / NaN-safe.
pub fn follow_setpoint(target: Ned, offset: Ned) -> Ned {
    let t = fin3(target);
    let o = fin3(offset);
    [t[0] + o[0], t[1] + o[1], t[2] + o[2]]
}

/// Land-in-place setpoint: hold the current horizontal position, command a
/// descent at `descent_rate` (m/s, positive down) over `dt` (s), never commanding
/// ABOVE the current altitude (z only increases / down is +z). Total / NaN-safe.
pub fn land_in_place(current: Ned, descent_rate: f32, dt: f32) -> Ned {
    let c = fin3(current);
    let rate = if descent_rate.is_finite() && descent_rate > 0.0 { descent_rate } else { 0.0 };
    let d = if dt.is_finite() && dt > 0.0 { dt.min(1.0) } else { 0.0 };
    [c[0], c[1], c[2] + rate * d] // down is +z → descend
}

/// RTL setpoint with altitude override: return to `home` horizontally but at the
/// safe return altitude `safe_alt_m` (height above home; converted to NED-down
/// `home.z - safe_alt`). Total / NaN-safe; a negative/NaN safe-alt clamps to 0
/// (return at home altitude).
pub fn rtl_setpoint(home: Ned, safe_alt_m: f32) -> Ned {
    let h = fin3(home);
    let alt = if safe_alt_m.is_finite() && safe_alt_m > 0.0 { safe_alt_m } else { 0.0 };
    [h[0], h[1], h[2] - alt] // up = -z in NED
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::FRAC_PI_2;

    #[test]
    fn poi_yaw_faces_the_point() {
        // POI due east of the vehicle ⇒ yaw +π/2.
        let y = poi_yaw([0.0, 0.0, -5.0], [0.0, 10.0, -5.0], 0.0);
        assert!((y - FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn poi_yaw_holds_when_on_top() {
        let y = poi_yaw([1.0, 1.0, -5.0], [1.0, 1.0, -5.0], 0.7);
        assert_eq!(y, 0.7); // degenerate ⇒ hold
    }

    #[test]
    fn follow_applies_offset() {
        let sp = follow_setpoint([10.0, 5.0, -8.0], [-2.0, 0.0, -1.0]);
        assert_eq!(sp, [8.0, 5.0, -9.0]);
    }

    #[test]
    fn land_descends_never_climbs() {
        let sp = land_in_place([3.0, 4.0, -10.0], 0.5, 0.5);
        assert_eq!(sp[0], 3.0);
        assert_eq!(sp[1], 4.0);
        assert!(sp[2] > -10.0); // down is +z → descended (less negative)
    }

    #[test]
    fn rtl_override_sets_safe_altitude() {
        // home at -2 (2 m up), safe alt 30 ⇒ z = -2 - 30 = -32 (32 m up over home).
        let sp = rtl_setpoint([0.0, 0.0, -2.0], 30.0);
        assert_eq!(sp, [0.0, 0.0, -32.0]);
    }

    #[test]
    fn all_nan_safe() {
        let n = f32::NAN;
        assert!(poi_yaw([n, n, n], [n, n, n], n).is_finite());
        assert!(follow_setpoint([n, 1.0, 2.0], [n, n, n]).iter().all(|x| x.is_finite()));
        assert!(land_in_place([n, n, n], n, n).iter().all(|x| x.is_finite()));
        assert!(rtl_setpoint([n, n, n], n).iter().all(|x| x.is_finite()));
    }
}

#[cfg(test)]
mod proptests {
    use super::*;

    proptest::proptest! {
        /// land_in_place never commands ABOVE the current altitude (down is +z, so
        /// the z setpoint is ≥ the current z) and stays finite.
        #[test]
        fn land_only_descends(z in -200.0f32..0.0, rate in -5.0f32..5.0, dt in -1.0f32..2.0) {
            let sp = land_in_place([0.0, 0.0, z], rate, dt);
            proptest::prop_assert!(sp[2] >= z - 1e-3);
            proptest::prop_assert!(sp.iter().all(|x| x.is_finite()));
        }
    }
}
