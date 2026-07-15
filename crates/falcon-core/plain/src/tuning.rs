//! IN-FLIGHT TUNING (MAVLINK-P06, v1.119) — the store→core application link.
//!
//! falcon-param (v1.96) bound the GCS wire to the verified relay-param store
//! (PARAM_SET → bounded write), and PARAM-P03 (v1.117) made the store
//! persistent — but nothing ever APPLIED a stored value to the running
//! [`FlightCore`]. This module is that missing link: a small named schema of
//! tunable knobs and an `apply` that pushes the store's current values into
//! the core. Called once per control cycle by the integration, a GCS
//! PARAM_SET lands in the loop ON THE NEXT TICK — bounded twice over (the
//! K01-proven store write, then the setters' own clamps).

use crate::FlightCore;
use relay_param::{param_id, ParamDef, ParamStore};

/// Register falcon's tunable knobs (schema bounds + current-behavior
/// defaults). Idempotent per store; returns false if the store lacks room.
pub fn register_tuning<const N: usize>(store: &mut ParamStore<N>) -> bool {
    let defs = [
        // Altitude P-I-D (the gz-reconciled defaults are per-plant; these
        // are the analytic-plant baseline the core constructs with).
        ParamDef { id: param_id("MC_ALT_P"), min: 0.01, max: 1.0, default: 0.05 },
        ParamDef { id: param_id("MC_ALT_D"), min: 0.0, max: 3.0, default: 0.30 },
        ParamDef { id: param_id("MC_ALT_I"), min: 0.0, max: 0.2, default: 0.0 },
        // Hover-thrust feedforward (per-airframe).
        ParamDef { id: param_id("MC_HOVER_THR"), min: 0.2, max: 0.8, default: 0.5 },
        // Landing descent rate (m/s, NED +down).
        ParamDef { id: param_id("MC_LAND_VZ"), min: 0.2, max: 1.5, default: 0.5 },
    ];
    for d in defs {
        if !store.register(d) {
            return false;
        }
    }
    true
}

/// Push the store's current tuning values into the core. Call once per
/// control cycle (cheap: five bounded reads) — a PARAM_SET applied to the
/// store is live in the loop on the next tick.
pub fn apply_tuning<const N: usize>(store: &ParamStore<N>, core: &mut FlightCore) {
    let g = |name: &str| store.get(&param_id(name));
    if let (Some(kp), Some(kd)) = (g("MC_ALT_P"), g("MC_ALT_D")) {
        core.set_altitude_gains(kp, kd);
    }
    if let Some(ki) = g("MC_ALT_I") {
        core.set_altitude_integral_gain(ki);
    }
    if let Some(h) = g("MC_HOVER_THR") {
        core.set_hover_thrust_core(h);
    }
    if let Some(vz) = g("MC_LAND_VZ") {
        core.set_landing_descent(vz);
    }
}
