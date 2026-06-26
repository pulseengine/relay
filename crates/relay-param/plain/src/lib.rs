//! Relay Param — a typed parameter store with schema-bounded writes.
//!
//! The safety core of a PX4-style MAVLink parameter system: a ground station can
//! list, read, and (validity-gated) write the vehicle's parameters — control
//! gains, failsafe thresholds, geofence radius, etc. The job this crate verifies
//! is the one that matters for safety:
//!
//!   **a parameter write outside the parameter's schema bounds is REJECTED and
//!   leaves the stored value unchanged.**
//!
//! So a GCS (or a corrupted PARAM_SET frame) can never push a gain or threshold
//! to an unsafe value. The MAVLink PARAM_REQUEST_LIST / PARAM_REQUEST_READ /
//! PARAM_SET / PARAM_VALUE wire framing lives in relay-mavlink and drives this
//! store; the typed schema + the bounded set are here.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`. Fixed-capacity, static storage.

#![no_std]
#![forbid(unsafe_code)]

/// A 16-byte MAVLink parameter id (NUL-padded), as on the wire.
pub type ParamId = [u8; 16];

/// The schema for one parameter: its id, allowed range, and default.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamDef {
    /// Parameter id (16 bytes, NUL-padded).
    pub id: ParamId,
    /// Inclusive minimum.
    pub min: f32,
    /// Inclusive maximum.
    pub max: f32,
    /// Default (used as the initial value; assumed within [min, max]).
    pub default: f32,
}

/// The result of a parameter write.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetResult {
    /// The value was within schema bounds and is now stored.
    Applied,
    /// The value was outside [min, max] (or NaN) — REJECTED, value unchanged.
    OutOfRange,
    /// No parameter with that id exists.
    Unknown,
}

#[derive(Clone, Copy)]
struct Param {
    def: ParamDef,
    value: f32,
}

/// A fixed-capacity parameter store (`N` parameters max).
pub struct ParamStore<const N: usize> {
    params: [Param; N],
    count: usize,
}

impl<const N: usize> Default for ParamStore<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Make a `ParamId` from a string, NUL-padded / truncated to 16 bytes.
pub fn param_id(name: &str) -> ParamId {
    let mut id = [0u8; 16];
    let b = name.as_bytes();
    let n = if b.len() < 16 { b.len() } else { 16 };
    id[..n].copy_from_slice(&b[..n]);
    id
}

impl<const N: usize> ParamStore<N> {
    /// An empty store.
    pub fn new() -> Self {
        let blank = Param {
            def: ParamDef { id: [0; 16], min: 0.0, max: 0.0, default: 0.0 },
            value: 0.0,
        };
        ParamStore { params: [blank; N], count: 0 }
    }

    /// Register a parameter (value initialised to its default). Returns false if
    /// the store is full. The default is clamped into [min, max] defensively.
    pub fn register(&mut self, def: ParamDef) -> bool {
        if self.count >= N {
            return false;
        }
        let value = clamp(def.default, def.min, def.max);
        self.params[self.count] = Param { def, value };
        self.count += 1;
        true
    }

    /// Number of registered parameters.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the store has no parameters.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    fn index_of(&self, id: &ParamId) -> Option<usize> {
        self.params[..self.count].iter().position(|p| &p.def.id == id)
    }

    /// The current value of a parameter (for PARAM_VALUE).
    pub fn get(&self, id: &ParamId) -> Option<f32> {
        self.index_of(id).map(|i| self.params[i].value)
    }

    /// The schema for a parameter.
    pub fn def(&self, id: &ParamId) -> Option<ParamDef> {
        self.index_of(id).map(|i| self.params[i].def)
    }

    /// Apply a parameter write (PARAM_SET). The value is stored ONLY if the
    /// parameter exists AND the value is within [min, max] (and finite);
    /// otherwise the stored value is left unchanged.
    pub fn set(&mut self, id: &ParamId, value: f32) -> SetResult {
        let i = match self.index_of(id) {
            Some(i) => i,
            None => return SetResult::Unknown,
        };
        let d = self.params[i].def;
        if value.is_finite() && value >= d.min && value <= d.max {
            self.params[i].value = value;
            SetResult::Applied
        } else {
            SetResult::OutOfRange
        }
    }

    /// The i-th parameter's (id, value), for PARAM_REQUEST_LIST iteration.
    pub fn nth(&self, i: usize) -> Option<(ParamId, f32)> {
        if i < self.count {
            Some((self.params[i].def.id, self.params[i].value))
        } else {
            None
        }
    }
}

#[inline]
fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if !x.is_finite() || x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn store() -> ParamStore<4> {
        let mut s = ParamStore::new();
        s.register(ParamDef { id: param_id("MC_ROLL_P"), min: 0.0, max: 12.0, default: 6.5 });
        s.register(ParamDef { id: param_id("BAT_LOW_V"), min: 10.0, max: 16.8, default: 14.0 });
        s
    }

    #[test]
    fn register_initialises_to_default() {
        let s = store();
        assert_eq!(s.len(), 2);
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(6.5));
    }

    #[test]
    fn in_range_write_applies() {
        let mut s = store();
        assert_eq!(s.set(&param_id("MC_ROLL_P"), 8.0), SetResult::Applied);
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(8.0));
    }

    #[test]
    fn out_of_range_write_rejected_value_unchanged() {
        let mut s = store();
        assert_eq!(s.set(&param_id("MC_ROLL_P"), 99.0), SetResult::OutOfRange);
        assert_eq!(s.set(&param_id("MC_ROLL_P"), -1.0), SetResult::OutOfRange);
        assert_eq!(s.set(&param_id("MC_ROLL_P"), f32::NAN), SetResult::OutOfRange);
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(6.5)); // unchanged
    }

    #[test]
    fn unknown_param_rejected() {
        let mut s = store();
        assert_eq!(s.set(&param_id("NOPE"), 1.0), SetResult::Unknown);
    }

    #[test]
    fn boundary_values_accepted() {
        let mut s = store();
        assert_eq!(s.set(&param_id("BAT_LOW_V"), 10.0), SetResult::Applied); // min
        assert_eq!(s.set(&param_id("BAT_LOW_V"), 16.8), SetResult::Applied); // max
    }

    #[test]
    fn full_store_register_fails() {
        let mut s: ParamStore<1> = ParamStore::new();
        assert!(s.register(ParamDef { id: param_id("A"), min: 0.0, max: 1.0, default: 0.5 }));
        assert!(!s.register(ParamDef { id: param_id("B"), min: 0.0, max: 1.0, default: 0.5 }));
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For any write value, the stored value always stays within the
        /// parameter's schema bounds (an out-of-range write never lands).
        #[test]
        fn stored_value_always_in_bounds(v in -1000.0f32..1000.0) {
            let mut s: ParamStore<2> = ParamStore::new();
            s.register(ParamDef { id: param_id("P"), min: 2.0, max: 8.0, default: 5.0 });
            let id = param_id("P");
            s.set(&id, v);
            let stored = s.get(&id).unwrap();
            prop_assert!((2.0..=8.0).contains(&stored));
        }
    }
}
