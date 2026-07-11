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

pub mod persist;

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
mod persist_tests {
    use super::persist::*;
    use super::*;

    const LAYOUT: Layout = Layout::new(4);
    const CAP: usize = LAYOUT.required_capacity();
    const VER: u32 = 1;

    fn schema() -> ParamStore<4> {
        let mut s = ParamStore::new();
        s.register(ParamDef { id: param_id("MC_ROLL_P"), min: 0.0, max: 12.0, default: 6.5 });
        s.register(ParamDef { id: param_id("BAT_LOW_V"), min: 10.0, max: 16.8, default: 14.0 });
        s.register(ParamDef { id: param_id("GF_RADIUS"), min: 5.0, max: 500.0, default: 100.0 });
        s
    }

    #[test]
    fn fresh_device_loads_defaults_loudly() {
        let nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        let r = load(&mut s, &nvm, LAYOUT, VER);
        assert_eq!(r.outcome, LoadOutcome::FreshDefaults);
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(6.5));
    }

    #[test]
    fn save_load_roundtrip_reboot() {
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        s.set(&param_id("MC_ROLL_P"), 8.25);
        s.set(&param_id("GF_RADIUS"), 42.0);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();
        // "Reboot": a fresh store built from the same schema.
        let mut s2 = schema();
        let r = load(&mut s2, &nvm, LAYOUT, VER);
        assert_eq!(r.outcome, LoadOutcome::Loaded);
        assert_eq!((r.applied, r.skipped_unknown, r.rejected), (3, 0, 0));
        assert_eq!(s2.get(&param_id("MC_ROLL_P")), Some(8.25));
        assert_eq!(s2.get(&param_id("GF_RADIUS")), Some(42.0));
        assert_eq!(s2.get(&param_id("BAT_LOW_V")), Some(14.0));
    }

    #[test]
    fn double_save_alternates_slots_and_newest_wins() {
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        s.set(&param_id("MC_ROLL_P"), 7.0);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();
        s.set(&param_id("MC_ROLL_P"), 9.0);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();
        let mut s2 = schema();
        assert_eq!(load(&mut s2, &nvm, LAYOUT, VER).outcome, LoadOutcome::Loaded);
        assert_eq!(s2.get(&param_id("MC_ROLL_P")), Some(9.0));
    }

    #[test]
    fn torn_commit_keeps_previous_image() {
        // Commit image #1, then simulate a crash MID-save of image #2: the new
        // slot is half-written but the selector never flips. Load must return
        // image #1 intact — the two-slot protocol's whole point.
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        s.set(&param_id("MC_ROLL_P"), 7.0);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();

        // Torn second save: garbage into the INACTIVE slot region, no flip.
        // (Selector is 0xA5 = slot A active; slot B starts at 16 + slot_len.)
        let slot_b = 16 + LAYOUT.slot_len();
        for i in 0..LAYOUT.slot_len() / 2 {
            nvm.bytes[slot_b + i] = 0xDB;
        }
        let mut s2 = schema();
        let r = load(&mut s2, &nvm, LAYOUT, VER);
        assert_eq!(r.outcome, LoadOutcome::Loaded);
        assert_eq!(s2.get(&param_id("MC_ROLL_P")), Some(7.0));
    }

    #[test]
    fn corruption_sweep_never_yields_out_of_schema() {
        // Flip EVERY byte of a committed image (one at a time): whatever the
        // outcome (Corrupt / rejected records / still-valid), no load may ever
        // leave a stored value outside its schema bounds, and none may panic.
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        s.set(&param_id("MC_ROLL_P"), 11.5);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();
        for i in 0..CAP {
            let mut evil = ArrayNvm::<CAP> { bytes: nvm.bytes };
            evil.bytes[i] ^= 0xFF;
            let mut s2 = schema();
            let _ = load(&mut s2, &evil, LAYOUT, VER);
            for p in 0..s2.len() {
                let (id, v) = s2.nth(p).unwrap();
                let d = s2.def(&id).unwrap();
                assert!(
                    v.is_finite() && v >= d.min && v <= d.max,
                    "byte {i}: value {v} escaped [{}, {}]",
                    d.min,
                    d.max
                );
            }
        }
    }

    #[test]
    fn schema_version_bump_resets_loudly() {
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        let mut s = schema();
        s.set(&param_id("MC_ROLL_P"), 8.0);
        save(&s, &mut nvm, LAYOUT, VER).unwrap();
        let mut s2 = schema();
        let r = load(&mut s2, &nvm, LAYOUT, VER + 1); // firmware upgraded
        assert_eq!(r.outcome, LoadOutcome::SchemaMismatch);
        assert_eq!(s2.get(&param_id("MC_ROLL_P")), Some(6.5)); // defaults, not 8.0
    }

    #[test]
    fn unknown_and_tightened_records_are_counted_not_applied() {
        // Save under a schema with an extra param and a wide bound; load under
        // a schema where that param is gone and the bound tightened. The
        // orphan is skipped_unknown, the now-out-of-bounds value rejected —
        // and BOTH are visible in the report (loud, never silent).
        let mut wide = ParamStore::<4>::new();
        wide.register(ParamDef { id: param_id("MC_ROLL_P"), min: 0.0, max: 50.0, default: 6.5 });
        wide.register(ParamDef { id: param_id("OLD_PARAM"), min: 0.0, max: 1.0, default: 0.5 });
        wide.set(&param_id("MC_ROLL_P"), 40.0); // legal then, illegal later
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        save(&wide, &mut nvm, LAYOUT, VER).unwrap();

        let mut tight = ParamStore::<4>::new();
        tight.register(ParamDef { id: param_id("MC_ROLL_P"), min: 0.0, max: 12.0, default: 6.5 });
        let r = load(&mut tight, &nvm, LAYOUT, VER);
        assert_eq!(r.outcome, LoadOutcome::Loaded);
        assert_eq!((r.applied, r.skipped_unknown, r.rejected), (0, 1, 1));
        assert_eq!(tight.get(&param_id("MC_ROLL_P")), Some(6.5)); // default stands
    }

    #[test]
    fn capacity_too_small_is_an_error_not_a_panic() {
        let mut nvm: ArrayNvm<8> = ArrayNvm::new();
        let s = schema();
        assert_eq!(save(&s, &mut nvm, LAYOUT, VER), Err(SaveError::Capacity));
        let mut s2 = schema();
        assert_eq!(load(&mut s2, &nvm, LAYOUT, VER).outcome, LoadOutcome::FreshDefaults);
    }
}

#[cfg(test)]
mod persist_proptests {
    use super::persist::*;
    use super::*;
    use proptest::prelude::*;

    const LAYOUT: Layout = Layout::new(2);
    const CAP: usize = LAYOUT.required_capacity();

    fn schema() -> ParamStore<2> {
        let mut s = ParamStore::new();
        s.register(ParamDef { id: param_id("P"), min: -3.0, max: 7.0, default: 1.0 });
        s.register(ParamDef { id: param_id("Q"), min: 0.0, max: 100.0, default: 50.0 });
        s
    }

    proptest! {
        /// Random in-schema values survive save → load bit-exactly.
        #[test]
        fn roundtrip_any_in_schema_values(p in -3.0f32..7.0, q in 0.0f32..100.0) {
            let mut s = schema();
            s.set(&param_id("P"), p);
            s.set(&param_id("Q"), q);
            let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
            save(&s, &mut nvm, LAYOUT, 1).unwrap();
            let mut s2 = schema();
            let r = load(&mut s2, &nvm, LAYOUT, 1);
            prop_assert_eq!(r.outcome, LoadOutcome::Loaded);
            prop_assert_eq!(s2.get(&param_id("P")).unwrap().to_bits(), p.to_bits());
            prop_assert_eq!(s2.get(&param_id("Q")).unwrap().to_bits(), q.to_bits());
        }

        /// MULTI-byte random corruption of a committed image never panics the
        /// loader and never yields an out-of-schema stored value (the proptest
        /// companion to the exhaustive single-byte sweep; Kani can't carry
        /// nondet through the CRC — see kani_proofs.rs).
        #[test]
        fn random_corruption_never_escapes_schema(
            positions in prop::collection::vec(0usize..CAP, 1..8),
            values in prop::collection::vec(any::<u8>(), 8),
        ) {
            let mut s = schema();
            s.set(&param_id("P"), 6.5);
            let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
            save(&s, &mut nvm, LAYOUT, 1).unwrap();
            for (i, pos) in positions.iter().enumerate() {
                nvm.bytes[*pos] = values[i % values.len()];
            }
            let mut s2 = schema();
            let _ = load(&mut s2, &nvm, LAYOUT, 1);
            for k in 0..s2.len() {
                let (id, v) = s2.nth(k).unwrap();
                let d = s2.def(&id).unwrap();
                prop_assert!(v.is_finite() && v >= d.min && v <= d.max);
            }
        }
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
