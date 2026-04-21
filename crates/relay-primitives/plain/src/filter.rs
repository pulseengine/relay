//! Filter primitive — keep/drop decision based on a predicate result.
//!
//! The minimum verified kernel for filter transformers: given a
//! predicate outcome, produce a Keep/Drop decision. The predicate
//! evaluation itself is another primitive (usually `compare`) composed
//! upstream. This separation keeps `filter` generic over any predicate.
//!
//! Used by: telemetry subscription filtering, command-code dispatch,
//! safe-mode gating, out-of-band sample rejection, quality-bit
//! screening, noise-floor thresholding.
//!
//! Verified properties:
//!   FLT-P01: Keep ⟺ predicate_holds == true
//!   FLT-P02: Drop ⟺ predicate_holds == false
/// Filter outcome.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FilterDecision {
    Keep,
    Drop,
}
/// Pure decision: pass the value through iff `predicate_holds`.
pub fn filter_decide(predicate_holds: bool) -> FilterDecision {
    if predicate_holds { FilterDecision::Keep } else { FilterDecision::Drop }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn true_is_keep() {
        assert_eq!(filter_decide(true), FilterDecision::Keep);
    }
    #[test]
    fn false_is_drop() {
        assert_eq!(filter_decide(false), FilterDecision::Drop);
    }
    #[test]
    fn total_deterministic() {
        assert_eq!(filter_decide(true), filter_decide(true));
        assert_eq!(filter_decide(false), filter_decide(false));
        assert_ne!(filter_decide(true), filter_decide(false));
    }
}
