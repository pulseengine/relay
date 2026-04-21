//! Comparison oracle primitive.
//!
//! Total, deterministic six-operator comparison. The fundamental building
//! block of every threshold check, every limit monitor, every interlock,
//! every diagnostic trouble code.
//!
//! Extracted from relay-lc/src/engine.rs:90-107. The operator enum is a
//! direct mirror of wit/interfaces/relay-common-types/types.wit:117
//! (`comparison-op`) so that WIT codegen lands exactly on this type.
//!
//! Verified properties:
//!   CMP-P01: compare is total — returns a bool for every (value, op, threshold)
//!   CMP-P02: compare is deterministic — same inputs always yield same output
//!   CMP-P03: each branch matches standard mathematical semantics
/// Matches `comparison-op` in wit/interfaces/relay-common-types/types.wit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ComparisonOp {
    LessThan = 0,
    GreaterThan = 1,
    LessOrEqual = 2,
    GreaterOrEqual = 3,
    Equal = 4,
    NotEqual = 5,
}
/// Compare an i64 value against a threshold under a given operator.
///
/// Pure, total, deterministic. No allocation, no panics, no UB.
pub fn compare_i64(value: i64, op: ComparisonOp, threshold: i64) -> bool {
    match op {
        ComparisonOp::LessThan => value < threshold,
        ComparisonOp::GreaterThan => value > threshold,
        ComparisonOp::LessOrEqual => value <= threshold,
        ComparisonOp::GreaterOrEqual => value >= threshold,
        ComparisonOp::Equal => value == threshold,
        ComparisonOp::NotEqual => value != threshold,
    }
}
/// Compare a u64 value against a threshold under a given operator.
pub fn compare_u64(value: u64, op: ComparisonOp, threshold: u64) -> bool {
    match op {
        ComparisonOp::LessThan => value < threshold,
        ComparisonOp::GreaterThan => value > threshold,
        ComparisonOp::LessOrEqual => value <= threshold,
        ComparisonOp::GreaterOrEqual => value >= threshold,
        ComparisonOp::Equal => value == threshold,
        ComparisonOp::NotEqual => value != threshold,
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn less_than() {
        assert!(compare_i64(1, ComparisonOp::LessThan, 2));
        assert!(! compare_i64(2, ComparisonOp::LessThan, 2));
    }
    #[test]
    fn all_ops_total_on_zero_zero() {
        for op in [
            ComparisonOp::LessThan,
            ComparisonOp::GreaterThan,
            ComparisonOp::LessOrEqual,
            ComparisonOp::GreaterOrEqual,
            ComparisonOp::Equal,
            ComparisonOp::NotEqual,
        ] {
            let _ = compare_i64(0, op, 0);
            let _ = compare_u64(0, op, 0);
        }
    }
}
