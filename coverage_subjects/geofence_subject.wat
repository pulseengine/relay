;; Coverage subject mirroring relay-lc::engine::Geofence::check's decision
;; shape, hand-written WAT to stay WASI-free.
;;
;; The real cascade core module (//:falcon-cascade-fused) imports
;; wasi:io@0.2.6 for Rust's panic/abort glue; witness's embedded
;; runtime is WASI-free, so the cascade target stays `tags=["manual"]`
;; until either witness ships a WASI runtime or we land a WASI-free
;; build path for the cascade. See FV-FALCON-COV-001.
;;
;; This subject keeps the decision shape the Geofence enforces:
;;
;;   if latched           -> 0  (already-latched branch)
;;   else if outside box  -> 1  (rising-edge / latch transition)
;;   else                 -> 0  (in-fence)
;;
;; Three branches × the standard MC/DC unique-cause matrix is what
;; witness reports against — the LCOV file proves the pipeline runs.

(module
  ;; check(latched: i32, n: i32, e: i32, lo: i32, hi: i32) -> i32 (0 or 1)
  (func $check
    (param $latched i32) (param $n i32) (param $e i32) (param $lo i32) (param $hi i32)
    (result i32)
    ;; if already latched, return 0
    local.get $latched
    (if (result i32)
      (then (i32.const 0))
      (else
        ;; outside = (n < lo) | (n > hi) | (e < lo) | (e > hi)
        local.get $n local.get $lo i32.lt_s
        local.get $n local.get $hi i32.gt_s
        i32.or
        local.get $e local.get $lo i32.lt_s
        i32.or
        local.get $e local.get $hi i32.gt_s
        i32.or
        (if (result i32)
          (then (i32.const 1))   ;; transition to latched
          (else (i32.const 0)))))) ;; in-fence
  ;; no-arg entry point witness invokes
  (func (export "run") (result i32)
    ;; inside box: 100 in [-1000, 1000] each axis, not latched
    (call $check (i32.const 0) (i32.const 100) (i32.const 100) (i32.const -1000) (i32.const 1000)))
  (memory (export "memory") 1))
