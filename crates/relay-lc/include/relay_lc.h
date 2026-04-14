/*
 * Relay Limit Checker — C API
 *
 * Drop-in replacement for NASA cFS LC watchpoint evaluation (lc_watch.c).
 * Behind this API: a formally verified Rust engine with 8 Verus SMT properties,
 * 8 unit tests, and 4 Kani bounded model checking harnesses.
 *
 * Usage in a cFS build:
 *   1. Link this library instead of lc_watch.o
 *   2. Call relay_lc_init() in LC_AppInit()
 *   3. Call relay_lc_add_watchpoint() when loading WDT entries
 *   4. Call relay_lc_evaluate() in LC_CheckWatchpoints()
 *
 * Scaling: f64 threshold/value → i64 fixed-point (×1000) internally.
 * Precision: 0.001 units. Range: ±9.2×10^15.
 */

#ifndef RELAY_LC_H
#define RELAY_LC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RELAY_LC_MAX_VIOLATIONS  32
#define RELAY_LC_MAX_WATCHPOINTS 128

/* Comparison operators — matches cFS LC_OPER_* from lc_tbldefs.h */
#define RELAY_LC_LT 1  /* Less Than         */
#define RELAY_LC_LE 2  /* Less or Equal     */
#define RELAY_LC_NE 3  /* Not Equal         */
#define RELAY_LC_GT 4  /* Greater Than      */
#define RELAY_LC_GE 5  /* Greater or Equal  */
#define RELAY_LC_EQ 6  /* Equal             */

/* Evaluation result — filled by relay_lc_evaluate() */
typedef struct {
    uint32_t violation_count;
    uint32_t violated_ids[RELAY_LC_MAX_VIOLATIONS];
    double   measured_values[RELAY_LC_MAX_VIOLATIONS];
    double   thresholds[RELAY_LC_MAX_VIOLATIONS];
} relay_lc_eval_result_t;

/*
 * Initialize the limit checker. Call once at app startup.
 * Returns: 0 on success.
 */
int32_t relay_lc_init(void);

/*
 * Add a watchpoint to the table.
 *
 * sensor_id:   Unique identifier for this data point (MsgId + offset)
 * oper:        Comparison operator (RELAY_LC_LT through RELAY_LC_EQ)
 * threshold:   Threshold value (f64, converted to fixed-point internally)
 * persistence: Consecutive violations required before triggering (0 → 1)
 *
 * Returns: 0 on success, -1 if table full, -2 if invalid operator.
 */
int32_t relay_lc_add_watchpoint(uint32_t sensor_id, uint32_t oper,
                                 double threshold, uint32_t persistence);

/*
 * Evaluate a sensor reading against all watchpoints.
 *
 * This is the hot path. Verified properties:
 *   - Output bounded: violation_count ≤ 32
 *   - Comparison total: all 6 operators correct for any input
 *   - Persistence correct: counter increments on violation, resets on pass
 *   - Disabled watchpoints never fire
 *
 * sensor_id: Which sensor produced this reading
 * value:     Measured value (f64)
 * result:    Output buffer (may be NULL if only count is needed)
 *
 * Returns: Number of violations.
 */
uint32_t relay_lc_evaluate(uint32_t sensor_id, double value,
                            relay_lc_eval_result_t *result);

/* Get current number of registered watchpoints. */
uint32_t relay_lc_watchpoint_count(void);

/* Reset: clear all watchpoints. Returns 0. */
int32_t relay_lc_reset(void);

/* Maximum watchpoints this build supports. */
uint32_t relay_lc_max_watchpoints(void);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_LC_H */
