#ifndef DECEL_TAIL_SHAPER_H
#define DECEL_TAIL_SHAPER_H

#include <math.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bounded downward-tail reference shaper for positive, nonzero downward setpoint steps.
 * 
 * Continuous model:
 * For error e = current_eff - target > 0:
 * When e > e_crit = R^2 / (2 * J), reference falls at linear rate R.
 * When e <= e_crit, reference follows the parabolic tail:
 *   e(t) = (max(0, sqrt(e_0) - sqrt(J/2) * t))^2
 * Arrival slope smoothly approaches 0 without crossing target.
 *
 * Constraints & Invariants:
 * - Pure stateless per-tick evaluation: independent of past command timing anchors.
 * - Handles tick crossing the linear/tail boundary consistently via split dt.
 * - Monotonic downward progression: target_rpm <= v_next <= current_v.
 * - Non-crossing: v_next never undershoots target_rpm.
 * - Bounded step: current_v - v_next <= R * dt.
 * - Exact finite-time arrival at target.
 */
static inline float decel_tail_shaper_step(
    float current_v,
    float target_rpm,
    float max_rate_rpm_s,
    float max_jerk_rpm_s2,
    float dt_s
) {
    // Numeric and sanity guards
    if (!isfinite(current_v) || !isfinite(target_rpm) ||
        !isfinite(max_rate_rpm_s) || !isfinite(max_jerk_rpm_s2) || !isfinite(dt_s)) {
        return isfinite(target_rpm) ? target_rpm : 0.0f;
    }
    if (dt_s <= 0.0f || max_rate_rpm_s <= 0.0f || max_jerk_rpm_s2 <= 0.0f) {
        return current_v;
    }

    // Shaping applies ONLY to downward steps toward positive targets (current_v > target > 0)
    // Non-downward, target=0, or already reached targets retain standard linear/immediate response
    if (current_v <= target_rpm) {
        float max_step = max_rate_rpm_s * dt_s;
        float v_up = current_v + max_step;
        return (v_up > target_rpm) ? target_rpm : v_up;
    }
    if (target_rpm <= 0.0f) {
        float max_step = max_rate_rpm_s * dt_s;
        float v_down = current_v - max_step;
        return (v_down < target_rpm) ? target_rpm : v_down;
    }

    float e = current_v - target_rpm;
    if (e <= 1e-6f) {
        return target_rpm;
    }

    // Parabolic transition boundary
    float R = max_rate_rpm_s;
    float J = max_jerk_rpm_s2;
    float e_crit = (R * R) / (2.0f * J);
    float sqrt_half_J = sqrtf(0.5f * J);

    float e_next = 0.0f;

    if (e > e_crit) {
        float max_step = R * dt_s;
        if (e - max_step >= e_crit) {
            // Entire tick is in the linear descent region
            e_next = e - max_step;
        } else {
            // Tick crosses the linear/tail boundary: split dt
            float dt_linear = (e - e_crit) / R;
            float dt_tail = dt_s - dt_linear;
            if (dt_tail < 0.0f) dt_tail = 0.0f;

            float term = sqrtf(e_crit) - (sqrt_half_J * dt_tail);
            if (term <= 0.0f) {
                e_next = 0.0f;
            } else {
                e_next = term * term;
            }
        }
    } else {
        // Entire tick is inside the parabolic tail region
        float term = sqrtf(e) - (sqrt_half_J * dt_s);
        if (term <= 0.0f) {
            e_next = 0.0f;
        } else {
            e_next = term * term;
        }
    }

    // Safety and monotonicity clamps
    if (e_next < 0.0f || !isfinite(e_next)) {
        e_next = 0.0f;
    }
    if (e_next > e) {
        e_next = e;
    }

    float v_next = target_rpm + e_next;

    // Enforce hard bounds
    if (v_next < target_rpm) {
        v_next = target_rpm;
    }
    if (v_next > current_v) {
        v_next = current_v;
    }

    return v_next;
}

#ifdef __cplusplus
}
#endif

#endif // DECEL_TAIL_SHAPER_H
