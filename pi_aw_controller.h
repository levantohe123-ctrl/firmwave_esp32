#ifndef PI_AW_CONTROLLER_H
#define PI_AW_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mode selection for Anti-Windup A/B testing
typedef enum {
    AW_MODE_BASELINE_CLAMP = 0, // Baseline: Amplitude clamping AW with downstream slew filter
    AW_MODE_TRACKING = 1        // Experimental: Actual-applied-output tracking anti-windup
} aw_mode_t;

// Pure stateful Jerk-Limited reference profile generator
typedef struct {
    float max_slope;            // Max acceleration slope magnitude (RPM/s, e.g. 100.0f)
    float max_jerk;             // Max jerk magnitude (RPM/s^2, e.g. 500.0f)
    float dt_s;                 // Control period (s, e.g. 0.020f)
    float min_rpm;              // Speed floor (RPM, e.g. 0.0f)
    float max_rpm;              // Speed ceiling (RPM, e.g. 150.0f)
} jerk_limiter_config_t;

typedef struct {
    float v;                    // Continuous velocity state (RPM)
    float a;                    // Continuous acceleration state (RPM/s)
} jerk_limiter_state_t;

static inline void jerk_limiter_init(jerk_limiter_state_t *st) {
    if (!st) return;
    st->v = 0.0f;
    st->a = 0.0f;
}

static inline void jerk_limiter_reset(jerk_limiter_state_t *st, float init_v) {
    if (!st) return;
    if (!isfinite(init_v)) init_v = 0.0f;
    st->v = init_v;
    st->a = 0.0f;
}

static inline bool jerk_limiter_step(
    const jerk_limiter_config_t *cfg,
    jerk_limiter_state_t *st,
    float target_rpm,
    bool stop_bypass,
    float *out_v,
    float *out_a,
    float *out_j
) {
    if (!cfg || !st) return false;
    if (stop_bypass) {
        st->v = 0.0f;
        st->a = 0.0f;
        if (out_v) *out_v = 0.0f;
        if (out_a) *out_a = 0.0f;
        if (out_j) *out_j = 0.0f;
        return true;
    }
    if (!isfinite(target_rpm) || !isfinite(st->v) || !isfinite(st->a)) return false;
    if (!isfinite(cfg->max_slope) || cfg->max_slope <= 0.0f ||
        !isfinite(cfg->max_jerk) || cfg->max_jerk <= 0.0f ||
        !isfinite(cfg->dt_s) || cfg->dt_s <= 0.0f ||
        !isfinite(cfg->min_rpm) || !isfinite(cfg->max_rpm) || cfg->min_rpm >= cfg->max_rpm) {
        return false;
    }

    float target = target_rpm;
    if (target < cfg->min_rpm) target = cfg->min_rpm;
    if (target > cfg->max_rpm) target = cfg->max_rpm;

    // Parabolic braking point if acceleration is immediately pulled back to 0 with max_jerk
    float v_stop = st->v + 0.5f * st->a * fabsf(st->a) / cfg->max_jerk;
    float a_target = 0.0f;

    if (v_stop < target) {
        float err_v = target - st->v;
        if (err_v < 0.0f) err_v = 0.0f;
        float a_bound = sqrtf(2.0f * cfg->max_jerk * err_v);
        a_target = fminf(cfg->max_slope, a_bound);
    } else if (v_stop > target) {
        float err_v = st->v - target;
        if (err_v < 0.0f) err_v = 0.0f;
        float a_bound = sqrtf(2.0f * cfg->max_jerk * err_v);
        a_target = fmaxf(-cfg->max_slope, -a_bound);
    } else {
        a_target = 0.0f;
    }

    float delta_a = a_target - st->a;
    float max_delta_a = cfg->max_jerk * cfg->dt_s;
    float j = 0.0f;
    float a_next = 0.0f;

    if (delta_a > max_delta_a) {
        j = cfg->max_jerk;
        a_next = st->a + max_delta_a;
    } else if (delta_a < -max_delta_a) {
        j = -cfg->max_jerk;
        a_next = st->a - max_delta_a;
    } else {
        j = delta_a / cfg->dt_s;
        a_next = a_target;
    }

    // Clamp acceleration slope
    if (a_next > cfg->max_slope) a_next = cfg->max_slope;
    if (a_next < -cfg->max_slope) a_next = -cfg->max_slope;

    // Integrate velocity with trapezoidal rule
    float v_next = st->v + 0.5f * (st->a + a_next) * cfg->dt_s;

    // Endpoint convergence & lock near target
    if ((st->v <= target && v_next >= target) || (st->v >= target && v_next <= target)) {
        if (fabsf(a_next) <= max_delta_a) {
            v_next = target;
            a_next = 0.0f;
            j = 0.0f;
        }
    }

    // Clamp velocity to hard bounds
    if (v_next < cfg->min_rpm) v_next = cfg->min_rpm;
    if (v_next > cfg->max_rpm) v_next = cfg->max_rpm;

    st->v = v_next;
    st->a = a_next;

    if (out_v) *out_v = v_next;
    if (out_a) *out_a = a_next;
    if (out_j) *out_j = j;
    return true;
}

typedef struct {
    float kp;
    float ki;
    float ts_s;
    float max_pwm;              // Output saturation limit (counts, e.g. 614.0f)
    float min_pwm;              // Output floor (counts, e.g. 0.0f)
    float slew_counts_per_tick; // Slew rate limit (counts/tick, e.g. 5.0f)
    aw_mode_t mode;             // A/B mode
    float kaw;                  // Tracking gain (e.g., 1.0f / (kp + 1e-6f) or bounded 0.0f..10.0f)
    bool ff_enabled;            // Feedforward toggle
    float ff_slope;             // Steady PWM/RPM slope (e.g., 3.2057f)
    float ff_intercept;         // Steady PWM intercept (e.g., 19.9421f)
} pi_controller_config_t;

typedef struct {
    float integral_term;
    float applied_pwm;
    float prev_unsaturated_pwm;
    uint32_t amplitude_saturation_ticks;
    uint32_t slew_saturation_ticks;
    bool was_amplitude_saturated;
    bool was_slew_saturated;
} pi_controller_state_t;

typedef struct {
    float proportional;
    float integral;
    float feedforward;
    float unsaturated_pwm;
    float candidate_pwm;
    float applied_pwm;
    bool amplitude_saturated;
    bool slew_saturated;
} pi_step_result_t;

static inline void pi_controller_init(pi_controller_state_t *st) {
    if (!st) return;
    st->integral_term = 0.0f;
    st->applied_pwm = 0.0f;
    st->prev_unsaturated_pwm = 0.0f;
    st->amplitude_saturation_ticks = 0;
    st->slew_saturation_ticks = 0;
    st->was_amplitude_saturated = false;
    st->was_slew_saturated = false;
}

static inline void pi_controller_reset_integral(pi_controller_state_t *st) {
    if (!st) return;
    st->integral_term = 0.0f;
    st->applied_pwm = 0.0f;
    st->prev_unsaturated_pwm = 0.0f;
    st->was_amplitude_saturated = false;
    st->was_slew_saturated = false;
}

static inline bool pi_controller_step(
    const pi_controller_config_t *cfg,
    pi_controller_state_t *st,
    float setpoint_rpm,
    float effective_setpoint_rpm,
    float measured_rpm,
    pi_step_result_t *res
) {
    if (!cfg || !st) return false;

    // Transactional configuration validation: fail closed if configuration is invalid
    if (!isfinite(cfg->kp) || cfg->kp < 0.0f ||
        !isfinite(cfg->ki) || cfg->ki < 0.0f ||
        !isfinite(cfg->ts_s) || cfg->ts_s <= 0.0f ||
        !isfinite(cfg->max_pwm) || !isfinite(cfg->min_pwm) || cfg->max_pwm <= cfg->min_pwm ||
        !isfinite(cfg->slew_counts_per_tick) || cfg->slew_counts_per_tick <= 0.0f ||
        (cfg->mode != AW_MODE_BASELINE_CLAMP && cfg->mode != AW_MODE_TRACKING) ||
        !isfinite(cfg->kaw) || cfg->kaw < 0.0f) {
        return false;
    }

    if (cfg->ff_enabled) {
        if (!isfinite(cfg->ff_slope) || !isfinite(cfg->ff_intercept)) {
            return false;
        }
    }

    // Check numerical validity of state and inputs
    if (!isfinite(setpoint_rpm) || !isfinite(effective_setpoint_rpm) || 
        !isfinite(measured_rpm) || !isfinite(st->integral_term) || !isfinite(st->applied_pwm) ||
        !isfinite(st->prev_unsaturated_pwm)) {
        return false;
    }

    float error = effective_setpoint_rpm - measured_rpm;
    if (!isfinite(error)) return false;

    float prop = cfg->kp * error;
    if (!isfinite(prop)) return false;

    // Ordinary zero-SP baseline equivalence:
    // If setpoint and ramp reached 0: clear integral and set candidate output to 0,
    // then retain downstream slew rate limiting toward 0.
    // Explicit STOP or fault bypasses this via separate immediate shutdown.
    if (setpoint_rpm <= 0.001f && effective_setpoint_rpm <= 0.001f) {
        float next_integral = 0.0f;
        float candidate_output = 0.0f;
        float raw_unsaturated = prop;
        float current_applied = st->applied_pwm;
        float pwm_delta = candidate_output - current_applied;
        bool slew_sat = false;
        float next_applied = current_applied;

        if (pwm_delta > cfg->slew_counts_per_tick) {
            next_applied += cfg->slew_counts_per_tick;
            slew_sat = true;
        } else if (pwm_delta < -cfg->slew_counts_per_tick) {
            next_applied -= cfg->slew_counts_per_tick;
            slew_sat = true;
        } else {
            next_applied = candidate_output;
        }

        if (next_applied > cfg->max_pwm) next_applied = cfg->max_pwm;
        if (next_applied < cfg->min_pwm) next_applied = cfg->min_pwm;

        if (!isfinite(next_applied) || !isfinite(raw_unsaturated)) return false;

        // Commit state transactionally
        st->integral_term = next_integral;
        st->prev_unsaturated_pwm = raw_unsaturated;
        st->applied_pwm = next_applied;
        st->was_amplitude_saturated = false;
        st->was_slew_saturated = slew_sat;
        if (slew_sat) st->slew_saturation_ticks++;

        if (res) {
            res->proportional = prop;
            res->integral = next_integral;
            res->feedforward = 0.0f;
            res->unsaturated_pwm = raw_unsaturated;
            res->candidate_pwm = candidate_output;
            res->applied_pwm = next_applied;
            res->amplitude_saturated = false;
            res->slew_saturated = slew_sat;
        }
        return true;
    }

    // Compute steady-state feedforward
    float ff = 0.0f;
    if (cfg->ff_enabled && effective_setpoint_rpm > 0.001f) {
        ff = cfg->ff_slope * effective_setpoint_rpm + cfg->ff_intercept;
        if (ff < 0.0f) ff = 0.0f;
        if (ff > cfg->max_pwm) ff = cfg->max_pwm;
    }

    float candidate_integral = st->integral_term;
    float candidate_output = 0.0f;
    float raw_unsaturated = 0.0f;
    bool amp_sat = false;
    bool slew_sat = false;

    if (cfg->mode == AW_MODE_BASELINE_CLAMP) {
        float int_delta = cfg->ki * error * cfg->ts_s;
        if (!isfinite(int_delta)) return false;
        candidate_integral = st->integral_term + int_delta;
        if (!isfinite(candidate_integral)) return false;

        candidate_output = prop + candidate_integral + ff;
        if (!isfinite(candidate_output)) return false;
        raw_unsaturated = candidate_output;

        float next_int = st->integral_term;
        if (candidate_output > cfg->max_pwm) {
            candidate_output = cfg->max_pwm;
            amp_sat = true;
            if (error < 0.0f) {
                next_int = candidate_integral;
            }
        } else if (candidate_output < cfg->min_pwm) {
            candidate_output = cfg->min_pwm;
            amp_sat = true;
            if (error > 0.0f) {
                next_int = candidate_integral;
            }
        } else {
            next_int = candidate_integral;
        }
        candidate_integral = next_int;

        float current_applied = st->applied_pwm;
        float pwm_delta = candidate_output - current_applied;
        float next_applied = current_applied;
        if (pwm_delta > cfg->slew_counts_per_tick) {
            next_applied += cfg->slew_counts_per_tick;
            slew_sat = true;
        } else if (pwm_delta < -cfg->slew_counts_per_tick) {
            next_applied -= cfg->slew_counts_per_tick;
            slew_sat = true;
        } else {
            next_applied = candidate_output;
        }

        if (next_applied > cfg->max_pwm) next_applied = cfg->max_pwm;
        if (next_applied < cfg->min_pwm) next_applied = cfg->min_pwm;

        if (!isfinite(candidate_integral) || !isfinite(next_applied)) return false;

        // Transactional commit
        st->integral_term = candidate_integral;
        st->prev_unsaturated_pwm = raw_unsaturated;
        st->applied_pwm = next_applied;
    } else {
        // AW_MODE_TRACKING: Actual-applied-output tracking anti-windup
        float tracking_diff = st->applied_pwm - st->prev_unsaturated_pwm;
        if (!isfinite(tracking_diff)) return false;

        float integral_delta = (cfg->ki * error + cfg->kaw * tracking_diff) * cfg->ts_s;
        if (!isfinite(integral_delta)) return false;

        candidate_integral = st->integral_term + integral_delta;
        if (!isfinite(candidate_integral)) return false;

        // Bounded sanity clamp on integral
        if (candidate_integral > cfg->max_pwm * 1.5f) {
            candidate_integral = cfg->max_pwm * 1.5f;
        } else if (candidate_integral < -cfg->max_pwm * 0.5f) {
            candidate_integral = -cfg->max_pwm * 0.5f;
        }

        raw_unsaturated = prop + candidate_integral + ff;
        if (!isfinite(raw_unsaturated)) return false;

        candidate_output = raw_unsaturated;
        if (candidate_output > cfg->max_pwm) {
            candidate_output = cfg->max_pwm;
            amp_sat = true;
        } else if (candidate_output < cfg->min_pwm) {
            candidate_output = cfg->min_pwm;
            amp_sat = true;
        }

        float current_applied = st->applied_pwm;
        float pwm_delta = candidate_output - current_applied;
        float next_applied = current_applied;
        if (pwm_delta > cfg->slew_counts_per_tick) {
            next_applied += cfg->slew_counts_per_tick;
            slew_sat = true;
        } else if (pwm_delta < -cfg->slew_counts_per_tick) {
            next_applied -= cfg->slew_counts_per_tick;
            slew_sat = true;
        } else {
            next_applied = candidate_output;
        }

        if (next_applied > cfg->max_pwm) next_applied = cfg->max_pwm;
        if (next_applied < cfg->min_pwm) next_applied = cfg->min_pwm;

        if (!isfinite(candidate_integral) || !isfinite(next_applied)) return false;

        // Transactional commit
        st->integral_term = candidate_integral;
        st->prev_unsaturated_pwm = raw_unsaturated;
        st->applied_pwm = next_applied;
    }

    st->was_amplitude_saturated = amp_sat;
    st->was_slew_saturated = slew_sat;
    if (amp_sat) st->amplitude_saturation_ticks++;
    if (slew_sat) st->slew_saturation_ticks++;

    if (res) {
        res->proportional = prop;
        res->integral = st->integral_term;
        res->feedforward = ff;
        res->unsaturated_pwm = st->prev_unsaturated_pwm;
        res->candidate_pwm = candidate_output;
        res->applied_pwm = st->applied_pwm;
        res->amplitude_saturated = amp_sat;
        res->slew_saturated = slew_sat;
    }

    return true;
}

#ifdef __cplusplus
}
#endif

#endif // PI_AW_CONTROLLER_H
