#pragma once

#include <math.h>

#include <stdint.h>

#include <string.h>

#include "pi_aw_controller.h"

#include "decel_tail_shaper.h"
#include "imu_observer.h"
#include "firmware_config.h"



#define D2_BUILD "DUAL-C8-A1-PACKAGED-20261110"

#ifndef D2_LOW_STARTUP_SCALE
#define D2_LOW_STARTUP_SCALE 1.2f
#endif
static_assert(D2_LOW_STARTUP_SCALE >= 1.0f && D2_LOW_STARTUP_SCALE <= 1.3f, "Bound startup experiment");

#define D2_CPR 998.4f

#define D2_PI 3.14159265358979323846f

// Single rounding of the exact 5 degree boundary, matching host conversion.

#define D2_MIN_TURN_RAD 0.08726646259971647f

// PO's 02/11 physical turn + gyro cross-check: legacy encoder yaw was reversed.

// Keep raw motor/encoder forward signs; correct only the wheel-pair yaw mapping.

#define D2_YAW_SIGN -1

enum D2State { D2_IDLE, D2_MOVE, D2_SPEED, D2_STOPPING, D2_FAULT };

enum D2Fault { D2_NONE, D2_WATCHDOG, D2_DEADLINE, D2_STALL1, D2_STALL2,

               D2_POLARITY, D2_DIVERGENCE, D2_TIMING, D2_CONFIG, D2_TIMER, D2_BOTH_PULSES_LOST };

struct D2Tune { float kp, ki, kaw, slope, offset, ramp, jerk; };
namespace robot_config {
static inline D2Tune tune_from(const float (&v)[7]) {return {v[0],v[1],v[2],v[3],v[4],v[5],v[6]};}
static inline D2Tune low_tune(unsigned i) {return tune_from(LOW_TUNING[i]);}
static inline D2Tune high_tune(unsigned i) {return tune_from(HIGH_TUNING[i]);}
}

struct D2GainSchedule {
    D2Tune high[2];
    float alpha, target_alpha, transition_s, requested_mps;
    uint32_t update_sequence;
    uint8_t staged;
    bool enabled, active_for_run, integral_clamped;
};


struct D2HighCtrl {
    uint8_t mode; // 0=OFF (original B2), 1=HIGH accel cap only, 2=HIGH accel cap + IMU heading PD
    float a_max; // default 0.50 m/s^2
    float k_yaw; // default 1.0 /s
    float k_rate; // default 0.15
    float max_omega; // default 0.15 rad/s
    float slew_omega; // default 0.75 rad/s^2

    // Runtime state (telemetry / per-run tracking)
    bool active;
    bool imu_ok;
    bool headroom_limited;
    bool reference_valid;
    bool used_in_run; // Retain reference offsets through LOW and terminal IDLE telemetry.
    bool accel_limited;
    float high_weight;
    float yaw_ref;
    float current_yaw;
    float gyro_z;
    float yaw_err;
    float omega_cmd;
    float omega_filtered;
    float wheel_offset[2]; // signed differential wheel reference offset (meters)
    float max_wheel_offset;
    uint32_t imu_epoch;
    uint32_t last_imu_sample_ms;
};

static inline bool d2_high_imu_ready(const ImuObserver *im,uint32_t ms) {
    return im && im->connected && im->calibrated && !im->calibrating && im->valid &&
        im->error==0 && im->calibration_id!=0 && (uint32_t)(ms-im->sample_ms)<=50 &&
        isfinite(im->rpy[2]) && isfinite(im->gyro[2]);
}

static inline bool d2_valid_high_ctrl(uint8_t mode, float a_max, float k_yaw, float k_rate, float max_omega, float slew_omega) {
    if (mode != robot_config::HIGH_MODE) return false;
    if (!isfinite(a_max) || a_max < 0.1f || a_max > 2.0f) return false;
    if (!isfinite(k_yaw) || k_yaw < 0.0f || k_yaw > 5.0f) return false;
    if (!isfinite(k_rate) || k_rate < 0.0f || k_rate > 1.0f) return false;
    if (!isfinite(max_omega) || max_omega < 0.01f || max_omega > 0.50f) return false;
    if (!isfinite(slew_omega) || slew_omega < 0.1f || slew_omega > 2.0f) return false;
    return true;
}

// Angular feedback is enabled at boot for TURN_TIME. Raw odometry remains
// encoder-derived. No geometry/gyro scale learning and no automatic reversal.
struct D2TurnFeedback {
    bool enabled,used_in_run,active,valid;
    uint8_t stop_reason; // 0 ongoing,1 target,2 deadline,3 invalid IMU,4 disagreement,5 fine alignment unverified
    uint32_t epoch,sample_ms;
    float yaw_ref,target_rad,angle_rad,yaw_rate,offset_m,rate_offset_mps;
    float target_offset_m; // latched physical wheel target correction at cutoff
    uint8_t finish_phase; // 0 original motion,1 observe coast,2 one forward pulse,3 final coast
    bool correction_used;
    uint32_t cutoff_ms,last_edge_ms,correction_ms,pulse_ms;
    uint16_t correction_elapsed_ms;
    int64_t finish_count[2];
    float cutoff_angle_rad,correction_start_rad;
};

struct D2CreepAssist {
    float boost[2],peak[2];
    uint16_t quiet_ms[2],assisted_ms[2];
    int64_t last_count[2];
    bool used[2],spent[2];
    int64_t start_count[2],credit_count[2];
    uint16_t confirm_ms[2];
    bool confirmed[2],failed[2];
};

struct D2TerminalSettle {
    bool used,pending,qualified;
    uint8_t rejection; // 0 none,1 position,2 heading/IMU,3 no quiet window
    uint32_t start_ms,last_pulse_ms,elapsed_ms,quiet_ms;
    int64_t last_count[2];
    float error_m[2],limit_m,yaw_error_deg;
};

struct D2Core {

    D2State state; D2Fault fault;

    D2Tune tune[2];

    pi_controller_state_t pi[2]; pi_step_result_t result[2];

    float diameter[2], track, max_rpm, max_pwm, slew, sync_kp, sync_max, divergence;

    float rpm[2], raw_rpm[2], base[2], effective[2], pwm[2], distance[2], trim;

    float target_m, duration_s, cruise_mps, ramp_s, speed_rpm;

    int64_t counts[2], origin[2], history[2][6];

    uint32_t history_ms[6], last_step_ms, heartbeat_ms, start_ms, stop_ms, quiet_ms;

    uint16_t stall_ms[2];

    uint16_t both_encoder_quiet_ms;

    int64_t previous_motion_counts[2];

    float governor_scale;

    float lead_cap_rpm;

    float position_ref_m, position_trim_rpm;

    float trajectory_velocity_mps;

    float governor_min_scale;

    // Distance mode only: trade common speed for wheel-distance agreement.

    float accuracy_scale;

    uint16_t governor_pressure_ms;

    uint8_t governor_reason;

    bool governor_last_pressure;

    bool governor_used;

    bool position_cutoff_latched;

    bool distance_priority;

    bool time_priority;

    bool distance_wheel_done[2];

    bool terminal_coasting[2],terminal_learned[2];

    float terminal_margin[2],terminal_cut_distance[2],terminal_cut_speed[2];

    int64_t terminal_last_count[2];

    uint32_t terminal_last_pulse_ms[2],terminal_cut_ms[2];

    uint8_t distance_phase;

    uint8_t distance_stop_reason; // 0 none, 1 target, 2 deadline

    unsigned history_size, history_pos, max_dt_ms;

    int direction;

    bool turning; // Opposite wheel signs; retained until the next admitted command.

    bool outputs, configured;

    uint32_t run_id;

    alignas(8) D2GainSchedule gains;

        // Per-run startup only; never reused for braking or terminal creep.
    float low_startup_s;

    D2HighCtrl high_ctrl;
    float common_v;
    float peak_achieved_mps;
    // Diagnostics: SP passed through the SAME time-window and filter as measured RPM.
    double reference_rpm_ms[2], reference_history[2][6];
    float observed_reference_rpm[2];
    float startup_accel_scale;
    bool startup_heading_guard;
    D2TurnFeedback turn_feedback;
    D2CreepAssist creep;
    D2TerminalSettle terminal_settle;
};

static inline int d2_wheel_direction(const D2Core &c,int i) {

    return c.direction*((c.turning && i==0)?-1:1)*(c.turning?D2_YAW_SIGN:1);

}

static inline float d2_clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

static inline float d2_feedback_progress(const D2Core &c,int i) {
    float encoder=d2_wheel_direction(c,i)*c.distance[i];
    return c.turn_feedback.used_in_run?encoder+c.turn_feedback.offset_m:encoder;
}

static inline float d2_feedback_speed(const D2Core &c,int i,float tick_mps) {
    float encoder=d2_wheel_direction(c,i)*tick_mps;
    return c.turn_feedback.used_in_run?encoder+c.turn_feedback.rate_offset_mps:encoder;
}

// MOVE_DIST target in the same normalized encoder frame as terminal latching.
// A latched arrival may move back by one encoder quantum; a larger retreat is
// an unverified position and must not keep both outputs off until the deadline.
static inline float d2_distance_wheel_target(const D2Core &c,int i) {
    return c.target_m+((c.high_ctrl.mode==2&&c.high_ctrl.used_in_run)?c.high_ctrl.wheel_offset[i]:0.0f);
}
static inline float d2_terminal_quantum(const D2Core &c,int i) {
    return D2_PI*c.diameter[i]/D2_CPR;
}
static inline bool d2_distance_arrived(const D2Core &c,int i) {
    float tolerance=.0005f+(c.distance_wheel_done[i]?d2_terminal_quantum(c,i):0.0f);
    return d2_wheel_direction(c,i)*c.distance[i]>=d2_distance_wheel_target(c,i)-tolerance;
}

// One bounded terminal-friction episode, not a kick stopped by the first edge.
// A pulse may be elastic recoil; progress is credited only after a net advance
// holds. The600ms episode budget is absolute and never reset by encoder jitter.
static inline float d2_creep_assist(D2Core &c,int i,float shaped,uint32_t dt) {
    auto &a=c.creep;
    int64_t delta=d2_wheel_direction(c,i)*(c.counts[i]-a.last_count[i]);
    bool pulse=delta!=0;a.last_count[i]=c.counts[i];a.confirmed[i]=false;
    float remaining=d2_distance_wheel_target(c,i)-d2_wheel_direction(c,i)*c.distance[i];
    float speed=shaped*D2_PI*c.diameter[i]/60;
    bool eligible=c.state==D2_MOVE&&c.distance_priority&&!c.time_priority&&!c.turning&&
        c.distance_phase==2&&!c.distance_wheel_done[i]&&remaining>.0005f&&remaining<=.008f&&
        shaped>1&&speed<=.030001f;
    if(!eligible) {
        if(a.used[i]&&!a.spent[i]) {
            a.spent[i]=true;
            if(c.state==D2_MOVE&&!d2_distance_arrived(c,i))a.failed[i]=true;
        }
        a.boost[i]=0;a.quiet_ms[i]=0;return 0;
    }
    if(a.spent[i])return 0;
    if(!a.used[i]) {
        if(pulse||fabsf(c.rpm[i])>=1){a.quiet_ms[i]=0;return 0;}
        a.quiet_ms[i]=(uint16_t)fminf(1000,a.quiet_ms[i]+dt);
        if(a.quiet_ms[i]<500||c.pi[i].applied_pwm<40||c.pi[i].applied_pwm>=120)return 0;
        a.used[i]=true;a.start_count[i]=a.credit_count[i]=c.counts[i];
        // Grant one finite terminal episode, not a per-pulse stall reset.
        c.stall_ms[i]=0;
    }
    a.assisted_ms[i]=(uint16_t)fminf(1000,a.assisted_ms[i]+dt);
    if(a.assisted_ms[i]>600) {
        // Exhausting extra-force authority is not itself a stall. Preserve
        // ordinary PI completion if credited progress is still retained;
        // the existing stall/deadline guards remain authoritative afterwards.
        bool retained=d2_wheel_direction(c,i)*(a.credit_count[i]-a.start_count[i])>=2&&
                      d2_wheel_direction(c,i)*(c.counts[i]-a.credit_count[i])>=0;
        a.spent[i]=true;a.failed[i]=!retained;a.boost[i]=0;return 0;
    }
    if(d2_wheel_direction(c,i)*(c.counts[i]-a.start_count[i])*d2_terminal_quantum(c,i)>.008f) {
        a.spent[i]=a.failed[i]=true;a.boost[i]=0;return 0;
    }
    if(d2_wheel_direction(c,i)*(c.counts[i]-a.credit_count[i])>=2) {
        a.confirm_ms[i]=(uint16_t)fminf(1000,a.confirm_ms[i]+dt);
        if(a.confirm_ms[i]>=40){a.confirmed[i]=true;a.credit_count[i]=c.counts[i];a.confirm_ms[i]=0;}
    } else a.confirm_ms[i]=0;
    // Breakaway force is only needed while stationary. Remove the extra
    // force on ANY observed advance, without spending the recovery episode.
    // If that edge recoils, force can ramp again within the SAME fixed budget.
    // This avoids accumulating extra force while the RPM filter catches up.
    if(delta>0||d2_wheel_direction(c,i)*c.rpm[i]>=1)a.boost[i]=0;
    else a.boost[i]=fminf(24,a.boost[i]+2*(dt/20.0f));
    a.peak[i]=fmaxf(a.peak[i],a.boost[i]);
    return a.boost[i];
}

static inline bool d2_running(const D2Core &c) {

    return c.state == D2_MOVE || c.state == D2_SPEED || c.state == D2_STOPPING;

}

static inline void d2_disable(D2Core &c) {

    c.outputs = false;
    c.gains.alpha=c.gains.target_alpha=0;c.gains.active_for_run=false;

    for (int i=0;i<2;i++) {

        c.pwm[i]=c.base[i]=c.effective[i]=0;
        c.creep.boost[i]=0;c.creep.quiet_ms[i]=0;

        pi_controller_init(&c.pi[i]);

        memset(&c.result[i],0,sizeof(c.result[i]));

    }

    c.trim=0;c.accuracy_scale=1.0f;c.governor_scale=1.0f;c.governor_pressure_ms=0;c.lead_cap_rpm=0;c.governor_last_pressure=false;c.position_trim_rpm=0;c.position_cutoff_latched=false;c.distance_priority=false;c.distance_phase=0;c.distance_wheel_done[0]=c.distance_wheel_done[1]=false;

        c.terminal_coasting[0]=c.terminal_coasting[1]=false;c.terminal_learned[0]=c.terminal_learned[1]=false;
    c.high_ctrl.active = false;
    c.turn_feedback.active = false;
    c.high_ctrl.high_weight = 0.0f;
    c.high_ctrl.omega_cmd = c.high_ctrl.omega_filtered = 0.0f;
    c.high_ctrl.headroom_limited = false;
    c.common_v = 0.0f;
}

static inline void d2_fault(D2Core &c, D2Fault f) {

    c.terminal_settle.pending=false;
    d2_disable(c); c.fault=f; c.state=D2_FAULT;

}

static inline void d2_stop(D2Core &c) {

    c.terminal_settle.pending=false;
    d2_disable(c); if (c.state!=D2_FAULT) c.state=D2_IDLE;

}

static inline void d2_init(D2Core &c, uint32_t ms) {

    memset(&c,0,sizeof(c));

    c.tune[0]=robot_config::low_tune(0);

    c.tune[1]=robot_config::low_tune(1);

    c.diameter[0]=c.diameter[1]=robot_config::WHEEL_DIAMETER_M; c.track=robot_config::TRACK_M;

    c.max_rpm=robot_config::MAX_RPM; c.max_pwm=robot_config::MAX_PWM;
    c.slew=robot_config::PWM_SLEW; c.sync_kp=robot_config::SYNC_KP; c.sync_max=robot_config::SYNC_MAX_RPM;

    c.divergence=robot_config::DIVERGENCE_M; c.direction=1; c.last_step_ms=c.heartbeat_ms=ms;

        c.quiet_ms=ms; c.configured=true; d2_disable(c);c.governor_min_scale=1.0f;
    c.high_ctrl.mode = robot_config::HIGH_MODE;
    c.high_ctrl.a_max = robot_config::HIGH_A_MAX;
    c.high_ctrl.k_yaw = robot_config::HIGH_K_YAW;
    c.high_ctrl.k_rate = robot_config::HIGH_K_RATE;
    c.high_ctrl.max_omega = robot_config::HIGH_MAX_OMEGA;
    c.high_ctrl.slew_omega = robot_config::HIGH_SLEW_OMEGA;
    c.high_ctrl.active = false;
    c.high_ctrl.imu_ok = false;
    c.high_ctrl.headroom_limited = false;
    c.high_ctrl.high_weight = 0.0f;
    c.high_ctrl.yaw_ref = 0;
    c.high_ctrl.current_yaw = 0;
    c.high_ctrl.gyro_z = 0;
    c.high_ctrl.yaw_err = 0;
    c.high_ctrl.omega_cmd = 0;
    c.high_ctrl.omega_filtered = 0;
    c.high_ctrl.wheel_offset[0] = c.high_ctrl.wheel_offset[1] = 0;
    c.high_ctrl.max_wheel_offset = 0.06f;
    c.high_ctrl.imu_epoch = 0;
    c.high_ctrl.last_imu_sample_ms = 0;
    c.gains.high[0]=robot_config::high_tune(0);
    c.gains.high[1]=robot_config::high_tune(1);
    c.gains.enabled=true; c.gains.transition_s=robot_config::GAIN_TRANSITION_S;
    c.turn_feedback.enabled=true;
    c.common_v = 0.0f;
    c.peak_achieved_mps = 0;
}

static inline bool d2_valid_tune(const D2Tune &t) {

    return isfinite(t.kp)&&t.kp>=0&&t.kp<=10 && isfinite(t.ki)&&t.ki>=0&&t.ki<=10 &&

           isfinite(t.kaw)&&t.kaw>=0&&t.kaw<=10 && isfinite(t.slope)&&t.slope>=0&&t.slope<=20 &&

           isfinite(t.offset)&&t.offset>=0&&t.offset<=200 && isfinite(t.ramp)&&t.ramp>=5&&t.ramp<=500 &&

           isfinite(t.jerk)&&t.jerk>=100&&t.jerk<=10000;

}

static inline bool d2_stationary(const D2Core &c, uint32_t ms) {

    return !c.outputs && fabsf(c.rpm[0])<1.5f && fabsf(c.rpm[1])<1.5f &&

           (uint32_t)(ms-c.quiet_ms)>=200 && c.history_size>=5;

}

static inline float d2_gs_weight(float mps) {
    if(mps<=.30f)return 0;
    if(mps>=.40f)return 1;
    float x=(mps-.30f)/.10f;return x*x*(3-2*x);
}

static inline bool d2_gs_eligible(const D2Core &c) {
    return !c.turning && !c.time_priority &&
        (c.state==D2_SPEED || (c.state==D2_MOVE && c.distance_priority));
}

static inline void d2_gs_invalidate(D2Core &c) {
    c.gains.enabled=false;c.gains.staged=0;
    c.gains.alpha=c.gains.target_alpha=0;c.gains.active_for_run=false;
}

static inline bool d2_gs_stage(D2Core &c,int i,const D2Tune &t,uint32_t ms) {
    if(i<0||i>1||c.state!=D2_IDLE||c.fault!=D2_NONE||!d2_stationary(c,ms)||
       !d2_valid_tune(t)||t.ramp!=c.tune[i].ramp||t.jerk!=c.tune[i].jerk)return false;
    c.gains.enabled=false;c.gains.active_for_run=false;
    c.gains.high[i]=t;c.gains.staged|=1u<<i;return true;
}

static inline bool d2_gs_enable(D2Core &c,bool enabled,float transition_ms,uint32_t ms) {
    if(c.state!=D2_IDLE||c.fault!=D2_NONE||!c.configured||!d2_stationary(c,ms)||
       !isfinite(transition_ms)||transition_ms<100||transition_ms>500)return false;
    if(enabled && (c.gains.staged!=3 || !d2_valid_tune(c.gains.high[0]) ||
       !d2_valid_tune(c.gains.high[1])))return false;
    c.gains.enabled=enabled;c.gains.transition_s=transition_ms*.001f;
    c.gains.staged=0; // Every subsequent commit needs both fresh motor profiles.
    c.gains.alpha=c.gains.target_alpha=0;c.gains.active_for_run=false;return true;
}

static inline void d2_gs_begin(D2Core &c) {
    c.gains.update_sequence=0;c.gains.integral_clamped=false;
    c.gains.requested_mps=c.state==D2_SPEED?c.speed_rpm*D2_PI*c.diameter[0]/60:c.cruise_mps;
    c.gains.active_for_run=c.gains.enabled&&d2_gs_eligible(c);
    c.gains.alpha=c.gains.target_alpha=c.gains.active_for_run?d2_gs_weight(c.gains.requested_mps):0;
}

static inline float d2_gs_mix(float a,float b,float alpha) {
    if(a==b||alpha<=0)return a;if(alpha>=1)return b;return a+(b-a)*alpha;
}

static inline D2Tune d2_gs_tune(const D2Core &c,int i,float alpha) {
    D2Tune t=c.tune[i];
    if(!c.gains.enabled||!c.gains.active_for_run||alpha<=0)return t;
    const D2Tune &h=c.gains.high[i];
    t.kp=d2_gs_mix(t.kp,h.kp,alpha);t.ki=d2_gs_mix(t.ki,h.ki,alpha);
    t.kaw=d2_gs_mix(t.kaw,h.kaw,alpha);t.slope=d2_gs_mix(t.slope,h.slope,alpha);
    t.offset=d2_gs_mix(t.offset,h.offset,alpha);return t;
}

static inline void d2_gs_tick(D2Core &c,float dt) {
    c.gains.integral_clamped=false;
    if(!c.gains.active_for_run||!d2_gs_eligible(c)||
       c.distance_phase!=0||c.position_cutoff_latched||
       c.distance_wheel_done[0]||c.distance_wheel_done[1])return;
    c.gains.target_alpha=d2_gs_weight(c.gains.requested_mps);
    float step=dt/c.gains.transition_s;
    c.gains.alpha+=d2_clamp(c.gains.target_alpha-c.gains.alpha,-step,step);
    c.gains.alpha=d2_clamp(c.gains.alpha,0,1);
}

static inline void d2_gs_rebase(D2Core &c,int i,const D2Tune &old,const D2Tune &next,
                              float sp,float rpm) {
    if(old.kp==next.kp&&old.slope==next.slope&&old.offset==next.offset)return;
    if(sp<=.001f)return;
    float error=sp-rpm;
    float ff_old=d2_clamp(old.slope*sp+old.offset,0,c.max_pwm);
    float ff_new=d2_clamp(next.slope*sp+next.offset,0,c.max_pwm);
    float value=c.pi[i].integral_term+(old.kp-next.kp)*error+ff_old-ff_new;
    float bounded=d2_clamp(value,-c.max_pwm*.5f,c.max_pwm*1.5f);
    c.gains.integral_clamped|=bounded!=value;
    c.pi[i].integral_term=bounded;
}

// Update one active straight command, not a new move. Identity/sequence and
// the original deadline remain mandatory. A rejected request changes nothing.
static inline bool d2_update_speed(D2Core &c,float mps,uint32_t run_id,
                                  uint32_t sequence,uint32_t ms,const ImuObserver *imu=nullptr) {
    if(!d2_gs_eligible(c)||!c.outputs||c.fault!=D2_NONE||!c.configured||
       run_id!=c.run_id||sequence==0||c.gains.update_sequence==UINT32_MAX||
       sequence!=c.gains.update_sequence+1||!isfinite(mps)||mps<.01f||mps>.6f||
       c.distance_phase!=0||c.position_cutoff_latched||
       c.distance_wheel_done[0]||c.distance_wheel_done[1]||
       ms-c.last_step_ms>100||ms-c.heartbeat_ms>480)return false;
    float elapsed=(ms-c.start_ms)*.001f,remaining_time=c.duration_s-elapsed;
    if(remaining_time<=0)return false;
    for(int i=0;i<2;i++)if(mps*60/(D2_PI*c.diameter[i])>c.max_rpm)return false;
    float progress=(d2_wheel_direction(c,0)*c.distance[0]+d2_wheel_direction(c,1)*c.distance[1])*.5f;
    if(c.state==D2_MOVE) {
        if(fmaxf(0,c.target_m-progress)/mps+.5f>remaining_time)return false;
    } else {
        float bound=mps;
        for(int i=0;i<2;i++)bound=fmaxf(bound,fmaxf(fabsf(c.rpm[i]),fabsf(c.effective[i]))*D2_PI*c.diameter[i]/60);
        if(progress+bound*remaining_time>5)return false;
    }
    if(c.distance_priority && !c.time_priority && !c.turning && c.high_ctrl.mode>0) {
        if(c.high_ctrl.mode==2 && mps>.30f &&
           (!c.high_ctrl.reference_valid || !isfinite(c.high_ctrl.yaw_ref) ||
            !d2_high_imu_ready(imu,ms) || imu->calibration_id!=c.high_ctrl.imu_epoch))return false;
        c.high_ctrl.active=mps>.30f;
        if(c.high_ctrl.mode==2 && mps>.30f)c.high_ctrl.used_in_run=true;
    }
    if(c.state==D2_SPEED)c.speed_rpm=mps*60/(D2_PI*c.diameter[0]);
    else c.cruise_mps=mps;
    c.gains.requested_mps=mps;c.gains.target_alpha=c.gains.active_for_run?d2_gs_weight(mps):0;
    c.gains.update_sequence=sequence;return true;
}

static inline bool d2_limits(D2Core &c,float rpm,float pwm,float slew,float kp,float trim,float div) {

    if (c.state!=D2_IDLE || !isfinite(rpm)||rpm<5||rpm>120 || !isfinite(pwm)||pwm<40||pwm>500 ||

        !isfinite(slew)||slew<1||slew>36 || !isfinite(kp)||kp<0||kp>1000 ||

        !isfinite(trim)||trim<0||trim>20 || !isfinite(div)||div<.01f||div>.15f) return false;

    c.max_rpm=rpm;c.max_pwm=pwm;c.slew=slew;c.sync_kp=kp;c.sync_max=trim;c.divergence=div;

    return true;

}

// mode=0: signed distance + duration. mode=1: signed matched-wheel RPM + duration.

static inline bool d2_start(D2Core &c,int mode,float value,float seconds,uint32_t ms,uint32_t run_id) {

    if(c.state!=D2_IDLE||c.fault!=D2_NONE||!c.configured||!d2_stationary(c,ms))return false;

    if(!isfinite(value)||value==0||!isfinite(seconds)||seconds<.5f||seconds>60)return false;

    if(!d2_valid_tune(c.tune[0])||!d2_valid_tune(c.tune[1]))return false;

    if(mode!=0&&mode!=1)return false;

    float ramp=fminf(.5f,seconds*.25f), cruise=0;

    if(mode==0) {

        if(fabsf(value)>5)return false;

        cruise=fabsf(value)/(seconds-ramp);

        for(int i=0;i<2;i++) {

            float rpm=cruise*60/(D2_PI*c.diameter[i]);

            if(rpm>c.max_rpm || rpm/ramp>c.tune[i].ramp)return false;

        }

    } else if(fabsf(value)>c.max_rpm||fabsf(value)*D2_PI*c.diameter[0]/60*seconds>5)return false;

    bool turn_feedback_enabled=c.turn_feedback.enabled;
    memset(&c.turn_feedback,0,sizeof(c.turn_feedback));
    memset(&c.creep,0,sizeof(c.creep));
    memset(&c.terminal_settle,0,sizeof(c.terminal_settle));
    c.turn_feedback.enabled=turn_feedback_enabled;
        c.low_startup_s=0;
    c.high_ctrl.active = false;
    c.high_ctrl.high_weight = 0.0f;
    c.high_ctrl.wheel_offset[0] = c.high_ctrl.wheel_offset[1] = 0.0f;
    c.high_ctrl.omega_cmd = c.high_ctrl.omega_filtered = 0.0f;
    c.high_ctrl.headroom_limited = false;
    c.high_ctrl.reference_valid=false;c.high_ctrl.used_in_run=false;c.high_ctrl.imu_ok=false;
    c.high_ctrl.imu_epoch=0;c.high_ctrl.yaw_ref=0;c.high_ctrl.yaw_err=0;
    c.high_ctrl.accel_limited=false;
    c.startup_accel_scale=1;c.startup_heading_guard=false;
    c.common_v = 0.0f;
    c.peak_achieved_mps = 0;
    c.trajectory_velocity_mps=0;c.time_priority=false;c.turning=false;c.direction=value>0?1:-1;c.target_m=mode==0?fabsf(value):0;

    c.speed_rpm=mode==1?fabsf(value):0;c.duration_s=seconds;c.cruise_mps=cruise;c.ramp_s=ramp;

    c.start_ms=c.heartbeat_ms=ms;c.stop_ms=0;c.run_id=run_id;c.trim=0;c.governor_used=false;

    c.accuracy_scale=1.0f;c.governor_scale=1.0f;c.governor_pressure_ms=0;c.lead_cap_rpm=0;c.governor_min_scale=1.0f;c.governor_reason=0;c.governor_last_pressure=false;c.position_ref_m=0;c.position_trim_rpm=0;c.position_cutoff_latched=false;c.distance_priority=false;c.distance_phase=0;c.distance_wheel_done[0]=c.distance_wheel_done[1]=false;

    c.terminal_coasting[0]=c.terminal_coasting[1]=false;c.terminal_learned[0]=c.terminal_learned[1]=false;c.distance_stop_reason=0;

    for(int i=0;i<2;i++) {

        c.origin[i]=c.counts[i];c.distance[i]=0;c.base[i]=c.effective[i]=c.pwm[i]=0;

        c.stall_ms[i]=0;pi_controller_init(&c.pi[i]);memset(&c.result[i],0,sizeof(c.result[i]));

        c.previous_motion_counts[i]=c.counts[i];

    }

    c.both_encoder_quiet_ms=0;

    c.outputs=true;c.state=mode==0?D2_MOVE:D2_SPEED;d2_gs_begin(c);return true;

}

// Explicit, bounded distance-priority admission; duration is a hard deadline.

static inline bool d2_start_distance(D2Core &c,float distance,float speed_mps,float deadline_s,uint32_t ms,uint32_t run_id,const ImuObserver *imu=nullptr) {

    if(!isfinite(distance)||!isfinite(speed_mps)||!isfinite(deadline_s)||speed_mps<.01f||speed_mps>.6f||deadline_s<.5f||deadline_s>60.0f)return false;

    float nominal=fabsf(distance)/speed_mps+.5f;

    if(nominal>deadline_s || nominal>60.0f)return false;
    if(c.high_ctrl.mode>0 && speed_mps>.30f) {
        // Optimistic lower bound: acceleration only, no braking/settling budget.
        // Reject definitely impossible deadlines; accepting is not a timing guarantee.
        float accel_distance=speed_mps*speed_mps/(2*c.high_ctrl.a_max);
        float minimum=fabsf(distance)<=accel_distance?sqrtf(2*fabsf(distance)/c.high_ctrl.a_max):
            speed_mps/c.high_ctrl.a_max+(fabsf(distance)-accel_distance)/speed_mps;
        if(minimum>deadline_s)return false;
    }

    float ramp=fminf(.5f,nominal*.25f);

    for(int i=0;i<2;i++) {

        float rpm=speed_mps*60/(D2_PI*c.diameter[i]);

        if(rpm>c.max_rpm||rpm/ramp>c.tune[i].ramp)return false;

    }

    if(c.high_ctrl.mode==2 && speed_mps>.30f && !d2_high_imu_ready(imu,ms))return false;
    if(!d2_start(c,0,distance,nominal,ms,run_id))return false;

    c.duration_s=deadline_s;c.cruise_mps=speed_mps;c.distance_priority=true;
    // Latch on admission, so live speed changes cannot re-arm this envelope.
    c.low_startup_s=c.ramp_s*(speed_mps<=.300001f?D2_LOW_STARTUP_SCALE:1.0f);
    bool high_eligible = (c.high_ctrl.mode > 0) && !c.turning && c.distance_priority && (c.cruise_mps > .30f);
    c.high_ctrl.active = high_eligible;
    c.high_ctrl.high_weight = high_eligible ? d2_gs_weight(speed_mps) : 0.0f;
    c.high_ctrl.used_in_run=high_eligible && c.high_ctrl.mode==2;
    if(c.high_ctrl.mode==2 && d2_high_imu_ready(imu,ms)) {
        c.high_ctrl.reference_valid=true;c.high_ctrl.imu_ok=true;
        c.high_ctrl.yaw_ref=imu->rpy[2];c.high_ctrl.imu_epoch=imu->calibration_id;
        c.high_ctrl.last_imu_sample_ms=imu->sample_ms;
    }
    c.high_ctrl.wheel_offset[0] = c.high_ctrl.wheel_offset[1] = 0.0f;
    c.high_ctrl.max_wheel_offset = fminf(0.06f, 0.25f * c.target_m);
    c.high_ctrl.omega_cmd = c.high_ctrl.omega_filtered = 0.0f;
    c.high_ctrl.headroom_limited = false;
    c.common_v = 0.0f;
    c.peak_achieved_mps = 0;
    d2_gs_begin(c);

    return true;

}

// Explicit in-place turn: +angle is left/CCW, -angle is right/CW.

// Each wheel traverses |angle| * track / 2 in opposite physical directions.

static inline bool d2_start_turn(D2Core &c,float angle_rad,float omega_rad_s,float deadline_s,uint32_t ms,uint32_t run_id) {

    if(!isfinite(angle_rad)||fabsf(angle_rad)<D2_MIN_TURN_RAD || fabsf(angle_rad)>2*D2_PI ||

       !isfinite(omega_rad_s)||omega_rad_s<.1f||omega_rad_s>1.0f ||

       !isfinite(c.track)||c.track<.1f||c.track>1.0f)return false;

    float arc=angle_rad*c.track*.5f;

    float wheel_speed=omega_rad_s*c.track*.5f;

    if(!d2_start_distance(c,arc,wheel_speed,deadline_s,ms,run_id))return false;

    c.turning=true;
    d2_gs_begin(c);

    return true;

}

// Position-and-time trajectory, separate from the legacy open-loop time mode.

// Bounded trapezoidal position reference; no deadline extension. The motor

// reference shaper/slew remain active independently of this outer loop.

static inline bool d2_start_timed(D2Core &c,float distance,float seconds,bool turn,uint32_t ms,uint32_t run_id) {

    if(!isfinite(distance)||!isfinite(seconds)||seconds<.5f||seconds>60||fabsf(distance)<(turn?.001f:.02f)||fabsf(distance)>5)return false;

    if(!isfinite(c.track)||c.track<=0)return false;

    if(turn&&fabsf(distance)>D2_PI*c.track)return false;

    // Reserve up to 400 ms for the 100 ms velocity observation window,

    // actuator settling and stationary confirmation. This is an explicit

    // uncalibrated timing budget, not extra time beyond the requested deadline.

    float settle=fminf(.4f,seconds*.15f),motion=seconds-settle;

    float ramp=fminf(.5f,motion*.25f);

    float peak=fabsf(distance)/(motion-ramp);

    float acceleration=peak/ramp;

    for(int i=0;i<2;i++) {

        float rpm=peak*60/(D2_PI*c.diameter[i]);

        if(peak>.6f||rpm>c.max_rpm||acceleration*60/(D2_PI*c.diameter[i])>c.tune[i].ramp)return false;

    }

    if(!d2_start(c,0,distance,seconds,ms,run_id))return false;

    c.cruise_mps=fminf(.6f,peak*1.35f+.03f); // Bounded recovery headroom, never a new hard speed limit.

    c.time_priority=true;c.distance_priority=true;c.turning=turn;

    return true;

}

static inline bool d2_start_turn_time(D2Core &c,float angle_rad,float seconds,uint32_t ms,uint32_t run_id,const ImuObserver *imu=nullptr) {

    if(!isfinite(angle_rad)||fabsf(angle_rad)<D2_MIN_TURN_RAD||fabsf(angle_rad)>2*D2_PI)return false;

    if(c.turn_feedback.enabled&&!d2_high_imu_ready(imu,ms))return false;
    if(!d2_start_timed(c,angle_rad*c.track*.5f,seconds,true,ms,run_id))return false;
    if(c.turn_feedback.enabled) {
        c.turn_feedback.used_in_run=c.turn_feedback.active=c.turn_feedback.valid=true;
        c.turn_feedback.epoch=imu->calibration_id;c.turn_feedback.sample_ms=imu->sample_ms;
        c.turn_feedback.yaw_ref=imu->rpy[2];c.turn_feedback.target_rad=angle_rad;
    }
    return true;

}

static inline void d2_soft_stop(D2Core &c,uint32_t ms) {

    if(d2_running(c)&&c.state!=D2_STOPPING){
        // Preserve every normal target/turn/deadline/fault path. Only an
        // exhausted straight terminal attempt may qualify by settled pose.
        if(c.distance_stop_reason==4 && c.state==D2_MOVE && c.distance_priority &&
           !c.time_priority && !c.turning && c.distance_phase==2) {
            auto &t=c.terminal_settle;t.used=t.pending=true;t.qualified=false;
            t.start_ms=t.last_pulse_ms=ms;t.limit_m=fminf(.01f,.01f*c.target_m);
            for(int i=0;i<2;i++)t.last_count[i]=c.counts[i];
            c.distance_stop_reason=0; // Pending verification, not target reached.
        }
        if(c.turn_feedback.used_in_run) {
            c.turn_feedback.target_offset_m=c.turn_feedback.offset_m;
            c.turn_feedback.active=false;
            if(!c.turn_feedback.stop_reason)c.turn_feedback.stop_reason=c.distance_stop_reason;
        }
        c.state=D2_STOPPING;c.stop_ms=ms;
    }

}

// A zero-output observation window. Never restarts a latched wheel, changes
// the target or grants additional force. Existing 10mm position /1deg heading
// limits are upper bounds; tiny commands also obey a stricter1% distance cap.
static inline void d2_terminal_settle_tick(D2Core &c,uint32_t ms,const ImuObserver *imu) {
    auto &t=c.terminal_settle;if(!t.pending)return;
    if(c.state!=D2_STOPPING || c.fault!=D2_NONE){t.pending=false;return;}
    t.elapsed_ms=ms-t.start_ms;
    for(int i=0;i<2;i++) {
        if(t.last_count[i]!=c.counts[i])t.last_pulse_ms=ms;
        t.last_count[i]=c.counts[i];
        t.error_m[i]=d2_wheel_direction(c,i)*c.distance[i]-d2_distance_wheel_target(c,i);
    }
    t.quiet_ms=ms-t.last_pulse_ms;
    bool stopped=c.pwm[0]<1&&c.pwm[1]<1&&fabsf(c.rpm[0])<1.5f&&fabsf(c.rpm[1])<1.5f;
    if(stopped || ms-c.stop_ms>=500)d2_disable(c);
    float center=(c.direction*(c.distance[0]+c.distance[1])*.5f)-c.target_m;
    bool pose=isfinite(center)&&isfinite(t.limit_m)&&t.limit_m>0&&
              fabsf(center)<=t.limit_m;
    for(int i=0;i<2;i++)pose=pose&&isfinite(t.error_m[i])&&fabsf(t.error_m[i])<=t.limit_m;
    t.yaw_error_deg=(c.distance[1]-c.distance[0])/c.track*180/D2_PI;
    bool heading=isfinite(t.yaw_error_deg)&&fabsf(t.yaw_error_deg)<=1;
    if(c.high_ctrl.mode==2&&(c.high_ctrl.reference_valid||c.high_ctrl.used_in_run)) {
        heading=c.high_ctrl.reference_valid&&d2_high_imu_ready(imu,ms)&&imu->calibration_id==c.high_ctrl.imu_epoch;
        if(heading) {
            float angle=imu->rpy[2]-c.high_ctrl.yaw_ref;
            t.yaw_error_deg=atan2f(sinf(angle),cosf(angle))*180/D2_PI;
            heading=isfinite(t.yaw_error_deg)&&fabsf(t.yaw_error_deg)<=1;
        }
    }
    t.rejection=!pose?1:!heading?2:3;
    if(!c.outputs&&stopped&&t.quiet_ms>=200&&pose&&heading) {
        t.pending=false;t.qualified=true;t.rejection=0;
        c.distance_stop_reason=5; // Settled within tolerance, not exact cutoff.
        d2_stop(c);return;
    }
    if(t.elapsed_ms>=700) {
        t.pending=false;c.distance_stop_reason=4;d2_stop(c);
    }
}

static inline bool d2_clear(D2Core &c,uint32_t ms,bool timer_healthy) {

    if(!timer_healthy||!d2_stationary(c,ms)||d2_running(c))return false;

    d2_stop(c);c.fault=D2_NONE;c.state=D2_IDLE;c.heartbeat_ms=ms;return true;

}

static inline void d2_turn_power_off(D2Core &c) {
    c.outputs=false;
    for(int i=0;i<2;i++) {
        c.base[i]=c.effective[i]=c.pwm[i]=0;
        pi_controller_init(&c.pi[i]);memset(&c.result[i],0,sizeof(c.result[i]));
    }
}

// A single bounded fine pulse AFTER observed standstill, only inside the
// original TURN_TIME command. No reverse, no host retry, no force after deadline.
static inline bool d2_turn_finish_tick(D2Core &c,uint32_t ms) {
    auto &t=c.turn_feedback;
    for(int i=0;i<2;i++) {
        if(t.finish_count[i]!=c.counts[i])t.last_edge_ms=ms;
        t.finish_count[i]=c.counts[i];
    }
    float error=fabsf(t.target_rad)-c.direction*t.angle_rad;
    bool quiet=fabsf(c.rpm[0])<1.5f&&fabsf(c.rpm[1])<1.5f&&
               fabsf(t.yaw_rate)<.035f&&ms-t.last_edge_ms>=80;
    float elapsed=(ms-c.start_ms)*.001f;
    if(t.finish_phase==2) {
        t.correction_elapsed_ms=(uint16_t)fminf(1000,ms-t.correction_ms);
        if(ms-t.correction_ms>=t.pulse_ms || error<=.05f*D2_PI/180 || elapsed>=c.duration_s) {
            t.finish_phase=3;d2_turn_power_off(c);
        }
    } else {
        bool stopped=c.pwm[0]<1&&c.pwm[1]<1;
        if(stopped&&(quiet||ms-t.cutoff_ms>=500))d2_turn_power_off(c);
        if(!c.outputs&&quiet) {
            uint32_t pulse=(uint32_t)d2_clamp((error-.15f*D2_PI/180)*c.track*.5f/.03f*1000,40,140);
            if(t.finish_phase==1&&!t.correction_used&&error>.35f*D2_PI/180&&error<=2*D2_PI/180&&
               elapsed+(pulse+40)*.001f<c.duration_s) {
                t.correction_used=true;t.finish_phase=2;t.correction_ms=ms;t.pulse_ms=pulse;
                t.correction_start_rad=t.angle_rad;c.outputs=true;
            } else {
                if(fabsf(error)>2*D2_PI/180)t.stop_reason=5;
                return true;
            }
        }
    }
    if(ms-t.cutoff_ms>=700) {t.stop_reason=5;d2_turn_power_off(c);return true;}
    return false;
}

static inline void d2_step(D2Core &c,int64_t raw1,int64_t raw2,uint32_t ms,const ImuObserver *imu=nullptr) {

    uint32_t dt=ms-c.last_step_ms;if(dt<20)return;

    c.last_step_ms=ms;if(dt>c.max_dt_ms)c.max_dt_ms=dt;

    int64_t raw[2]={raw1,raw2};
    // Previous effective SP was held over the elapsed interval. Count observation
    // and reference integration use the same sample instants; no fitted time shift.
    for(int i=0;i<2;i++)c.reference_rpm_ms[i]+=(double)c.effective[i]*dt;

    // Encoder signs fixed to vehicle-forward frame: M1=-1, M2=+1.

    float tick_mps[2];

    for(int i=0;i<2;i++) {

        int64_t next=raw[i]*(i==0?-1:1);

        // Unfiltered interval velocity is used only for coast prediction.

        // The 100 ms RPM window remains the PI measurement, but its stale

        // braking velocity must not decide the final wheel cutoff.

        tick_mps[i]=(float)(next-c.counts[i])*D2_PI*c.diameter[i]/D2_CPR/(dt*.001f);

        c.counts[i]=next;

    }

    unsigned slot=c.history_pos;

    for(int i=0;i<2;i++){
        c.history[i][slot]=c.counts[i];
        c.reference_history[i][slot]=c.reference_rpm_ms[i];
    }

    c.history_ms[slot]=ms;c.history_pos=(slot+1)%6;if(c.history_size<6)c.history_size++;

    unsigned oldest=c.history_size<6?0:c.history_pos;

    uint32_t window=ms-c.history_ms[oldest];

    for(int i=0;i<2;i++) {

        if(window) {

            c.raw_rpm[i]=(float)(c.counts[i]-c.history[i][oldest])*60000/(D2_CPR*window);

            c.rpm[i]=.6f*c.raw_rpm[i]+.4f*c.rpm[i];
            float reference=(float)((c.reference_rpm_ms[i]-c.reference_history[i][oldest])/window);
            c.observed_reference_rpm[i]=.6f*reference+.4f*c.observed_reference_rpm[i];

        }

        c.distance[i]=(float)(c.counts[i]-c.origin[i])*D2_PI*c.diameter[i]/D2_CPR;

    }

    if(fabsf(c.rpm[0])>=1.5f||fabsf(c.rpm[1])>=1.5f)c.quiet_ms=ms;

    if(!d2_running(c))return;

    if(dt>40){d2_fault(c,D2_TIMING);return;}

    if((uint32_t)(ms-c.heartbeat_ms)>480){d2_fault(c,D2_WATCHDOG);return;}

    float elapsed=(ms-c.start_ms)*.001f;

    if(elapsed>c.duration_s+1){d2_fault(c,D2_DEADLINE);return;}

    float progress=(d2_wheel_direction(c,0)*c.distance[0]+d2_wheel_direction(c,1)*c.distance[1])*.5f;

    float raw_error=d2_wheel_direction(c,0)*c.distance[0]-d2_wheel_direction(c,1)*c.distance[1];
    float error=raw_error;
    if(c.high_ctrl.mode==2 && c.high_ctrl.used_in_run) {
        float rel0 = d2_wheel_direction(c,0)*c.distance[0] - c.high_ctrl.wheel_offset[0];
        float rel1 = d2_wheel_direction(c,1)*c.distance[1] - c.high_ctrl.wheel_offset[1];
        error = rel0 - rel1;
    }

    // Additional loss-of-pulses guard only after movement is established and

    // both motors are commanded well above crawl.  The ordinary per-wheel

    // stall guards still protect startup and low speed.

    bool both_unchanged=c.counts[0]==c.previous_motion_counts[0] && c.counts[1]==c.previous_motion_counts[1];

    c.previous_motion_counts[0]=c.counts[0];c.previous_motion_counts[1]=c.counts[1];

    bool established_motion=progress>.02f && d2_wheel_direction(c,0)*c.effective[0]>10 && d2_wheel_direction(c,1)*c.effective[1]>10;

    bool creep_motion=(c.distance_phase==2||(c.time_priority&&progress>.02f)) &&

        d2_wheel_direction(c,0)*c.effective[0]>1 && d2_wheel_direction(c,1)*c.effective[1]>1;

    bool watch_pulses=c.distance_priority && c.state==D2_MOVE && (established_motion||creep_motion) &&

        c.pwm[0]>40 && c.pwm[1]>40;

    if(watch_pulses && both_unchanged) {

        if(c.both_encoder_quiet_ms<300)c.both_encoder_quiet_ms+=(uint16_t)dt;

        if(c.both_encoder_quiet_ms>=300){d2_fault(c,D2_BOTH_PULSES_LOST);return;}

    } else c.both_encoder_quiet_ms=0;

    if(fabsf(error)>c.divergence){d2_fault(c,D2_DIVERGENCE);return;}

    // A hard travel guard applies even when the time profile has not finished.

    if(c.target_m>0 && progress>c.target_m+.03f){d2_fault(c,D2_DEADLINE);return;}

    if(fabsf(progress)>5.03f){d2_fault(c,D2_DEADLINE);return;}

    // All preceding pulse/divergence/overtravel guards use REAL encoders.
    // Only a latched TURN_TIME uses IMU angle for its outer angular position
    // loop. The encoder-derived difference (center translation) is preserved.
    if(c.turn_feedback.used_in_run && c.turning && c.time_priority) {
        auto &t=c.turn_feedback;
        bool healthy=d2_high_imu_ready(imu,ms)&&imu->calibration_id==t.epoch;
        float angle=0,rate=0,offset=0;
        if(healthy) {
            float cp=cosf(imu->rpy[1]);
            healthy=isfinite(cp)&&cp>.2f;
            if(healthy) {
                angle=imu->rpy[2]-t.yaw_ref; // unwrapped; 360 is not zero progress
                rate=(imu->gyro[1]*sinf(imu->rpy[0])+imu->gyro[2]*cosf(imu->rpy[0]))/cp;
                offset=c.direction*angle*c.track*.5f-progress;
                healthy=isfinite(angle)&&isfinite(rate)&&fabsf(rate)<=5&&isfinite(offset);
            }
        }
        float disagreement_limit=fmaxf(.005f,fminf(.020f,.03f*c.target_m));
        bool disagreement=healthy&&fabsf(offset)>disagreement_limit;
        t.valid=healthy&&!disagreement;
        if(!t.valid) {
            if(!t.stop_reason)t.stop_reason=disagreement?4:3;
            c.distance_stop_reason=3;d2_soft_stop(c,ms);
        } else {
            t.sample_ms=imu->sample_ms;t.angle_rad=angle;t.yaw_rate=rate;
            t.offset_m=offset;
            float encoder_rate=(d2_wheel_direction(c,0)*tick_mps[0]+d2_wheel_direction(c,1)*tick_mps[1])*.5f;
            t.rate_offset_mps=c.direction*rate*c.track*.5f-encoder_rate;
        }
        progress+=t.offset_m;
    }

    // Disable outputs at deadline; the host checks final distance only after standstill.

    // Existing 30 mm overtravel and duration+1s limits remain active in STOPPING.

    if(elapsed>=c.duration_s) {
        if(c.turning&&c.time_priority&&c.turn_feedback.used_in_run&&c.turn_feedback.finish_phase&&c.state==D2_MOVE) {
            if(c.turn_feedback.finish_phase==2)c.turn_feedback.correction_elapsed_ms=(uint16_t)fminf(1000,ms-c.turn_feedback.correction_ms);
            c.turn_feedback.finish_phase=3;d2_turn_power_off(c);
        } else {if(c.distance_priority&&c.state==D2_MOVE)c.distance_stop_reason=2;d2_soft_stop(c,ms);}
    }

    float v=c.cruise_mps;

    if(c.state==D2_MOVE && c.time_priority) {

        float settle=fminf(.4f,c.duration_s*.15f),motion=c.duration_s-settle;

        float ramp=fminf(.5f,motion*.25f),peak=c.target_m/(motion-ramp);

        float t=fminf(elapsed,motion),reference=0,velocity=0;

        if(t<ramp){velocity=peak*t/ramp;reference=.5f*peak*t*t/ramp;}

        else if(t<motion-ramp){velocity=peak;reference=peak*(t-.5f*ramp);}

        else {float left=motion-t;velocity=peak*left/ramp;reference=c.target_m-.5f*peak*left*left/ramp;}

        c.position_ref_m=reference;

        c.trajectory_velocity_mps=velocity;

        float correction=d2_clamp(4.0f*(reference-progress),-.15f,.15f);

        c.position_trim_rpm=correction*60/(D2_PI*c.diameter[0]);

        v=d2_clamp(velocity+correction,0,c.cruise_mps);

        float remaining=fmaxf(0,c.target_m-progress);

        v=fminf(v,sqrtf(.6f*remaining));

        // Avoid asymptotic sub-count creep only in the terminal window.

        // The time limit, stall check and no-reversal rule remain active.

        if(elapsed>=motion && remaining>.001f)v=fmaxf(v,fminf(.035f,c.cruise_mps));

        bool done=true;
        if(c.turn_feedback.used_in_run) {
            // Yaw is one coupled axis: do not latch/coast one wheel while the
            // other changes the IMU-derived progress of BOTH virtual wheels.
            // Keep encoder centering active until the shared angular cutoff.
            float speed=fmaxf(0,c.direction*c.turn_feedback.yaw_rate*c.track*.5f);
            float coast=d2_clamp(speed*.15f,.0005f,.006f);
            done=c.turn_feedback.valid&&speed<=.04f&&progress>=c.target_m-coast;
            auto &t=c.turn_feedback;
            if(!t.finish_phase&&done) {
                t.finish_phase=1;t.cutoff_ms=t.last_edge_ms=ms;t.cutoff_angle_rad=t.angle_rad;
                for(int i=0;i<2;i++)t.finish_count[i]=c.counts[i];
            }
            if(t.finish_phase)done=d2_turn_finish_tick(c,ms);
            c.distance_phase=done?1:0;
        } else {
        for(int i=0;i<2;i++) {

            float speed=fmaxf(0,d2_feedback_speed(c,i,tick_mps[i]));

            float wheel_progress=d2_feedback_progress(c,i);

            float coast=fminf(.006f,fmaxf(.002f,speed*.15f));

            if(c.terminal_learned[i])coast=d2_clamp(c.terminal_margin[i]*speed/fmaxf(.01f,c.terminal_cut_speed[i]),.0003f,.006f);

            // Power-off prediction and measured arrival are separate decisions.

            if(speed<=.04f&&wheel_progress>=c.target_m-.001f)c.distance_wheel_done[i]=true;

            if(c.terminal_coasting[i]) {

                if(c.counts[i]!=c.terminal_last_count[i]){c.terminal_last_count[i]=c.counts[i];c.terminal_last_pulse_ms[i]=ms;}

                bool settled=ms-c.terminal_last_pulse_ms[i]>=100&&ms-c.terminal_cut_ms[i]>=120&&fabsf(c.rpm[i])<1.5f;

                if(settled&&!c.distance_wheel_done[i]&&elapsed<c.duration_s-.08f) {

                    c.terminal_margin[i]=fmaxf(0,wheel_progress-c.terminal_cut_distance[i]);

                    c.terminal_learned[i]=true;c.terminal_coasting[i]=false;

                }

            } else if(!c.distance_wheel_done[i]&&speed<=.04f&&wheel_progress>=c.target_m-coast) {

                c.terminal_coasting[i]=true;c.terminal_cut_distance[i]=wheel_progress;

                c.terminal_cut_speed[i]=fmaxf(.01f,speed);c.terminal_last_count[i]=c.counts[i];

                c.terminal_cut_ms[i]=c.terminal_last_pulse_ms[i]=ms;

            }

            done=done&&c.distance_wheel_done[i];

        }

        c.distance_phase=(c.terminal_learned[0]||c.terminal_learned[1])?2:

                         (c.terminal_coasting[0]||c.terminal_coasting[1]||c.distance_wheel_done[0]||c.distance_wheel_done[1])?1:0;
        }

        if(done){c.position_cutoff_latched=true;c.distance_stop_reason=1;d2_soft_stop(c,ms);v=0;}

    } else if(c.state==D2_MOVE && c.distance_priority) {

        // Experimental: measured progress limits the speed; deadline and all

        // hard guards remain in force.  No reverse or automatic second move.

        float remaining=fmaxf(0.0f,c.target_m-progress);

        float measured_mps=fmaxf(0.0f,(d2_wheel_direction(c,0)*c.rpm[0]+d2_wheel_direction(c,1)*c.rpm[1])*.5f)*D2_PI*c.diameter[0]/60;
        if(measured_mps>c.peak_achieved_mps)c.peak_achieved_mps=measured_mps;

        float margin=fminf(.05f,measured_mps*.25f);

        c.position_ref_m=c.target_m;c.position_trim_rpm=0;

        bool both_reached=true;
        for(int i=0;i<2;i++) {
            bool arrived=d2_distance_arrived(c,i);
            if((c.distance_wheel_done[i]&&!arrived)||(c.creep.failed[i]&&!arrived)) {
                // Do not restart a wheel after a significant post-latch retreat.
                // Reason4 is a bounded unsuccessful stop, never target success.
                c.distance_stop_reason=4;d2_soft_stop(c,ms);v=0;c.common_v=0;
                both_reached=false;break;
            }
            if(!arrived)both_reached=false;
        }

        if(c.state==D2_STOPPING) {
            v=0;c.common_v=0;
        } else if(both_reached) {
            c.position_cutoff_latched=true;c.distance_stop_reason=1;d2_soft_stop(c,ms);v=0;c.common_v=0;
        } else if(c.distance_phase==1) {
            v=0;c.common_v=0;
            if(fabsf(c.rpm[0])<1.5f && fabsf(c.rpm[1])<1.5f &&
               c.pwm[0]<1 && c.pwm[1]<1 && (uint32_t)(ms-c.quiet_ms)>=200)c.distance_phase=2;
        } else if(c.distance_phase==2) {
            v=fminf(c.cruise_mps,.03f);c.common_v=v;
        } else if(remaining<=fmaxf(.0005f,margin)) {
            c.distance_phase=1;v=0;c.common_v=0;
        } else {
            float preview_m=fminf(.05f,measured_mps*.20f);
            v=fminf(v,sqrtf(.30f*fmaxf(0.0f,remaining-preview_m)));
            float startup_s=fmaxf(c.ramp_s,c.low_startup_s);
            if(elapsed<startup_s)v*=elapsed/startup_s;

            // The HIGH limiter runs after Governor and joint headroom allocation.
        }

    } else if(c.state==D2_MOVE) {

        // Preserve candidate3 MOVE semantics; distance priority is explicit.

        c.position_ref_m=0;c.position_trim_rpm=0;

        if(elapsed<c.ramp_s)v*=elapsed/c.ramp_s;

        else if(elapsed>c.duration_s-c.ramp_s)v*=fmaxf(0,(c.duration_s-elapsed)/c.ramp_s);

        if(progress>=c.target_m && elapsed>=c.duration_s-c.ramp_s)v=0;

    } else c.position_trim_rpm=0;

    c.trim=d2_clamp(c.sync_kp*error,-c.sync_max,c.sync_max);

    float delta_s=dt*.001f;
    float previous_gain_alpha=c.gains.alpha;
    d2_gs_tick(c,delta_s);

    float v_diff_raw = 0.0f;
    c.high_ctrl.headroom_limited = false;
    c.high_ctrl.omega_cmd=0;c.high_ctrl.accel_limited=false;
    c.startup_accel_scale=1;c.startup_heading_guard=false;

    // Feature weight filter with 150ms bounded transition
    float target_weight = 0.0f;
    if (c.high_ctrl.mode > 0 && c.state==D2_MOVE && c.cruise_mps > .30f && c.distance_priority && !c.time_priority && !c.turning) {
        float remaining_w = fmaxf(0.0f, c.target_m - progress);
        float measured_mps_w = fmaxf(0.0f, (d2_wheel_direction(c,0)*c.rpm[0] + d2_wheel_direction(c,1)*c.rpm[1])*.5f)*D2_PI*c.diameter[0]/60;
        float margin_w = fminf(.05f, measured_mps_w * .25f);
        if (c.distance_phase == 0 && remaining_w > margin_w) {
            target_weight = d2_gs_weight(c.cruise_mps);
        }
    }
    float weight_step = delta_s / 0.150f;
    c.high_ctrl.high_weight += d2_clamp(target_weight - c.high_ctrl.high_weight, -weight_step, weight_step);
    c.high_ctrl.high_weight = d2_clamp(c.high_ctrl.high_weight, 0.0f, 1.0f);

    c.high_ctrl.active=c.high_ctrl.mode>0 && c.state==D2_MOVE && c.distance_priority &&
        !c.time_priority && !c.turning && c.cruise_mps>.30f;
    if (c.high_ctrl.mode == 2 && (c.high_ctrl.active || c.high_ctrl.high_weight>0) && c.state == D2_MOVE) {
        // IMU Health verification
        bool imu_fresh=d2_high_imu_ready(imu,ms) && c.high_ctrl.reference_valid &&
            imu->calibration_id==c.high_ctrl.imu_epoch;

        if (!imu_fresh) {
            c.high_ctrl.imu_ok = false;
            c.distance_stop_reason = 3; // IMU lost/invalid during Mode 2 run
            d2_soft_stop(c, ms);
            v = 0; c.common_v = 0;
        } else {
            c.high_ctrl.imu_ok = true;
            c.high_ctrl.last_imu_sample_ms = imu->sample_ms;
            c.high_ctrl.current_yaw = imu->rpy[2];
            c.high_ctrl.gyro_z = imu->gyro[2];

            // Bounded O(1) angle wrap to [-pi, pi]
            float diff = c.high_ctrl.yaw_ref - imu->rpy[2];
            if (!isfinite(diff)) {
                c.high_ctrl.imu_ok = false;
                c.distance_stop_reason = 3;
                d2_soft_stop(c, ms);
                v = 0; c.common_v = 0;
            } else {
                float e_yaw = fmodf(diff + D2_PI, 2.0f * D2_PI);
                if (e_yaw < 0.0f) e_yaw += 2.0f * D2_PI;
                e_yaw -= D2_PI;
                c.high_ctrl.yaw_err = e_yaw;

                // PD calculation
                float omega_raw = c.high_ctrl.k_yaw * e_yaw - c.high_ctrl.k_rate * imu->gyro[2];

                // Angular slew rate limiter
                float max_d_omega = c.high_ctrl.slew_omega * delta_s;
                float omega = d2_clamp(omega_raw, c.high_ctrl.omega_filtered - max_d_omega, c.high_ctrl.omega_filtered + max_d_omega);
                omega = d2_clamp(omega, -c.high_ctrl.max_omega, c.high_ctrl.max_omega);
                c.high_ctrl.omega_filtered = omega;

                // Scale omega by smooth feature weight
                omega *= c.high_ctrl.high_weight;
                c.high_ctrl.omega_cmd = omega;

                v_diff_raw = c.direction * (omega * c.track * 0.5f);

            }
        }
    }

    // Prior-cycle outputs only: require both a real tracking deficit and sustained saturation.

    // Hysteresis and bounded rates prevent a short PWM transient from changing the command.

    bool pressure=false;

    if(c.state==D2_MOVE || c.state==D2_SPEED){

        // Planned acceleration naturally saturates briefly. Wait until its

        // reference ramp plus a short settling allowance before counting pressure.

        bool cruise_observation=elapsed>=c.ramp_s+(c.distance_priority?-.2f:.2f);

        for(int i=0;i<2;i++)if(c.pwm[i]>=c.max_pwm*.98f &&

           cruise_observation && fabsf(c.effective[i])>8.0f && fabsf(c.effective[i])-d2_wheel_direction(c,i)*c.rpm[i]>4.0f)pressure=true;

        c.governor_last_pressure=pressure;

        if(pressure){if(c.governor_pressure_ms<1000)c.governor_pressure_ms+=(uint16_t)dt;}

        else if(c.governor_pressure_ms>dt)c.governor_pressure_ms-=(uint16_t)dt;

        else c.governor_pressure_ms=0;

        if(c.governor_pressure_ms>=(c.distance_priority?60:120)){

            float down_rate=c.distance_priority?1.4f:.35f;

            c.governor_scale=fmaxf(.55f,c.governor_scale-down_rate*delta_s);c.governor_used=true;c.governor_reason|=1;

        }

        else if(c.governor_pressure_ms==0)c.governor_scale=fminf(1.0f,c.governor_scale+.30f*delta_s);

        c.governor_min_scale=fminf(c.governor_min_scale,c.governor_scale);

    }

    // Within 3 mm, keep the requested speed. Progressively slow the pair

    // beyond that band, before the independent 40 mm divergence protection.

    // Bounded rates avoid an abrupt reference jump; deadlines never change.

    if(c.state==D2_MOVE && c.distance_priority && !c.time_priority && c.distance_phase==0) {

        c.accuracy_scale=1.0f;

    } else c.accuracy_scale=1.0f;

    int lead=error>0?0:1,lag=1-lead;

    bool lead_limit=(c.state==D2_MOVE||c.state==D2_SPEED) && fabsf(error)>.006f &&

                    (c.pwm[lag]>=c.max_pwm*.95f) &&

                    d2_wheel_direction(c,lead)*c.rpm[lead]>d2_wheel_direction(c,lag)*c.rpm[lag]+2.0f;

    c.lead_cap_rpm=0;

    // One mixer for the experimental path. LOW/mode0/other commands retain B2.
    // Ksync is tracking feedback, never part of the integrated heading reference.
    bool joint_path=c.high_ctrl.mode>0 && c.state==D2_MOVE && c.distance_priority &&
        !c.time_priority && !c.turning && c.distance_phase==0 &&
        (c.high_ctrl.active || c.high_ctrl.high_weight>0 || c.common_v>.30f);
    float joint_target[2]={},without_heading[2]={},joint_base[2]={};
    float heading_delta_m=0;
    if(joint_path) {
        float common=v*c.governor_scale*c.accuracy_scale;
        float heading=v_diff_raw*c.governor_scale*c.accuracy_scale;
        float budget=c.high_ctrl.max_wheel_offset,off=c.high_ctrl.wheel_offset[0];
        heading=d2_clamp(heading,(-budget-off)/delta_s,(budget-off)/delta_s);
        heading=d2_clamp(heading,-common,common);
        float circumference=D2_PI*(c.diameter[0]+c.diameter[1])*.5f;
        float sync=-c.trim*circumference/60;
        float total=sync+heading;
        float max_v=c.max_rpm*D2_PI*fminf(c.diameter[0],c.diameter[1])/60;
        float headroom=fmaxf(0,max_v-fabsf(total));
        if(common>headroom){common=headroom;c.high_ctrl.headroom_limited=true;}
        // C2: give the existing heading controller time to recover during reverse
        // HIGH startup. Reduce only positive common acceleration, never braking,
        // LOW, forward motion or mode1. No extra turn, timeout extension or retry.
        if(c.direction<0 && c.high_ctrl.mode==2 && c.high_ctrl.active &&
           c.high_ctrl.imu_ok && elapsed<1.5f) {
            float error=c.high_ctrl.yaw_err;
            float outward=error*c.high_ctrl.gyro_z<0?fabsf(c.high_ctrl.gyro_z):0;
            float predicted=fabsf(error)+.25f*outward;
            const float soft_angle=.5f*D2_PI/180;
            c.startup_accel_scale=d2_clamp(soft_angle/fmaxf(soft_angle,predicted),.35f,1.0f);
            c.startup_heading_guard=c.startup_accel_scale<.999f;
        }
        float upper=c.common_v+c.high_ctrl.a_max*c.startup_accel_scale*delta_s;
        if(common>upper){common=upper;c.high_ctrl.accel_limited=true;}
        common=fmaxf(0,common);c.common_v=common;
        total=d2_clamp(total,-common,common);
        float baseline=d2_clamp(sync,-common,common);
        for(int i=0;i<2;i++) {
            float sign=i==0?1.0f:-1.0f,conv=60/(D2_PI*c.diameter[i]);
            joint_target[i]=(common+sign*total)*conv;
            without_heading[i]=(common+sign*baseline)*conv;
            joint_base[i]=(common+sign*(total-baseline))*conv;
        }
    } else c.common_v=(c.state==D2_MOVE?v*c.governor_scale*c.accuracy_scale:0);

    for(int i=0;i<2;i++) {

        if(c.time_priority&&(c.distance_wheel_done[i]||c.terminal_coasting[i])) {

            // Measured completion stays latched. A temporary coast holds PWM

            // off without integrating; any recovery is admitted above only

            // after pulse/RPM quiet and before the unchanged time deadline.

            c.base[i]=c.effective[i]=c.pwm[i]=0;

            if(c.distance_wheel_done[i])pi_controller_init(&c.pi[i]);

            else {c.pi[i].applied_pwm=0;c.pi[i].prev_unsaturated_pwm=0;}

            memset(&c.result[i],0,sizeof(c.result[i]));

            c.stall_ms[i]=0;continue;

        }

        float base=c.state==D2_STOPPING?0:(c.state==D2_SPEED?c.speed_rpm:v*60/(D2_PI*c.diameter[i]));

        base*=c.governor_scale*c.accuracy_scale;

        float target=base>0?d2_clamp(base+(i==0?-c.trim:c.trim),0,c.max_rpm):0;
        if(joint_path){base=joint_base[i];target=joint_target[i];}

        if(c.time_priority&&c.state==D2_MOVE) {

            // Each wheel tracks the scheduled position. Configured Ksync

            // adds bounded cross-coupling to reject unequal motor response;

            // it must not replace independent recovery of lost wheel travel.

            float wheel_progress=d2_feedback_progress(c,i);

            float sync_mps=(i==0?-c.trim:c.trim)*D2_PI*c.diameter[i]/60;

            // Allocate differential authority before clipping the common speed.

            // Clipping two large absolute requests independently erases Ksync

            // precisely when the lagging wheel needs maximum recovery.

            float differential=4.0f*(progress-wheel_progress)+sync_mps;

            float common=c.trajectory_velocity_mps+4.0f*(c.position_ref_m-progress);

            float velocity_limit=fminf(c.cruise_mps,c.max_rpm*D2_PI*c.diameter[i]/60);

            common=d2_clamp(common,0,fmaxf(0,velocity_limit-fabsf(differential)));

            float requested=common+differential;

            float remaining=fmaxf(0,c.target_m-wheel_progress);

            float measured=fmaxf(0,d2_wheel_direction(c,i)*c.rpm[i])*D2_PI*c.diameter[i]/60;
            if(c.turn_feedback.used_in_run)measured=fmaxf(0,d2_feedback_speed(c,i,tick_mps[i]));

            requested=d2_clamp(requested,0,c.cruise_mps);

            float motion=c.duration_s-fminf(.4f,c.duration_s*.15f);

            float ramp=fminf(.5f,motion*.25f);

            float planned_decel=c.target_m/((motion-ramp)*ramp);

            // The distance envelope must use the same deceleration as the

            // admitted time profile. A fixed 0.3 m/s^2 envelope silently

            // brakes a 0.5 m/s trajectory far earlier than its schedule.

            // Predict the distance already committed by the observed motion,

            // independently of the new request. A request-only braking curve

            // ignores the motor's present velocity during tracking lag.

            float slew_decel=c.slew/.02f/fmaxf(.01f,c.tune[i].slope)*D2_PI*c.diameter[i]/60;

            float decel=fminf(planned_decel,slew_decel);

            float preview=fminf(.12f,measured*.15f);

            requested=fminf(requested,sqrtf(2*decel*fmaxf(0,remaining-preview)));

            if(elapsed>=motion&&remaining>.001f&&measured<.04f)requested=fmaxf(requested,fminf(.035f,c.cruise_mps));

            requested*=c.governor_scale;

            target=d2_clamp(requested*60/(D2_PI*c.diameter[i]),0,c.max_rpm);

        }

        // Slow the leading wheel before the 40 mm divergence fault when a real

        // displacement error exists and the lagging output is near saturation.

        // Never reverse a wheel, never bypass stall detection or extend deadlines.

        if(i==lead && lead_limit){

            float cap=fmaxf(0.0f,d2_wheel_direction(c,lag)*c.rpm[lag]+1.0f);

            if(target>cap){target=cap;c.governor_used=true;c.governor_reason|=2;c.lead_cap_rpm=cap;}
            if(joint_path)without_heading[i]=fminf(without_heading[i],cap);

        }

        if(c.state==D2_MOVE && c.distance_priority && (c.distance_phase==2||c.time_priority)) {

            // Coast a wheel once it reaches its own target. Latch until the

            // move ends so noise cannot restart it. No reversal is permitted.

            if(!c.time_priority&&d2_distance_arrived(c,i))c.distance_wheel_done[i]=true;

            if(c.distance_wheel_done[i]) {

                c.base[i]=c.effective[i]=c.pwm[i]=0;
                c.creep.boost[i]=0;

                pi_controller_init(&c.pi[i]);

                memset(&c.result[i],0,sizeof(c.result[i]));

                c.stall_ms[i]=0;

                continue;

            }

            // Synchronization may slow a wheel, but must not accelerate the

            // remaining wheel above the creep speed during terminal alignment.

            if(!c.time_priority)target=fminf(target,base);

        }

        // Timed metrics must compare against the actual position/sync request,

        // not the unused legacy common-speed reference.

        bool turn_finish=c.turning&&c.time_priority&&c.turn_feedback.used_in_run&&c.turn_feedback.finish_phase;
        if(turn_finish)target=c.turn_feedback.finish_phase==2?fminf(target,.03f*60/(D2_PI*c.diameter[i])):0;
        c.base[i]=d2_wheel_direction(c,i)*(c.time_priority?target:base);

        float previous_sp=fabsf(c.effective[i]);
        float shaped=decel_tail_shaper_step(previous_sp,target,c.tune[i].ramp,c.tune[i].jerk,delta_s);
        if(joint_path && c.high_ctrl.mode==2) {
            float baseline=decel_tail_shaper_step(previous_sp,without_heading[i],c.tune[i].ramp,c.tune[i].jerk,delta_s);
            heading_delta_m+=(i==0?1.0f:-1.0f)*.5f*(shaped-baseline)*D2_PI*c.diameter[i]/60*delta_s;
        }

        c.effective[i]=d2_wheel_direction(c,i)*shaped;

        pi_controller_config_t cfg={};

        D2Tune t=d2_gs_tune(c,i,c.gains.alpha);
        D2Tune previous_t=d2_gs_tune(c,i,previous_gain_alpha);
        d2_gs_rebase(c,i,previous_t,t,shaped,d2_wheel_direction(c,i)*c.rpm[i]);
        cfg.kp=t.kp;cfg.ki=t.ki;cfg.kaw=t.kaw;cfg.ts_s=delta_s;

        cfg.mode=AW_MODE_TRACKING;cfg.ff_enabled=true;cfg.ff_slope=t.slope;cfg.ff_intercept=t.offset;

        cfg.max_pwm=c.max_pwm;cfg.min_pwm=0;cfg.slew_counts_per_tick=c.slew;
        if(turn_finish&&c.turn_feedback.finish_phase==2)cfg.max_pwm=fminf(cfg.max_pwm,120);
        float creep_boost=d2_creep_assist(c,i,shaped,dt);
        if(c.creep.used[i]&&!c.creep.spent[i]) {
            cfg.ff_intercept+=creep_boost;
            cfg.max_pwm=fminf(cfg.max_pwm,120.0f);
        }

        if(!pi_controller_step(&cfg,&c.pi[i],target,shaped,d2_wheel_direction(c,i)*c.rpm[i],&c.result[i])) {

            d2_fault(c,D2_CONFIG);return;

        }

        c.pwm[i]=c.pi[i].applied_pwm;

        if(c.pwm[i]>40 && d2_wheel_direction(c,i)*c.rpm[i]<-3){d2_fault(c,D2_POLARITY);return;}

        // Creep can demand less than 5 RPM after wheel synchronization;

        // keep the stall guard active for a genuine nonzero creep command.

        bool stall_command=shaped>5 || (c.distance_priority && (c.distance_phase==2||c.time_priority) && shaped>1);

        if(stall_command && c.pwm[i]>40 && fabsf(c.rpm[i])<1) {

            if(c.creep.confirmed[i])c.stall_ms[i]=0;
            else c.stall_ms[i]+=(uint16_t)dt;

            if(c.stall_ms[i]>=800){

                if(c.distance_priority && c.distance_phase==2) {

                    // A creep stall alone is not evidence that BOTH targets
                    // were reached. Keep the stop bounded and truthfully failed.
                    bool arrived=c.time_priority||(d2_distance_arrived(c,0)&&d2_distance_arrived(c,1));
                    c.position_cutoff_latched=arrived;c.distance_stop_reason=arrived?1:4;
                    d2_soft_stop(c,ms);return;

                }

                d2_fault(c,i==0?D2_STALL1:D2_STALL2);return;

            }

        } else c.stall_ms[i]=0;

    }

    if(joint_path && c.high_ctrl.mode==2 && c.state==D2_MOVE) {
        c.high_ctrl.wheel_offset[0]=d2_clamp(c.high_ctrl.wheel_offset[0]+heading_delta_m,
            -c.high_ctrl.max_wheel_offset,c.high_ctrl.max_wheel_offset);
        c.high_ctrl.wheel_offset[1]=-c.high_ctrl.wheel_offset[0];
        c.high_ctrl.omega_cmd=c.direction*2*heading_delta_m/(delta_s*c.track);
    }

    if(c.terminal_settle.pending)d2_terminal_settle_tick(c,ms,imu);
    else if(c.state==D2_STOPPING && ((c.pwm[0]<1&&c.pwm[1]<1&&fabsf(c.rpm[0])<1.5f&&fabsf(c.rpm[1])<1.5f)||ms-c.stop_ms>=500))d2_stop(c);

}

// Mutually exclusive bridge legs; reverse direction changes only after stationary admission.

static inline void d2_legs(float pwm,int sign,uint16_t &a,uint16_t &b) {

    uint16_t duty=(uint16_t)(d2_clamp(pwm,0,500)+.5f);a=sign>0?duty:0;b=sign<0?duty:0;

}

