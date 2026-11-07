#pragma once

#include <math.h>

#include <stdint.h>

#include <string.h>

#include "pi_aw_controller.h"

#include "decel_tail_shaper.h"



#define D2_BUILD "DUAL-GOV7-LOW-STARTUP-20261107-B2"

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

struct D2GainSchedule {
    D2Tune high[2];
    float alpha, target_alpha, transition_s, requested_mps;
    uint32_t update_sequence;
    uint8_t staged;
    bool enabled, active_for_run, integral_clamped;
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

};

static inline int d2_wheel_direction(const D2Core &c,int i) {

    return c.direction*((c.turning && i==0)?-1:1)*(c.turning?D2_YAW_SIGN:1);

}

static inline float d2_clamp(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

static inline bool d2_running(const D2Core &c) {

    return c.state == D2_MOVE || c.state == D2_SPEED || c.state == D2_STOPPING;

}

static inline void d2_disable(D2Core &c) {

    c.outputs = false;
    c.gains.alpha=c.gains.target_alpha=0;c.gains.active_for_run=false;

    for (int i=0;i<2;i++) {

        c.pwm[i]=c.base[i]=c.effective[i]=0;

        pi_controller_init(&c.pi[i]);

        memset(&c.result[i],0,sizeof(c.result[i]));

    }

    c.trim=0;c.accuracy_scale=1.0f;c.governor_scale=1.0f;c.governor_pressure_ms=0;c.lead_cap_rpm=0;c.governor_last_pressure=false;c.position_trim_rpm=0;c.position_cutoff_latched=false;c.distance_priority=false;c.distance_phase=0;c.distance_wheel_done[0]=c.distance_wheel_done[1]=false;

    c.terminal_coasting[0]=c.terminal_coasting[1]=false;c.terminal_learned[0]=c.terminal_learned[1]=false;

}

static inline void d2_fault(D2Core &c, D2Fault f) {

    d2_disable(c); c.fault=f; c.state=D2_FAULT;

}

static inline void d2_stop(D2Core &c) {

    d2_disable(c); if (c.state!=D2_FAULT) c.state=D2_IDLE;

}

static inline void d2_init(D2Core &c, uint32_t ms) {
    memset(&c, 0, sizeof(c));
    // Bank LOW parameters (0.0 - 0.30 m/s: locked and verified on physical robot)
    c.tune[0] = {1.0800f, 1.1800f, 0.5000f, 3.1300f, 45.9000f, 320.0f, 1500.0f};
    c.tune[1] = {1.1100f, 1.2200f, 0.5000f, 3.0780f, 68.3500f, 320.0f, 1500.0f};
    c.diameter[0] = c.diameter[1] = 0.1f;
    c.track = 0.4f;
    // MCU limits & instantaneous cross-coupling sync
    c.max_rpm = 114.592f;   // 0.60 m/s limit at 100mm wheel diameter
    c.max_pwm = 470.0f;     // Bounded 10-bit PWM cap
    c.slew = 27.0f;         // Max PWM change per 20ms tick
    c.sync_kp = 720.0f;     // Instantaneous restoring trim gain
    c.sync_max = 18.0f;     // Max differential speed trim (RPM)
    c.divergence = 0.04f;   // 40 mm divergence guard
    c.direction = 1;
    c.last_step_ms = c.heartbeat_ms = ms;
    c.quiet_ms = ms;
    c.configured = true;    // Pre-configured with verified parameters on boot
    d2_disable(c);
    c.governor_min_scale = 1.0f;

    // Bank HIGH parameters (0.40 - 0.60 m/s: Gain Scheduling enabled)
    c.gains.high[0] = {1.4500f, 0.5500f, 2.0000f, 3.0200f, 44.4000f, 320.0f, 1500.0f};
    c.gains.high[1] = {1.4800f, 0.5800f, 2.0000f, 2.9800f, 65.0000f, 320.0f, 1500.0f};
    c.gains.enabled = true;
    c.gains.transition_s = 0.150f;
    c.gains.staged = 0;
    c.gains.alpha = c.gains.target_alpha = 0;
    c.gains.active_for_run = false;
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
                                  uint32_t sequence,uint32_t ms) {
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

    c.low_startup_s=0;
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

static inline bool d2_start_distance(D2Core &c,float distance,float speed_mps,float deadline_s,uint32_t ms,uint32_t run_id) {

    if(!isfinite(distance)||!isfinite(speed_mps)||!isfinite(deadline_s)||speed_mps<.01f||speed_mps>.6f||deadline_s<.5f||deadline_s>60.0f)return false;

    float nominal=fabsf(distance)/speed_mps+.5f;

    if(nominal>deadline_s || nominal>60.0f)return false;

    float ramp=fminf(.5f,nominal*.25f);

    for(int i=0;i<2;i++) {

        float rpm=speed_mps*60/(D2_PI*c.diameter[i]);

        if(rpm>c.max_rpm||rpm/ramp>c.tune[i].ramp)return false;

    }

    if(!d2_start(c,0,distance,nominal,ms,run_id))return false;

    c.duration_s=deadline_s;c.cruise_mps=speed_mps;c.distance_priority=true;
    // Latch on admission, so live speed changes cannot re-arm this envelope.
    c.low_startup_s=c.ramp_s*(speed_mps<=.300001f?D2_LOW_STARTUP_SCALE:1.0f);
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

static inline bool d2_start_turn_time(D2Core &c,float angle_rad,float seconds,uint32_t ms,uint32_t run_id) {

    if(!isfinite(angle_rad)||fabsf(angle_rad)<D2_MIN_TURN_RAD||fabsf(angle_rad)>2*D2_PI)return false;

    return d2_start_timed(c,angle_rad*c.track*.5f,seconds,true,ms,run_id);

}

static inline void d2_soft_stop(D2Core &c,uint32_t ms) {

    if(d2_running(c)&&c.state!=D2_STOPPING){c.state=D2_STOPPING;c.stop_ms=ms;}

}

static inline bool d2_clear(D2Core &c,uint32_t ms,bool timer_healthy) {

    if(!timer_healthy||!d2_stationary(c,ms)||d2_running(c))return false;

    d2_stop(c);c.fault=D2_NONE;c.state=D2_IDLE;c.heartbeat_ms=ms;return true;

}

static inline void d2_step(D2Core &c,int64_t raw1,int64_t raw2,uint32_t ms) {

    uint32_t dt=ms-c.last_step_ms;if(dt<20)return;

    c.last_step_ms=ms;if(dt>c.max_dt_ms)c.max_dt_ms=dt;

    int64_t raw[2]={raw1,raw2};

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

    for(int i=0;i<2;i++)c.history[i][slot]=c.counts[i];

    c.history_ms[slot]=ms;c.history_pos=(slot+1)%6;if(c.history_size<6)c.history_size++;

    unsigned oldest=c.history_size<6?0:c.history_pos;

    uint32_t window=ms-c.history_ms[oldest];

    for(int i=0;i<2;i++) {

        if(window) {

            c.raw_rpm[i]=(float)(c.counts[i]-c.history[i][oldest])*60000/(D2_CPR*window);

            c.rpm[i]=.6f*c.raw_rpm[i]+.4f*c.rpm[i];

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

    float error=d2_wheel_direction(c,0)*c.distance[0]-d2_wheel_direction(c,1)*c.distance[1];

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

    // Disable outputs at deadline; the host checks final distance only after standstill.

    // Existing 30 mm overtravel and duration+1s limits remain active in STOPPING.

    if(elapsed>=c.duration_s){if(c.distance_priority && c.state==D2_MOVE)c.distance_stop_reason=2;d2_soft_stop(c,ms);}

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

        for(int i=0;i<2;i++) {

            float speed=fmaxf(0,d2_wheel_direction(c,i)*tick_mps[i]);

            float wheel_progress=d2_wheel_direction(c,i)*c.distance[i];

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

        if(done){c.position_cutoff_latched=true;c.distance_stop_reason=1;d2_soft_stop(c,ms);v=0;}

    } else if(c.state==D2_MOVE && c.distance_priority) {

        // Experimental: measured progress limits the speed; deadline and all

        // hard guards remain in force.  No reverse or automatic second move.

        float remaining=fmaxf(0.0f,c.target_m-progress);

        float measured_mps=fmaxf(0.0f,(d2_wheel_direction(c,0)*c.rpm[0]+d2_wheel_direction(c,1)*c.rpm[1])*.5f)*D2_PI*c.diameter[0]/60;

        float margin=fminf(.05f,measured_mps*.25f);

        c.position_ref_m=c.target_m;c.position_trim_rpm=0;

        // Complete only when both wheels have reached the terminal band.

        // Average distance alone can hide a wheel that remains short.

        bool both_reached=true;

        for(int i=0;i<2;i++)if(d2_wheel_direction(c,i)*c.distance[i]<c.target_m-.0005f)both_reached=false;

        if(both_reached) {

            c.position_cutoff_latched=true;c.distance_stop_reason=1;d2_soft_stop(c,ms);v=0;

        } else if(c.distance_phase==1) {

            v=0;

            // Do not restart while the robot is coasting. The same move,

            // watchdog, deadline and fault guards remain in force.

            if(fabsf(c.rpm[0])<1.5f && fabsf(c.rpm[1])<1.5f &&

               c.pwm[0]<1 && c.pwm[1]<1 && (uint32_t)(ms-c.quiet_ms)>=200)c.distance_phase=2;

        } else if(c.distance_phase==2) {

            v=fminf(c.cruise_mps,.03f);

        } else if(remaining<=fmaxf(.0005f,margin)) {

            c.distance_phase=1;v=0;

        } else {

            // Reserve a bounded distance for sensing/control lag before applying

            // the deceleration envelope. 0.20 s is a conservative hypothesis,

            // not a calibrated measurement or a guarantee of stopping distance.

            float preview_m=fminf(.05f,measured_mps*.20f);

            v=fminf(v,sqrtf(.30f*fmaxf(0.0f,remaining-preview_m)));

            float startup_s=fmaxf(c.ramp_s,c.low_startup_s);
            if(elapsed<startup_s)v*=elapsed/startup_s;

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

        if(c.time_priority&&c.state==D2_MOVE) {

            // Each wheel tracks the scheduled position. Configured Ksync

            // adds bounded cross-coupling to reject unequal motor response;

            // it must not replace independent recovery of lost wheel travel.

            float wheel_progress=d2_wheel_direction(c,i)*c.distance[i];

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

        }

        if(c.state==D2_MOVE && c.distance_priority && (c.distance_phase==2||c.time_priority)) {

            // Coast a wheel once it reaches its own target. Latch until the

            // move ends so noise cannot restart it. No reversal is permitted.

            if(!c.time_priority&&d2_wheel_direction(c,i)*c.distance[i]>=c.target_m-.0005f)c.distance_wheel_done[i]=true;

            if(c.distance_wheel_done[i]) {

                c.base[i]=c.effective[i]=c.pwm[i]=0;

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

        c.base[i]=d2_wheel_direction(c,i)*(c.time_priority?target:base);

        float shaped=decel_tail_shaper_step(fabsf(c.effective[i]),target,c.tune[i].ramp,c.tune[i].jerk,delta_s);

        c.effective[i]=d2_wheel_direction(c,i)*shaped;

        pi_controller_config_t cfg={};

        D2Tune t=d2_gs_tune(c,i,c.gains.alpha);
        D2Tune previous_t=d2_gs_tune(c,i,previous_gain_alpha);
        d2_gs_rebase(c,i,previous_t,t,shaped,d2_wheel_direction(c,i)*c.rpm[i]);
        cfg.kp=t.kp;cfg.ki=t.ki;cfg.kaw=t.kaw;cfg.ts_s=delta_s;

        cfg.mode=AW_MODE_TRACKING;cfg.ff_enabled=true;cfg.ff_slope=t.slope;cfg.ff_intercept=t.offset;

        cfg.max_pwm=c.max_pwm;cfg.min_pwm=0;cfg.slew_counts_per_tick=c.slew;

        if(!pi_controller_step(&cfg,&c.pi[i],target,shaped,d2_wheel_direction(c,i)*c.rpm[i],&c.result[i])) {

            d2_fault(c,D2_CONFIG);return;

        }

        c.pwm[i]=c.pi[i].applied_pwm;

        if(c.pwm[i]>40 && d2_wheel_direction(c,i)*c.rpm[i]<-3){d2_fault(c,D2_POLARITY);return;}

        // Creep can demand less than 5 RPM after wheel synchronization;

        // keep the stall guard active for a genuine nonzero creep command.

        bool stall_command=shaped>5 || (c.distance_priority && (c.distance_phase==2||c.time_priority) && shaped>1);

        if(stall_command && c.pwm[i]>40 && fabsf(c.rpm[i])<1) {

            c.stall_ms[i]+=(uint16_t)dt;

            if(c.stall_ms[i]>=800){

                if(c.distance_priority && c.distance_phase==2) {

                    c.position_cutoff_latched=true;c.distance_stop_reason=1;d2_soft_stop(c,ms);return;

                }

                d2_fault(c,i==0?D2_STALL1:D2_STALL2);return;

            }

        } else c.stall_ms[i]=0;

    }

    if(c.state==D2_STOPPING && ((c.pwm[0]<1&&c.pwm[1]<1&&fabsf(c.rpm[0])<1.5f&&fabsf(c.rpm[1])<1.5f)||ms-c.stop_ms>=500))d2_stop(c);

}

// Mutually exclusive bridge legs; reverse direction changes only after stationary admission.

static inline void d2_legs(float pwm,int sign,uint16_t &a,uint16_t &b) {

    uint16_t duty=(uint16_t)(d2_clamp(pwm,0,500)+.5f);a=sign>0?duty:0;b=sign<0?duty:0;

}

