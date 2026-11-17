#pragma once
#include "dual_v2_core.h"

// Continuous wheel-speed remote. Setpoint trajectories stay in the C8 core.
struct ManualDrive {
    bool active=false,neutral_required=false;
    float requested[2]={},reference[2]={};
    int sign[2]={1,1};
    uint32_t last_command_ms=0;
    static void mix(float throttle,float steer,float speed,float track,float out[2]) {
        // Preserve the empirically accepted left/right convention of C8.
        float omega=steer*D2_YAW_SIGN*fminf(1.0f,2*speed/track);
        if(fabsf(throttle)>.01f)omega*=.65f;
        out[0]=throttle*speed-omega*track*.5f;
        out[1]=throttle*speed+omega*track*.5f;
        float scale=fmaxf(1,fmaxf(fabsf(out[0]),fabsf(out[1]))/.6f);
        out[0]/=scale;out[1]/=scale;
    }
    void stop(D2Core &c,bool require_neutral=false) {
        active=false;neutral_required=require_neutral;
        requested[0]=requested[1]=reference[0]=reference[1]=0;d2_stop(c);
    }
    bool command(D2Core &c,float throttle,float steer,float speed,uint32_t ms) {
        if(!isfinite(throttle)||!isfinite(steer)||!isfinite(speed)||fabsf(throttle)>1||
           fabsf(steer)>1||speed<.1f||speed>.6f)return false;
        bool neutral=fabsf(throttle)<.001f&&fabsf(steer)<.001f;
        if(neutral){if(active)stop(c);neutral_required=false;last_command_ms=ms;return true;}
        if(neutral_required||c.fault!=D2_NONE)return false;
        if(!active) {
            d2_stop(c);active=true;c.state=D2_SPEED;c.outputs=true;
            c.start_ms=ms;c.duration_s=60;c.turning=false;c.distance_priority=false;
            c.time_priority=false;c.high_ctrl.active=false;c.high_ctrl.used_in_run=false;
            c.turn_feedback.used_in_run=false;c.gains.alpha=0;c.gains.active_for_run=c.gains.enabled;
            for(int i=0;i<2;i++) {
                c.origin[i]=c.counts[i];c.distance[i]=0;c.base[i]=c.effective[i]=c.pwm[i]=0;
                reference[i]=0;sign[i]=fabsf(c.rpm[i])>3?(c.rpm[i]>0?1:-1):1;
                c.stall_ms[i]=0;pi_controller_init(&c.pi[i]);
            }
        }
        float wheel[2];mix(throttle,steer,speed,c.track,wheel);
        for(int i=0;i<2;i++)requested[i]=wheel[i]*60/(D2_PI*c.diameter[i]);
        c.gains.requested_mps=fmaxf(fabsf(wheel[0]),fabsf(wheel[1]));
        c.gains.target_alpha=d2_gs_weight(c.gains.requested_mps);
        c.heartbeat_ms=last_command_ms=ms;return true;
    }
    bool timed_out(D2Core &c,uint32_t ms) {
        if(active&&uint32_t(ms-last_command_ms)>400){stop(c,true);return true;}
        return false;
    }
    void tick(D2Core &c,int64_t a,int64_t b,uint32_t ms,const ImuObserver *imu) {
        uint32_t dt=ms-c.last_step_ms;
        if(c.fault!=D2_NONE){stop(c,true);return;}
        if(timed_out(c,ms))return;
        // Reuse C8's signed encoder/RPM observer only. Its setpoint control
        // path is left unchanged and never runs concurrently with this loop.
        c.state=D2_IDLE;d2_step(c,a,b,ms,imu);c.state=D2_SPEED;
        if(dt>40){d2_fault(c,D2_TIMING);active=false;return;}
        float seconds=dt*.001f,old_alpha=c.gains.alpha;
        float delta=seconds/fmaxf(.02f,c.gains.transition_s);
        c.gains.alpha+=d2_clamp(c.gains.target_alpha-c.gains.alpha,-delta,delta);
        for(int i=0;i<2;i++) {
            float target=requested[i];int wanted=target<0?-1:1;
            bool reversing=fabsf(target)>.01f&&wanted!=sign[i];
            float magnitude=reversing?0:fabsf(target);
            reference[i]=decel_tail_shaper_step(reference[i],magnitude,c.tune[i].ramp,c.tune[i].jerk,seconds);
            if(reversing&&reference[i]<.05f) {
                reference[i]=0;c.pwm[i]=c.effective[i]=0;pi_controller_init(&c.pi[i]);
                if(fabsf(c.rpm[i])<=3)sign[i]=wanted;
                continue;
            }
            c.base[i]=target;c.effective[i]=sign[i]*reference[i];
            if(reference[i]<.05f){c.pwm[i]=0;pi_controller_init(&c.pi[i]);continue;}
            D2Tune tune=d2_gs_tune(c,i,c.gains.alpha),previous=d2_gs_tune(c,i,old_alpha);
            d2_gs_rebase(c,i,previous,tune,reference[i],sign[i]*c.rpm[i]);
            pi_controller_config_t cfg={};cfg.kp=tune.kp;cfg.ki=tune.ki;cfg.kaw=tune.kaw;
            cfg.ts_s=seconds;cfg.mode=AW_MODE_TRACKING;cfg.ff_enabled=true;
            cfg.ff_slope=tune.slope;cfg.ff_intercept=tune.offset;cfg.max_pwm=c.max_pwm;
            cfg.slew_counts_per_tick=c.slew;
            if(!pi_controller_step(&cfg,&c.pi[i],magnitude,reference[i],sign[i]*c.rpm[i],&c.result[i])) {
                d2_fault(c,D2_CONFIG);active=false;return;
            }
            c.pwm[i]=c.pi[i].applied_pwm;
            if(c.pwm[i]>40&&sign[i]*c.rpm[i]<-3){d2_fault(c,D2_POLARITY);active=false;return;}
            if(reference[i]>5&&c.pwm[i]>40&&fabsf(c.rpm[i])<1)c.stall_ms[i]+=(uint16_t)dt;
            else c.stall_ms[i]=0;
            if(c.stall_ms[i]>=800){d2_fault(c,i?D2_STALL2:D2_STALL1);active=false;return;}
        }
        c.outputs=true;
    }
};
