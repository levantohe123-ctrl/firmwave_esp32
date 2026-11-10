#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>
#ifdef NDEBUG
#error Build with assertions enabled
#endif
#include "../pi_aw_controller.h"
#include "../decel_tail_shaper.h"
#include "../imu_observer.h"
#ifdef C8_REFERENCE_HEADER
namespace reference {
#include C8_REFERENCE_HEADER
}
#undef D2_BUILD
#endif
#include "../dual_v2_core.h"

static unsigned cases=0, paired_ticks=0;
static ImuObserver sensor(unsigned ms, float yaw=0) {
    ImuObserver im={}; im.connected=im.calibrated=im.valid=true;
    im.calibration_id=1; im.sample_ms=ms; im.rpy[2]=yaw; im.q[0]=1; return im;
}
static D2Core ready() {
    D2Core c; d2_init(c,0);
    for(unsigned ms=20;ms<=300;ms+=20){c.heartbeat_ms=ms;d2_step(c,0,0,ms);}
    assert(d2_stationary(c,300)); return c;
}
static void admission_and_safety() {
    auto c=ready(); assert(c.configured&&c.gains.enabled&&c.turn_feedback.enabled);
    assert(c.high_ctrl.mode==2&&!c.outputs&&c.pwm[0]==0&&c.pwm[1]==0);
    auto im=sensor(300);
    assert(!d2_start_distance(c,1.2f,.4f,10,300,1)); // HIGH cannot run blind.
    assert(!d2_start_turn_time(c,D2_PI,7,300,1));
    assert(!d2_valid_high_ctrl(0,.5f,1,.15f,.15f,.75f));
    assert(!d2_valid_high_ctrl(1,.5f,1,.15f,.15f,.75f));
    assert(d2_valid_high_ctrl(2,.5f,1,.15f,.15f,.75f)); cases+=6;
    for(float speed:{.1f,.2f,.3f,.35f,.4f,.5f,.6f}) for(int dir:{-1,1}) {
        c=ready(); im=sensor(300);
        assert(d2_start_distance(c,dir*1.2f,speed,30,300,7,&im));
        assert(c.gains.alpha==d2_gs_weight(speed));
        assert(c.high_ctrl.used_in_run==(speed>.30f));
        if(speed>.3f) {
            auto stale=sensor(400); stale.sample_ms=300;
            c.heartbeat_ms=400; // Use several normal ticks: a 100ms gap is a timing fault.
            for(unsigned ms=320;ms<=400;ms+=20){c.heartbeat_ms=ms;d2_step(c,0,0,ms,&stale);}
            assert(!c.high_ctrl.active||c.state!=D2_MOVE);
        }
        c=ready(); im=sensor(300);
        assert(d2_start_distance(c,dir*1.2f,speed,30,300,7,&im));
        // Feed normal cadence; withhold heartbeat only. Watchdog must cut outputs.
        for(unsigned ms=320;ms<=800;ms+=20){im=sensor(ms);d2_step(c,0,0,ms,&im);}
        assert(c.state==D2_FAULT&&c.fault==D2_WATCHDOG&&!c.outputs);
        assert(c.pwm[0]==0&&c.pwm[1]==0); cases+=2;
    }
    c=ready(); im=sensor(300);
    assert(d2_start_distance(c,1.2f,.3f,20,300,9,&im));
    assert(!d2_update_speed(c,.4f,8,1,320,&im)); // Wrong run must not update.
    im=sensor(320); assert(d2_update_speed(c,.4f,9,1,320,&im));
    assert(!d2_update_speed(c,.5f,9,1,320,&im)); // No sequence replay.
    d2_stop(c); assert(!c.outputs&&c.pwm[0]==0&&c.pwm[1]==0); cases+=4;
    // A new run must preserve an explicit idle tuning transaction.
    c=ready(); im=sensor(300);
    D2Tune tuned=c.gains.high[0]; tuned.kp+=.1f;
    assert(d2_gs_stage(c,0,tuned,300));
    assert(d2_gs_stage(c,1,c.gains.high[1],300));
    assert(d2_gs_enable(c,true,200,300));
    assert(d2_start_distance(c,1.2f,.4f,10,300,11,&im));
    assert(c.gains.high[0].kp==tuned.kp && fabsf(c.gains.transition_s-.2f)<1e-6f);
    cases++;
    for(int dir:{-1,1}) {
        c=ready(); im=sensor(300); assert(d2_start_turn_time(c,dir*D2_PI,7,300,3,&im));
        assert(c.turn_feedback.used_in_run&&!c.gains.active_for_run);
        im=sensor(320); im.valid=false; c.heartbeat_ms=320; d2_step(c,0,0,320,&im);
        assert(c.state==D2_STOPPING||c.state==D2_IDLE);
        assert(c.turn_feedback.stop_reason==3); cases++;
    }
}

#ifdef C8_REFERENCE_HEADER
static void configure_reference(reference::D2Core &c) {
    reference::d2_init(c,0);
    for(int i=0;i<2;i++) {
        const float *a=robot_config::LOW_TUNING[i], *b=robot_config::HIGH_TUNING[i];
        c.tune[i]={a[0],a[1],a[2],a[3],a[4],a[5],a[6]};
        c.gains.high[i]={b[0],b[1],b[2],b[3],b[4],b[5],b[6]};
    }
    assert(reference::d2_limits(c,114.592f,470,27,720,18,.04f));
    c.configured=true;c.gains.enabled=true;c.gains.transition_s=.15f;
    c.high_ctrl.mode=2;c.turn_feedback.enabled=true;
    for(unsigned ms=20;ms<=300;ms+=20){c.heartbeat_ms=ms;reference::d2_step(c,0,0,ms);}
}
static void same(const reference::D2Core &a,const D2Core &b) {
    assert(int(a.state)==int(b.state)&&int(a.fault)==int(b.fault));
    assert(a.outputs==b.outputs&&a.direction==b.direction&&a.turning==b.turning);
    assert(a.distance_phase==b.distance_phase&&a.distance_stop_reason==b.distance_stop_reason);
#define SAME(field) assert(!std::memcmp(&a.field,&b.field,sizeof(a.field)))
    SAME(distance);SAME(rpm);SAME(raw_rpm);SAME(effective);SAME(pwm);SAME(base);
    SAME(trim);SAME(common_v);SAME(governor_scale);SAME(observed_reference_rpm);
    SAME(position_ref_m);SAME(trajectory_velocity_mps);SAME(terminal_coasting);
    SAME(distance_wheel_done);SAME(high_ctrl);SAME(turn_feedback);
#undef SAME
    for(int i=0;i<2;i++) {
        assert(a.result[i].proportional==b.result[i].proportional);
        assert(a.result[i].integral==b.result[i].integral);
        assert(a.result[i].feedforward==b.result[i].feedforward);
    }
    paired_ticks++;
}
static void equivalence() {
    for(int dir:{-1,1}) for(float speed:{.1f,.2f,.3f,.35f,.4f,.5f,.6f})
    for(bool dynamic:{false,true}) {
        reference::D2Core a; configure_reference(a); auto b=ready(); auto im=sensor(300);
        assert(reference::d2_start_distance(a,dir*1.2f,speed,30,300,7,&im));
        assert(d2_start_distance(b,dir*1.2f,speed,30,300,7,&im));
        double count[2]={};
        for(unsigned k=1;k<=1600;k++) {
            unsigned ms=300+k*20; float t=k*.02f;
            count[0]+=dir*speed*.5f*(1+std::sin(t*1.3f))*.02f/(D2_PI*.1f)*D2_CPR;
            count[1]+=dir*speed*.49f*(1+std::sin(t*1.2f))*.02f/(D2_PI*.1f)*D2_CPR;
            im=sensor(ms,.01f*std::sin(t)); a.heartbeat_ms=b.heartbeat_ms=ms;
            if(dynamic&&(k==50||k==100||k==150)) {
                float v=k==50?.5f:k==100?.2f:.6f;
                assert(reference::d2_update_speed(a,v,7,k/50,ms,&im)==d2_update_speed(b,v,7,k/50,ms,&im));
            }
            reference::d2_step(a,-llround(count[0]),llround(count[1]),ms,&im);
            d2_step(b,-llround(count[0]),llround(count[1]),ms,&im); same(a,b);
        }
        cases++;
    }
    for(int dir:{-1,1}) for(float degrees:{5.f,90.f,180.f,360.f}) {
        reference::D2Core a;configure_reference(a);auto b=ready();auto im=sensor(300);
        float angle=dir*degrees*D2_PI/180;float seconds=degrees==360?12:7;
        assert(reference::d2_start_turn_time(a,angle,seconds,300,7,&im));
        assert(d2_start_turn_time(b,angle,seconds,300,7,&im));
        for(unsigned k=1;k<=800;k++) {
            unsigned ms=300+k*20; float yaw=angle*fminf(k*.02f/seconds,1);
            long n=lroundf(yaw*.2f/(D2_PI*.1f)*D2_CPR);
            im=sensor(ms,yaw);a.heartbeat_ms=b.heartbeat_ms=ms;
            reference::d2_step(a,n,n,ms,&im);d2_step(b,n,n,ms,&im);same(a,b);
        }
        cases++;
    }
}
#endif
int main() {
    admission_and_safety();
#ifdef C8_REFERENCE_HEADER
    equivalence();
#endif
    std::printf("CONTROL_CORE_PASS cases=%u paired_ticks=%u\n",cases,paired_ticks);
}
