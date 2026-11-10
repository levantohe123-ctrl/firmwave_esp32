#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <initializer_list>
#ifdef NDEBUG
#error Assertions required
#endif
#include "../dual_v2_core.h"
static unsigned cases=0;
static ImuObserver sensor(unsigned ms) {
    ImuObserver im={};im.connected=im.calibrated=im.valid=true;
    im.calibration_id=1;im.sample_ms=ms;return im;
}
static D2Core ready(int direction=1) {
    D2Core c;d2_init(c,0);c.configured=true;
    for(unsigned ms=20;ms<=300;ms+=20){c.heartbeat_ms=ms;d2_step(c,0,0,ms);}
    auto im=sensor(300);c.turn_feedback.enabled=true;
    assert(d2_start_turn_time(c,direction*D2_PI,7,300,123,&im));
    c.start_ms=0;c.last_step_ms=6200;c.heartbeat_ms=6200;
    auto &t=c.turn_feedback;t.finish_phase=1;t.cutoff_ms=t.last_edge_ms=6200;
    t.angle_rad=direction*(D2_PI-D2_PI/180);t.yaw_rate=0;
    c.outputs=false;c.pwm[0]=c.pwm[1]=0;return c;
}
static void pulse_guards() {
    for(int dir:{-1,1}) {
        auto c=ready(dir);auto &t=c.turn_feedback;
        assert(!d2_turn_finish_tick(c,6300)&&t.correction_used&&t.finish_phase==2&&c.outputs);
        assert(t.pulse_ms>=40&&t.pulse_ms<=140);int unchanged_direction=c.direction;
        unsigned pulse=t.pulse_ms;
        assert(!d2_turn_finish_tick(c,6300+pulse)&&t.finish_phase==3&&!c.outputs);
        assert(d2_turn_finish_tick(c,6500)&&t.correction_used&&c.direction==unchanged_direction&&!c.outputs);cases++;
        d2_stop(c);assert(d2_start_distance(c,.3f,.3f,10,7000,124));
        assert(!c.turn_feedback.used_in_run&&!c.turn_feedback.correction_used&&c.turn_feedback.finish_phase==0);cases++;
        c=ready(dir);c.turn_feedback.angle_rad=dir*(D2_PI+.01f);
        assert(d2_turn_finish_tick(c,6300)&&!c.turn_feedback.correction_used&&!c.outputs);cases++;
        c=ready(dir);assert(d2_turn_finish_tick(c,6950)&&!c.turn_feedback.correction_used&&!c.outputs);cases++;
        c=ready(dir);c.turn_feedback.angle_rad=dir*(D2_PI-.001f);
        assert(d2_turn_finish_tick(c,6300)&&!c.turn_feedback.correction_used);cases++;
        c=ready(dir);c.turn_feedback.angle_rad=dir*(D2_PI-.05f);
        assert(d2_turn_finish_tick(c,6300)&&c.turn_feedback.stop_reason==5&&!c.outputs);cases++;
        c=ready(dir);c.rpm[0]=2;
        assert(!d2_turn_finish_tick(c,6300)&&!c.turn_feedback.correction_used);cases++;
        c=ready(dir);c.counts[0]++;
        assert(!d2_turn_finish_tick(c,6300)&&!c.turn_feedback.correction_used);cases++;
        c=ready(dir);c.turn_feedback.yaw_rate=.1f;
        assert(!d2_turn_finish_tick(c,6300)&&!c.turn_feedback.correction_used);
        assert(d2_turn_finish_tick(c,6920)&&c.turn_feedback.stop_reason==5&&!c.outputs);cases++;
    }
}
static void actual_core_safety() {
    for(int kind:{0,1,2,3,4}) {
        auto c=ready(1);c.turn_feedback.finish_phase=2;c.turn_feedback.correction_used=true;
        c.turn_feedback.correction_ms=6200;c.turn_feedback.pulse_ms=140;c.outputs=true;
        c.pwm[0]=c.pwm[1]=100;
        long n=lroundf((D2_PI-.01f)*.2f/(D2_PI*.1f)*D2_CPR);
        c.counts[0]=n;c.counts[1]=-n;
        for(int i=0;i<2;i++)for(int j=0;j<6;j++)c.history[i][j]=c.counts[i];
        c.turn_feedback.offset_m=0;
        auto im=sensor(6220);im.rpy[2]=D2_PI-.01f;
        if(kind==0)d2_stop(c);
        if(kind==1)im.valid=false;
        if(kind==2)im.calibration_id=2;
        if(kind==3)c.heartbeat_ms=0;
        if(kind==4){c.start_ms=0;c.last_step_ms=6980;im.sample_ms=7000;}
        unsigned ms=kind==4?7000:6220;
        if(kind!=3)c.heartbeat_ms=ms;
        d2_step(c,-n,-n,ms,&im);
        if(kind==0||kind==3||kind==4)assert(!c.outputs&&c.pwm[0]==0&&c.pwm[1]==0);
        if(kind==1||kind==2){assert(c.state==D2_STOPPING||c.state==D2_IDLE);assert(c.turn_feedback.stop_reason==3);}
        assert(!c.turn_feedback.correction_used||c.turn_feedback.pulse_ms<=140);cases++;
    }
}
int main(){pulse_guards();actual_core_safety();printf("C8_TURN_GUARDS_PASS cases=%u\n",cases);}
