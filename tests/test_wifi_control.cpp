#include <cassert>
#include <cstdio>
#include <cmath>
#include <initializer_list>
#include "../wifi_control.h"
#ifdef NDEBUG
#error Enable assertions
#endif
static unsigned checks=0;
#define CHECK(x) do {assert(x);++checks;} while(0)
static D2Core ready(){D2Core c;d2_init(c,0);for(unsigned ms=20;ms<=300;ms+=20)d2_step(c,0,0,ms);return c;}
static ImuObserver imu(unsigned ms){ImuObserver i={};i.connected=i.calibrated=i.valid=true;i.calibration_id=1;i.sample_ms=ms;i.q[0]=1;return i;}
static RemoteCommand q(RemoteSession &s,RemoteOp op,unsigned ms=300){RemoteCommand v={};v.op=op;v.owner=123;v.sequence=s.sequence+1;v.received_ms=v.seen_ms=ms;v.speed=.3f;v.throttle=1;v.direction=1;v.distance=1.2f;v.angle=180;v.seconds=6;return v;}
static bool apply(RemoteSession &s,D2Core &c,RemoteCommand v,unsigned ms=300){auto i=imu(ms);return !strcmp(s.apply(c,v,ms,i,true,false),"ok");}
static void claim(RemoteSession &s,D2Core &c,unsigned ms=300){CHECK(apply(s,c,q(s,RemoteOp::Claim,ms),ms));}
int main(){
 {auto c=ready();RemoteSession s;claim(s,c);CHECK(!s.expire(c,600000));CHECK(s.owner==123); // idle connection persists
  claim(s,c,600000);CHECK(s.owner==123);}
 {auto c=ready();RemoteSession s;claim(s,c);auto v=q(s,RemoteOp::Claim);v.owner=222;CHECK(!apply(s,c,v));CHECK(s.owner==123);}
 for(float speed:{.1f,.3f,.4f,.6f})for(float throttle:{-1.f,0.f,1.f})for(float steer:{-1.f,0.f,1.f}){
  float w[2];ManualDrive::mix(throttle,steer,speed,.4f,w);
  CHECK(fabsf(w[0])<=.600001f&&fabsf(w[1])<=.600001f);
  auto c=ready();RemoteSession s;claim(s,c);ImuObserver absent={};auto v=q(s,RemoteOp::Drive);v.speed=speed;v.throttle=throttle;v.steer=steer;
  CHECK(!strcmp(s.apply(c,v,300,absent,true,false),"ok"));CHECK(s.drive.active==(throttle!=0||steer!=0));
  CHECK(c.high_ctrl.mode==2&&c.turn_feedback.enabled); // setpoint policy untouched
  if(throttle==0&&steer!=0){auto ref=ready();auto i=imu(300);CHECK(d2_start_turn_time(ref,steer*D2_PI,6,300,1,&i));for(int n=0;n<2;n++)CHECK((w[n]>0?1:-1)==d2_wheel_direction(ref,n));}
 }
 {auto c=ready();RemoteSession s;claim(s,c);CHECK(apply(s,c,q(s,RemoteOp::Drive)));
  auto v=q(s,RemoteOp::Drive,320);v.steer=1;CHECK(apply(s,c,v,320));CHECK(s.drive.requested[0]!=s.drive.requested[1]);
  v=q(s,RemoteOp::Drive,340);v.steer=-1;CHECK(apply(s,c,v,340));CHECK(c.outputs); // no stationary admission on turn update
  CHECK(s.expire(c,741));CHECK(!c.outputs&&s.owner==123&&s.drive.neutral_required);
  v=q(s,RemoteOp::Drive,750);CHECK(!apply(s,c,v,750));CHECK(!c.outputs);
  v=q(s,RemoteOp::Drive,760);v.throttle=0;CHECK(apply(s,c,v,760));CHECK(!s.drive.neutral_required);
  CHECK(apply(s,c,q(s,RemoteOp::Drive,780),780));CHECK(c.outputs); // fresh gesture, same connection
 }
 {auto c=ready();RemoteSession s;claim(s,c);CHECK(apply(s,c,q(s,RemoteOp::Drive)));
  auto v=q(s,RemoteOp::Stop);CHECK(apply(s,c,v));CHECK(!c.outputs);
  v=q(s,RemoteOp::Drive);v.sequence=s.sequence-1;CHECK(!apply(s,c,v));CHECK(!c.outputs);
  v=q(s,RemoteOp::Drive,900);v.seen_ms=300;CHECK(!apply(s,c,v,900));CHECK(!c.outputs); // stale browser cannot replay held input
 }
 {auto c=ready();RemoteSession s;claim(s,c);CHECK(apply(s,c,q(s,RemoteOp::Drive)));
  CHECK(apply(s,c,q(s,RemoteOp::Release)));CHECK(!c.outputs&&!s.owner);
  auto v=q(s,RemoteOp::Drive);CHECK(!apply(s,c,v));}
 {auto c=ready();RemoteSession s;claim(s,c);auto old=q(s,RemoteOp::Drive);
  CHECK(apply(s,c,q(s,RemoteOp::Release)));auto next=q(s,RemoteOp::Claim);next.owner=456;CHECK(apply(s,c,next));
  CHECK(!apply(s,c,old));CHECK(!c.outputs&&s.owner==456);
  auto neutral=q(s,RemoteOp::Drive);neutral.owner=456;neutral.throttle=0;CHECK(apply(s,c,neutral));
  old.op=RemoteOp::Stop;old.sequence=9999;CHECK(apply(s,c,old));CHECK(s.sequence<9999); // old STOP cannot poison new connection numbering
 }
 {auto c=ready();RemoteSession s;claim(s,c);ImuObserver absent={};auto v=q(s,RemoteOp::Move);CHECK(strcmp(s.apply(c,v,300,absent,true,false),"ok"));CHECK(!c.outputs);}
 {auto c=ready();RemoteSession s;claim(s,c);auto v=q(s,RemoteOp::Calibrate);CHECK(!strcmp(s.apply(c,v,300,imu(300),true,false),"calibrate"));}
 // A synthetic encoder follows previous shaped wheel RPM; validates math,
 // limits, steering updates, gain transitions and state, not vehicle dynamics.
 for(float speed:{.1f,.3f,.4f,.6f}){
  auto c=ready();RemoteSession s;claim(s,c);double count[2]={};
  for(unsigned ms=320;ms<=2320;ms+=20){
   auto v=q(s,RemoteOp::Drive,ms);v.speed=speed;v.steer=ms<1000?0:ms<1600?1:-1;CHECK(apply(s,c,v,ms));
   for(int i=0;i<2;i++)count[i]+=s.drive.sign[i]*s.drive.reference[i]*D2_CPR*.02/60;
   auto im=imu(ms);s.drive.tick(c,-(int64_t)llround(count[0]),(int64_t)llround(count[1]),ms,&im);
   CHECK(c.fault==D2_NONE&&s.drive.active);for(int i=0;i<2;i++)CHECK(isfinite(c.pwm[i])&&c.pwm[i]>=0&&c.pwm[i]<=470);
  }
  CHECK(c.gains.active_for_run&&c.gains.alpha>=0&&c.gains.alpha<=1);
  if(speed>=.4f)CHECK(c.gains.alpha>.9f);
 }
 {auto c=ready();ManualDrive d;c.rpm[0]=c.rpm[1]=30;CHECK(d.command(c,-1,0,.3f,300));CHECK(d.sign[0]==1);
  for(unsigned ms=320;ms<=600;ms+=20){d.command(c,-1,0,.3f,ms);d.tick(c,0,0,ms,nullptr);if(d.sign[0]<0)CHECK(fabsf(c.rpm[0])<=3);}
  CHECK(d.sign[0]==-1);}
 {auto c=ready();ManualDrive d;CHECK(d.command(c,1,0,.3f,300));d.tick(c,0,0,350,nullptr);CHECK(c.fault==D2_TIMING&&!d.active&&!c.outputs);}
 {auto c=ready();ManualDrive d;CHECK(!d.command(c,NAN,0,.3f,300));CHECK(!c.outputs);}
 // Service's exact original calibration gates: 300 real-quality input samples.
 {ImuObserver i;imu_init(i);i.connected=true;CHECK(imu_begin_calibration(i,0,true));float a[3]={0,0,9.80665f},g[3]={.01f,-.005f,.002f};unsigned ms=0;
  for(unsigned n=0;n<300;n++){ms+=n%47==0?20:10;imu_sample(i,a,g,25,ms*1000,ms,true);}
  CHECK(i.calibrated&&i.valid&&i.calibration_samples>=300&&i.calibration_id==1);}
 printf("FLEX_REMOTE_PASS checks=%u\n",checks);
}
