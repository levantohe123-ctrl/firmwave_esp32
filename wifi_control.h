#pragma once
#include "manual_drive.h"

enum class RemoteOp : uint8_t { Claim, Pulse, Stop, Move, Turn, Jog, Calibrate, Clear, Drive, Release };
struct RemoteCommand {
    RemoteOp op;
    uint64_t owner;
    uint32_t sequence, revision, received_ms, ticket, seen_ms;
    float distance, speed, angle, seconds, throttle, steer;
    int direction;
    bool held;
};
struct RemoteSession {
    uint64_t owner=0;
    uint32_t sequence=0,revision=1,last_pulse_ms=0,run=0;
    bool manual=false;
    ManualDrive drive;
    void stop(D2Core &c) {drive.stop(c,true);manual=false;++revision;}
    bool expire(D2Core &c,uint32_t ms) {
        // Connection ownership never expires while idle. Only moving outputs
        // stop if the browser ceases to supply fresh input; connection stays.
        if(owner&&d2_running(c)&&uint32_t(ms-last_pulse_ms)>400) {
            drive.stop(c,true);manual=false;++revision;return true;
        }
        return false;
    }
    const char *apply(D2Core &c,const RemoteCommand &q,uint32_t ms,
                      const ImuObserver &im,bool timer_ok,bool imu_busy) {
        if(q.op==RemoteOp::Stop){stop(c);if(q.owner==owner&&q.sequence>sequence)sequence=q.sequence;return "ok";}
        if(q.op==RemoteOp::Release) {
            if(q.owner!=owner)return "ignored";
            stop(c);owner=0;sequence=0;return "ok";
        }
        if(q.op==RemoteOp::Claim) {
            if(!q.owner)return "invalid_session";
            if(owner&&owner!=q.owner)return "controller_busy";
            owner=q.owner;sequence=0;last_pulse_ms=ms;return "ok";
        }
        if(owner!=q.owner||!owner)return "not_owner";
        if(q.sequence<=sequence)return "ignored";
        sequence=q.sequence;
        if(uint32_t(ms-q.received_ms)>150)return "ignored";
        if(q.op==RemoteOp::Drive||q.op==RemoteOp::Pulse) {
            int32_t age=(int32_t)(ms-q.seen_ms);
            if(age>500||age< -100)return "ignored";
            if(q.op==RemoteOp::Drive) {
                bool neutral=fabsf(q.throttle)<.001f&&fabsf(q.steer)<.001f;
                if(!neutral&&(!timer_ok||imu_busy))return "not_ready";
                if(!drive.command(c,q.throttle,q.steer,q.speed,ms))return c.fault!=D2_NONE?"motor_fault":"neutral_required";
                manual=drive.active;
            }
            last_pulse_ms=ms;c.heartbeat_ms=ms;return "ok";
        }
        if(q.op==RemoteOp::Clear)return d2_clear(c,ms,timer_ok)?"ok":"cannot_clear";
        if(drive.active||c.state!=D2_IDLE||!d2_stationary(c,ms)||c.fault!=D2_NONE||!timer_ok||imu_busy)return "not_ready";
        if(q.op==RemoteOp::Calibrate)return "calibrate";
        if(!d2_high_imu_ready(&im,ms))return "imu_not_ready";
        uint32_t next=run+1;if(!next)next=1;
        bool ok=false;
        if(q.op==RemoteOp::Move&&(q.direction==1||q.direction==-1)&&isfinite(q.distance)&&
           q.distance>=.05f&&q.distance<=5&&isfinite(q.speed)&&q.speed>=.1f&&q.speed<=.6f) {
            ok=d2_start_distance(c,q.direction*q.distance,q.speed,fminf(60,q.distance/q.speed+8),ms,next,&im);
        } else if(q.op==RemoteOp::Turn&&(q.direction==2||q.direction==-2)&&isfinite(q.angle)&&
                  q.angle>=5&&q.angle<=360&&isfinite(q.seconds)&&q.seconds>=2&&q.seconds<=60) {
            ok=d2_start_turn_time(c,(q.direction>0?1:-1)*q.angle*D2_PI/180,q.seconds,ms,next,&im);
        }
        if(!ok)return "command_rejected";
        run=next;last_pulse_ms=ms;return "ok";
    }
};
