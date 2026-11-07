#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

static inline bool imu_supported_chip(uint8_t who) { return who==0x68||who==0x70; }
static inline float imu_temperature_c(int16_t raw,uint8_t who) {
    return who==0x70?raw/333.87f+21.0f:raw/340.0f+36.53f;
}

// Observe only. Body axes: X forward, Y left, Z up. No dependency on motor PI.
struct ImuObserver {
    float q[4], accel[3], gyro[3], bias[3], rpy[3], temperature;
    double mean[3], m2[3];
    uint32_t sample_ms, sample_us, sequence, calibration_id, calibration_start_ms;
    unsigned calibration_samples;
    bool connected, calibrated, calibrating, valid;
    uint8_t error; // 1 I2C, 2 identity, 3 gap, 4 motion, 5 unstable, 6 task, 7 clip
    uint8_t probe68, probe69, whoami, config_reg; // Passive connection diagnostics.
    float last_wrapped_yaw;
};
static inline void imu_init(ImuObserver &s) { memset(&s,0,sizeof(s));s.q[0]=1; }
static inline void imu_fail(ImuObserver &s,uint8_t error) {
    s.valid=false;s.error=error;
    if(s.calibrating){s.calibrating=false;s.calibrated=false;}
}
static inline bool imu_begin_calibration(ImuObserver &s,uint32_t ms,bool stationary) {
    if(!s.connected||s.calibrating||!stationary)return false;
    s.calibrating=true;s.calibrated=s.valid=false;s.error=0;
    s.calibration_samples=0;s.calibration_start_ms=ms;
    memset(s.mean,0,sizeof(s.mean));memset(s.m2,0,sizeof(s.m2));return true;
}
static inline void imu_rpy(ImuObserver &s,bool reset_yaw=false) {
    float w=s.q[0],x=s.q[1],y=s.q[2],z=s.q[3];
    s.rpy[0]=atan2f(2*(w*x+y*z),1-2*(x*x+y*y));
    s.rpy[1]=asinf(fmaxf(-1,fminf(1,2*(w*y-z*x))));
    float yaw=atan2f(2*(w*z+x*y),1-2*(y*y+z*z));
    float delta=yaw-s.last_wrapped_yaw;
    if(delta>3.14159265f)delta-=6.2831853f;
    if(delta< -3.14159265f)delta+=6.2831853f;
    s.rpy[2]=reset_yaw?0:s.rpy[2]+delta;s.last_wrapped_yaw=yaw;
}
static inline void imu_sample(ImuObserver &s,const float accel[3],const float rate[3],
                              float temp,uint32_t us,uint32_t ms,bool stationary) {
    uint32_t delta=us-s.sample_us;
    bool first=s.sequence==0;
    s.sample_us=us;s.sample_ms=ms;s.sequence++;s.connected=true;
    float norm2=0;
    for(int i=0;i<3;i++) {
        if(!isfinite(accel[i])||!isfinite(rate[i])){imu_fail(s,7);return;}
        s.accel[i]=accel[i];s.gyro[i]=rate[i]-s.bias[i];norm2+=accel[i]*accel[i];
        if(fabsf(accel[i])>38.0f||fabsf(rate[i])>8.5f){imu_fail(s,7);s.calibrated=false;return;}
    }
    s.temperature=temp;
    float norm=sqrtf(norm2);
    if(s.calibrating) {
        if(!stationary){imu_fail(s,4);return;}
        if((uint32_t)(ms-s.calibration_start_ms)>10000){imu_fail(s,5);return;}
        // Allow untrimmed accelerometer offset; this is a plausibility gate,
        // not a claim that acceleration proves zero angular motion.
        bool still=norm>8.83f&&norm<10.79f&&accel[2]>0;
        if(!first&&delta>40000)still=false;
        // Datasheet zero-rate offset can be large before calibration. Wheel
        // standstill, low variance and the operator's stationary setup matter
        // more than imposing a tiny pre-calibration gyro reading.
        for(int i=0;i<3;i++)if(fabsf(rate[i])>.349066f)still=false;
        if(!still){s.calibration_samples=0;memset(s.mean,0,sizeof(s.mean));memset(s.m2,0,sizeof(s.m2));return;}
        unsigned n=++s.calibration_samples;
        for(int i=0;i<3;i++) {
            double d=rate[i]-s.mean[i];s.mean[i]+=d/n;s.m2[i]+=d*(rate[i]-s.mean[i]);
        }
        if(n>=300 && (uint32_t)(ms-s.calibration_start_ms)>=2900) {
            bool stable=true;
            for(int i=0;i<3;i++)if(s.m2[i]/(n-1)>.00001904)stable=false; // std <= .25 deg/s
            if(!stable){imu_fail(s,5);return;}
            for(int i=0;i<3;i++){s.bias[i]=(float)s.mean[i];s.gyro[i]=rate[i]-s.bias[i];}
            float roll=atan2f(accel[1],accel[2]);
            float pitch=atan2f(-accel[0],sqrtf(accel[1]*accel[1]+accel[2]*accel[2]));
            s.q[0]=cosf(roll*.5f)*cosf(pitch*.5f);
            s.q[1]=sinf(roll*.5f)*cosf(pitch*.5f);
            s.q[2]=cosf(roll*.5f)*sinf(pitch*.5f);
            s.q[3]=-sinf(roll*.5f)*sinf(pitch*.5f);
            s.calibrating=false;s.calibrated=s.valid=true;s.error=0;s.calibration_id++;
            imu_rpy(s,true);
        }
        return;
    }
    if(!s.calibrated)return;
    // Missed orientation cannot be recovered by silently resuming integration.
    if(first||delta<1000||delta>40000){imu_fail(s,3);s.calibrated=false;return;}
    float gx=s.gyro[0],gy=s.gyro[1],gz=s.gyro[2];
    float w=s.q[0],x=s.q[1],y=s.q[2],z=s.q[3];
    // Mahony proportional gravity correction; no magnetometer and no yaw lock.
    // Reject large acceleration-magnitude excursions rather than interpreting
    // every acceleration as a tilt. Dynamic tilt still requires real validation.
    if(norm>8.83f&&norm<10.79f) {
        float ax=accel[0]/norm,ay=accel[1]/norm,az=accel[2]/norm;
        float vx=2*(x*z-w*y),vy=2*(w*x+y*z),vz=w*w-x*x-y*y+z*z;
        gx+=2*(ay*vz-az*vy);gy+=2*(az*vx-ax*vz);gz+=2*(ax*vy-ay*vx);
    }
    float half_dt=delta*.0000005f;
    s.q[0]+=(-x*gx-y*gy-z*gz)*half_dt;
    s.q[1]+=(w*gx+y*gz-z*gy)*half_dt;
    s.q[2]+=(w*gy-x*gz+z*gx)*half_dt;
    s.q[3]+=(w*gz+x*gy-y*gx)*half_dt;
    float qnorm=0;for(int i=0;i<4;i++)qnorm+=s.q[i]*s.q[i];qnorm=sqrtf(qnorm);
    if(!isfinite(qnorm)||qnorm<.1f){imu_fail(s,7);s.calibrated=false;return;}
    for(int i=0;i<4;i++)s.q[i]/=qnorm;
    imu_rpy(s);s.valid=true;s.error=0;
}
