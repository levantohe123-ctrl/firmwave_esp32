#pragma once
#include <stdint.h>
#include <math.h>

namespace battery_config {
static constexpr float DIVIDER=11.0f; // R3=100k, R4=10k; R5=1k series.
// Single-point field calibration 2026-11-12: PO meter ~24.1 V / GPIO2.19 V,
// contemporaneous 12-sample filtered telemetry mean 24.45325 V (gain=1).
// Approximate local correction; not a calibrated full-range ADC curve.
static constexpr float VOLTAGE_GAIN=.9855f, VOLTAGE_OFFSET=0.0f;
static constexpr uint32_t REST_MS=10000, STABLE_MS=5000, STALE_MS=1500;
static constexpr uint32_t WARNING_MS=5000;
static constexpr float STABLE_BAND_V=.12f;
// Generic Li-ion OCV approximation, NOT a measured curve for this pack.
static constexpr float CELL_V[]={3.20f,3.40f,3.50f,3.60f,3.70f,3.75f,3.80f,3.85f,3.95f,4.05f,4.20f};
static constexpr float SOC[]={0,5,10,20,30,40,50,60,75,90,100};
}

struct BatteryView {
    bool valid=false,soc_valid=false,held=true;
    float voltage=0,percent=0;
    uint32_t sample_ms=0,soc_ms=0;
    uint8_t warning=0; // 0 normal, 1 low, 2 critical (advisory only).
};

static inline bool battery_sample_fresh(uint32_t now,uint32_t sampled) {
    // Core 0 can publish after loop() captured 'now'. Do not turn a 1 ms
    // future timestamp into a 49-day-old sample through unsigned subtraction.
    int32_t age=(int32_t)(now-sampled);
    return age>=-100 && age<=(int32_t)battery_config::STALE_MS;
}

class BatteryEstimator {
    BatteryView value;
    float samples[5]={},voltage=0,stable_low=0,stable_high=0;
    unsigned count=0,index=0;
    bool rest=false,filter_ready=false,stable=false,warning_pending=false;
    uint8_t pending_warning=0;
    uint32_t last_ms=0,rest_ms=0,stable_ms=0,warning_ms=0;
public:
    static float packVoltage(float adc_mv) {
        return adc_mv*.001f*battery_config::DIVIDER*battery_config::VOLTAGE_GAIN+battery_config::VOLTAGE_OFFSET;
    }
    static float percentAt(float pack_v) {
        if(!isfinite(pack_v))return NAN;
        float cell=pack_v/6;
        using namespace battery_config;
        if(cell<=CELL_V[0])return 0;
        for(unsigned i=1;i<sizeof(CELL_V)/sizeof(CELL_V[0]);i++)
            if(cell<=CELL_V[i])return SOC[i-1]+(SOC[i]-SOC[i-1])*(cell-CELL_V[i-1])/(CELL_V[i]-CELL_V[i-1]);
        return 100;
    }
    void update(float adc_mv,bool motor_or_motion,uint32_t ms) {
        using namespace battery_config;
        uint32_t elapsed=uint32_t(ms-last_ms);
        bool gap=value.sample_ms!=0 && elapsed>STALE_MS;
        last_ms=ms;value.sample_ms=ms;value.held=true;
        // Treat missing/railed wiring as unknown, not as a flat battery.
        if(!isfinite(adc_mv)||adc_mv<500||adc_mv>2600) {
            value.valid=false;rest=false;stable=false;count=index=0;
            filter_ready=false;warning_pending=false;return;
        }
        if(gap){rest=false;stable=false;count=index=0;filter_ready=false;warning_pending=false;}
        samples[index]=packVoltage(adc_mv);index=(index+1)%5;
        if(count<5)++count;
        if(count<5){value.valid=false;return;}
        float sorted[5];for(unsigned i=0;i<5;i++)sorted[i]=samples[i];
        for(unsigned i=1;i<5;i++){float v=sorted[i];unsigned j=i;while(j&&sorted[j-1]>v){sorted[j]=sorted[j-1];--j;}sorted[j]=v;}
        float median=sorted[2];
        if(!filter_ready){voltage=median;filter_ready=true;}
        else {float dt=fminf(elapsed*.001f,.5f);voltage+=dt/(1.0f+dt)*(median-voltage);}
        value.voltage=voltage;value.valid=true;
        if(motor_or_motion) {rest=false;stable=false;warning_pending=false;return;}
        if(!rest){rest=true;rest_ms=ms;stable=false;}
        if(!stable){stable=true;stable_ms=ms;stable_low=stable_high=voltage;}
        stable_low=fminf(stable_low,voltage);stable_high=fmaxf(stable_high,voltage);
        if(stable_high-stable_low>STABLE_BAND_V) {
            stable_ms=ms;stable_low=stable_high=voltage;warning_pending=false;
        }
        if(uint32_t(ms-rest_ms)<REST_MS||uint32_t(ms-stable_ms)<STABLE_MS)return;
        value.held=false;
        float target=percentAt(voltage);
        if(!value.soc_valid){value.percent=target;value.soc_valid=true;}
        else {
            // One percentage point per 5 seconds, only with qualified rest data.
            float step=.2f*fminf(elapsed*.001f,.5f);
            value.percent+=fmaxf(-step,fminf(step,target-value.percent));
        }
        value.soc_ms=ms;
        // Warnings use the qualified estimate, not delayed UI smoothing.
        uint8_t next=target<=10?2:target<=20?1:0;
        if(value.warning==2&&target<=13)next=2;
        else if(value.warning>=1&&target<=23&&next<1)next=1;
        if(next==value.warning)warning_pending=false;
        else {
            if(!warning_pending||pending_warning!=next){warning_pending=true;pending_warning=next;warning_ms=ms;}
            if(uint32_t(ms-warning_ms)>=WARNING_MS){value.warning=next;warning_pending=false;}
        }
    }
    BatteryView snapshot(uint32_t ms) const {
        BatteryView result=value;
        if(!battery_sample_fresh(ms,value.sample_ms))result.valid=false;
        return result;
    }
};
