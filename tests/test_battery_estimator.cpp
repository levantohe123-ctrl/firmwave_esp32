#include <cassert>
#include <cstdio>
#include <initializer_list>
#include "../battery_estimator.h"
#ifdef NDEBUG
#error Assertions must be enabled
#endif
static unsigned checks=0;
#define CHECK(x) do { assert(x); ++checks; } while(0)
static void feed(BatteryEstimator &b,uint32_t &ms,float volts,bool motion,unsigned duration) {
    float mv=(volts-battery_config::VOLTAGE_OFFSET)*1000/(11*battery_config::VOLTAGE_GAIN);
    for(unsigned t=0;t<duration;t+=50){ms+=50;b.update(mv,motion,ms);}
}
int main() {
    CHECK(battery_sample_fresh(100,101)); // concurrent publisher wins by 1 ms
    CHECK(battery_sample_fresh(20,UINT32_MAX-10)); // wrap, 31 ms old
    CHECK(!battery_sample_fresh(2000,100)); // truly stale
    CHECK(!battery_sample_fresh(100,1000)); // implausible future sample
    CHECK(fabsf(BatteryEstimator::packVoltage(2200)-24.2f*battery_config::VOLTAGE_GAIN)<.0001f);
    CHECK(fabsf(BatteryEstimator::packVoltage(2190.909f)-24.1f*battery_config::VOLTAGE_GAIN)<.001f);
    CHECK(fabsf(BatteryEstimator::packVoltage(2223.023f)-24.1f)<.02f); // measured field pair
    CHECK(BatteryEstimator::percentAt(25.2f)==100);
    CHECK(BatteryEstimator::percentAt(19.2f)==0);
    CHECK(isnan(BatteryEstimator::percentAt(NAN)));
    float prev=-1;for(float v=18;v<=27;v+=.05f){auto p=BatteryEstimator::percentAt(v);CHECK(p>=prev&&p>=0&&p<=100);prev=p;}
    {BatteryEstimator b;uint32_t ms=0;CHECK(!b.snapshot(ms).valid);
     feed(b,ms,24.1f,false,10000);CHECK(!b.snapshot(ms).soc_valid);
     feed(b,ms,24.1f,false,1000);auto s=b.snapshot(ms);
     CHECK(s.valid&&s.soc_valid&&!s.held);CHECK(fabsf(s.voltage-24.1f)<.001f);
     CHECK(fabsf(s.percent-BatteryEstimator::percentAt(24.1f))<.01f);
     float saved=s.percent;
     feed(b,ms,19.8f,true,60000);s=b.snapshot(ms);
     CHECK(s.held&&s.percent==saved&&s.warning==0);CHECK(fabsf(s.voltage-19.8f)<.01f);
     feed(b,ms,24.1f,false,9500);CHECK(b.snapshot(ms).held&&b.snapshot(ms).percent==saved);
     feed(b,ms,24.1f,false,6000);CHECK(!b.snapshot(ms).held&&fabsf(b.snapshot(ms).percent-saved)<.01f);
     CHECK(!b.snapshot(ms+1501).valid);
     ms+=2000;feed(b,ms,22.2f,false,200);CHECK(!b.snapshot(ms).soc_valid||b.snapshot(ms).held);
     feed(b,ms,22.2f,false,10000);CHECK(b.snapshot(ms).held); // restart rest window after gap/filter warmup
     feed(b,ms,22.2f,false,6000);s=b.snapshot(ms);CHECK(!s.held);CHECK(s.percent<saved&&s.percent>saved-4);
    }
    {BatteryEstimator b;uint32_t ms=0;feed(b,ms,20.4f,true,30000);
     CHECK(b.snapshot(ms).valid&&!b.snapshot(ms).soc_valid&&b.snapshot(ms).warning==0);
     feed(b,ms,21.0f,false,11000);CHECK(b.snapshot(ms).soc_valid&&b.snapshot(ms).warning==0);
     feed(b,ms,21.0f,false,5000);CHECK(b.snapshot(ms).warning==2);
     feed(b,ms,21.1f,false,20000);CHECK(b.snapshot(ms).warning==2); // 13% hysteresis
     feed(b,ms,22.2f,false,30000);CHECK(b.snapshot(ms).warning==0);
    }
    {BatteryEstimator b;uint32_t ms=0;feed(b,ms,21.5f,false,17000);CHECK(b.snapshot(ms).warning==1);
     feed(b,ms,21.7f,false,20000);CHECK(b.snapshot(ms).warning==1); // <=23% holds low
     feed(b,ms,23.7f,false,30000);CHECK(b.snapshot(ms).warning==0);
     float p=b.snapshot(ms).percent;feed(b,ms,18,true,10000);CHECK(b.snapshot(ms).percent==p&&b.snapshot(ms).warning==0);
    }
    {BatteryEstimator b;uint32_t ms=0;feed(b,ms,24.1f,false,12000);
     float v=b.snapshot(ms).voltage;b.update(2400,false,ms+=50);CHECK(fabsf(b.snapshot(ms).voltage-v)<.01f); // median rejects one spike
     for(float invalid:{NAN,0.f,499.f,2601.f}){b.update(invalid,false,ms+=50);CHECK(!b.snapshot(ms).valid);}
     feed(b,ms,24.1f,false,1000);CHECK(b.snapshot(ms).valid&&b.snapshot(ms).held);
    }
    {BatteryEstimator b;uint32_t ms=UINT32_MAX-8000;feed(b,ms,24.1f,false,15000);
     CHECK(b.snapshot(ms).valid&&b.snapshot(ms).soc_valid&&!b.snapshot(ms).held);
    }
    {BatteryEstimator b;uint32_t ms=0;
     for(unsigned i=0;i<12;i++)feed(b,ms,i%2?24.f:22.f,false,2000);
     CHECK(!b.snapshot(ms).soc_valid); // unstable rest is not a qualified estimate
    }
    printf("BATTERY_ESTIMATOR_PASS checks=%u\n",checks);
}
