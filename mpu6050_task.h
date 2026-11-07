#pragma once
#include <Wire.h>
#include "imu_observer.h"

// Single I2C owner on a low-priority task, never under the motor output lock.
static portMUX_TYPE imu_mux=portMUX_INITIALIZER_UNLOCKED;
static ImuObserver imu_shared;
static bool imu_calibration_request=false;
static uint8_t imu_address=0x68;
static bool imu_vehicle_still() {
    portENTER_CRITICAL(&output_mux);
    bool still=!d2_running(ctrl)&&ctrl.state==D2_IDLE&&d2_stationary(ctrl,millis());
    portEXIT_CRITICAL(&output_mux);return still;
}
static ImuObserver imu_snapshot() {
    ImuObserver s;portENTER_CRITICAL(&imu_mux);s=imu_shared;portEXIT_CRITICAL(&imu_mux);
    if(!s.sequence||(uint32_t)(millis()-s.sample_ms)>50)s.valid=false;
    return s;
}
static bool imu_busy() {
    portENTER_CRITICAL(&imu_mux);
    bool busy=imu_calibration_request||imu_shared.calibrating;
    portEXIT_CRITICAL(&imu_mux);return busy;
}
static bool imu_request_calibration() {
    portENTER_CRITICAL(&imu_mux);
    bool ok=imu_shared.connected&&imu_shared.sequence&&imu_shared.error!=9&&!imu_shared.calibrating&&
        !imu_calibration_request&&(uint32_t)(millis()-imu_shared.sample_ms)<=50;
    if(ok){imu_calibration_request=true;imu_shared.calibrating=true;imu_shared.valid=false;}
    portEXIT_CRITICAL(&imu_mux);return ok;
}
static bool imu_read(uint8_t reg,uint8_t *out,uint8_t length) {
    Wire.beginTransmission(imu_address);Wire.write(reg);
    if(Wire.endTransmission(false)!=0)return false;
    if(Wire.requestFrom(imu_address,length,(uint8_t)true)!=length)return false;
    for(unsigned i=0;i<length;i++)out[i]=Wire.read();return true;
}
static bool imu_write(uint8_t reg,uint8_t value) {
    Wire.beginTransmission(imu_address);Wire.write(reg);Wire.write(value);
    return Wire.endTransmission(true)==0;
}
static bool imu_connect(ImuObserver &s) {
    uint8_t who=0;
    s.probe68=s.probe69=255;s.whoami=0;s.config_reg=0;s.error=1;
    for(uint8_t address:{0x68,0x69}) {
        imu_address=address;
        Wire.beginTransmission(address);
        uint8_t probe=Wire.endTransmission(true);
        if(address==0x68)s.probe68=probe;else s.probe69=probe;
        if(probe!=0)continue;
        if(!imu_read(0x75,&who,1))continue;
        s.whoami=who;s.error=2;
        if(imu_supported_chip(who)) {
            // Reset any DMP/FIFO/standby settings left by a previous sketch.
            if(!imu_write(0x6b,0x80)){s.config_reg=0x6b;s.error=10;return false;}
            vTaskDelay(pdMS_TO_TICKS(100));
            // PLL X gyro, 100 Hz data-ready, DLPF 42/44 Hz, gyro +/-500 dps,
            // accel +/-4g. Registers are read back; unknown chips are rejected.
            const uint8_t reg[]={0x6b,0x6c,0x6a,0x23,0x37,0x19,0x1a,0x1b,0x1c,0x38};
            const uint8_t val[]={0x01,0,0,0,0,9,3,8,8,1};
            for(unsigned i=0;i<sizeof(reg);i++)if(!imu_write(reg[i],val[i])){s.config_reg=reg[i];s.error=10;return false;}
            if(who==0x70&&!imu_write(0x1d,3)){s.config_reg=0x1d;s.error=10;return false;}
            vTaskDelay(pdMS_TO_TICKS(100));
            for(unsigned i=0;i<sizeof(reg);i++) {
                uint8_t actual=0;
                if(!imu_read(reg[i],&actual,1)||actual!=val[i]){s.config_reg=reg[i];s.error=10;return false;}
            }
            if(who==0x70) {
                uint8_t actual=0;
                if(!imu_read(0x1d,&actual,1)||actual!=3){s.config_reg=0x1d;s.error=10;return false;}
            }
            return true;
        }
    }
    return false;
}
static void imu_task(void*) {
    Wire.begin(21,22,100000);Wire.setTimeOut(3);
    ImuObserver s;imu_init(s);
    bool ready=false;uint32_t retry_ms=millis()-1000,warm_ms=0;
    TickType_t wake=xTaskGetTickCount();
    for(;;) {
        uint32_t ms=millis();
        if(!ready&&(uint32_t)(ms-retry_ms)>=1000) {
            retry_ms=ms;
            if(imu_connect(s)) {
                uint32_t epoch=s.calibration_id;
                uint8_t p68=s.probe68,p69=s.probe69,who=s.whoami;
                imu_init(s);s.calibration_id=epoch;s.probe68=p68;s.probe69=p69;s.whoami=who;
                ready=true;s.connected=true;warm_ms=millis();wake=xTaskGetTickCount();
            } else {s.connected=false;imu_fail(s,s.error);}
        }
        bool requested;
        portENTER_CRITICAL(&imu_mux);requested=imu_calibration_request;imu_calibration_request=false;portEXIT_CRITICAL(&imu_mux);
        bool still=imu_vehicle_still();
        if(requested) {
            if(!ready||(uint32_t)(ms-warm_ms)<1000||!imu_begin_calibration(s,ms,still))imu_fail(s,5);
        }
        if(ready) {
            uint8_t data_ready=0,bytes[14];
            bool ok=imu_read(0x3a,&data_ready,1);
            if(ok&&(data_ready&1)) {
                ok=imu_read(0x3b,bytes,14);
                if(ok) {
                    auto value=[&](unsigned i){return (int16_t)((uint16_t(bytes[i])<<8)|bytes[i+1]);};
                    float accel[3],gyro[3];
                    for(unsigned i=0;i<3;i++) {
                        accel[i]=value(i*2)*(9.80665f/8192);
                        gyro[i]=value(8+i*2)*(3.14159265359f/(180*65.5f));
                    }
                    imu_sample(s,accel,gyro,imu_temperature_c(value(6),s.whoami),micros(),millis(),still);
                }
            }
            if(!ok) {imu_fail(s,1);s.calibrated=false;s.connected=false;ready=false;retry_ms=ms;}
            else if(s.sequence&&(uint32_t)(millis()-s.sample_ms)>50){
                imu_fail(s,3);s.calibrated=false;s.connected=false;ready=false;retry_ms=ms;
            }
        }
        if(ready&&!s.calibrating&&!s.calibrated) {
            if((uint32_t)(millis()-warm_ms)<1000)s.error=9;
            else if(s.error==9)s.error=0;
        }
        portENTER_CRITICAL(&imu_mux);imu_shared=s;portEXIT_CRITICAL(&imu_mux);
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(10));
    }
}
static void imu_setup() {
    imu_init(imu_shared);
    if(xTaskCreatePinnedToCore(imu_task,"mpu_observer",4096,nullptr,1,nullptr,0)!=pdPASS)
        imu_fail(imu_shared,6);
}
