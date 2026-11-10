#include <Arduino.h>
#include <esp_timer.h>
#include "dual_v2_core.h"

static D2Core ctrl;
static portMUX_TYPE output_mux=portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE encoder_mux=portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t enc[2]={0,0};
static volatile uint8_t previous_ab[2]={0,0};
static volatile uint32_t illegal[2]={0,0};
static const int8_t quad[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
static const uint8_t A[2]={34,36},B[2]={35,39},SD[2]={23,27};
static bool timer_healthy=false,disabled=true;
static esp_timer_handle_t safety_timer;
static char boot[9],rx[256];
static size_t rx_len=0;
static bool rx_discard=false;
static uint32_t telem_ms=0,seq=0,tx_dropped=0;
// One bounded, non-interleaving TX queue. Control never waits for serial bandwidth.
static char tx[8][3072];
static uint16_t tx_len[8]={0},tx_offset=0;
static unsigned tx_head=0,tx_tail=0;
static char response[3072];
#include "mpu6050_task.h"
static void IRAM_ATTR read_encoder(unsigned i) {
    portENTER_CRITICAL_ISR(&encoder_mux);
    uint8_t ab=(digitalRead(A[i])<<1)|digitalRead(B[i]);
    int8_t step=quad[((previous_ab[i]<<2)|ab)&15];
    if(step)enc[i]+=step;else if(ab!=previous_ab[i])illegal[i]++;
    previous_ab[i]=ab;
    portEXIT_CRITICAL_ISR(&encoder_mux);
}
void IRAM_ATTR encoder1(){read_encoder(0);}
void IRAM_ATTR encoder2(){read_encoder(1);}
static void apply_locked() {
    bool on=!disabled&&timer_healthy&&ctrl.outputs&&d2_running(ctrl);
    if(!on){digitalWrite(SD[0],LOW);digitalWrite(SD[1],LOW);}
    for(int i=0;i<2;i++) {
        uint16_t a=0,b=0;
        if(on)d2_legs(ctrl.pwm[i],d2_wheel_direction(ctrl,i)*(i==0?-1:1),a,b);
        ledcWrite(i*2,a);ledcWrite(i*2+1,b);
        if(on)digitalWrite(SD[i],HIGH);
    }
}
static void watchdog(void*) {
    uint32_t ms=millis();
    portENTER_CRITICAL(&output_mux);
    if(d2_running(ctrl) && (uint32_t)(ms-ctrl.heartbeat_ms)>480) {
        disabled=true;d2_fault(ctrl,D2_WATCHDOG);apply_locked();
    }
    portEXIT_CRITICAL(&output_mux);
}
static bool enqueue(const char *line,bool telemetry=false) {
    unsigned next=(tx_tail+1)%8;
    // Reserve two slots for command ACKs; telemetry is expendable, never mixed.
    unsigned used=(tx_tail+8-tx_head)%8;
    if(next==tx_head || (telemetry&&used>=4)){tx_dropped++;return false;}
    size_t n=strlen(line);
    if(n+1>=sizeof(tx[0])){tx_dropped++;return false;}
    memcpy(tx[tx_tail],line,n);tx[tx_tail][n++]='\n';tx_len[tx_tail]=n;
    tx_tail=next;return true;
}
static void pump_tx() {
    if(tx_head==tx_tail)return;
    int available=Serial.availableForWrite();if(available<=0)return;
    size_t n=min((size_t)available,(size_t)(tx_len[tx_head]-tx_offset));
    if(n>64)n=64;
    size_t wrote=Serial.write((uint8_t*)tx[tx_head]+tx_offset,n);
    tx_offset+=wrote;
    if(tx_offset==tx_len[tx_head]){tx_offset=0;tx_head=(tx_head+1)%8;}
}
static const char* state_name(D2State s) {
    switch(s){case D2_IDLE:return "IDLE";case D2_MOVE:return "MOVE";case D2_SPEED:return "SPEED";
        case D2_STOPPING:return "STOPPING";default:return "FAULT";}
}
static uint32_t telemetry_period_ms(const D2Core &c) {
    // The optional turn evidence is larger. Leave UART room for heartbeat ACKs.
    // Straight motion and legacy turns retain the C2 100 ms telemetry cadence.
    return (c.turning&&c.turn_feedback.used_in_run)||c.creep.used[0]||c.creep.used[1]||c.terminal_settle.used?125u:100u;
}
static bool enqueue_terminal_status(const D2Core &c,bool telemetry) {
    if(c.terminal_settle.used) {
        const auto &t=c.terminal_settle;size_t len=strlen(response);
        if(!len||response[len-1]!='}')return false;
        int n=snprintf(response+len-1,sizeof(response)-len+1,
            ",\"terminal\":{\"used\":true,\"pending\":%s,\"qualified\":%s,\"error_m\":[%.6f,%.6f],"
            "\"limit_m\":%.6f,\"yaw_error_deg\":%.4f,\"quiet_ms\":%lu,\"elapsed_ms\":%lu,\"rejection\":%u}}",
            t.pending?"true":"false",t.qualified?"true":"false",t.error_m[0],t.error_m[1],
            t.limit_m,t.yaw_error_deg,(unsigned long)t.quiet_ms,(unsigned long)t.elapsed_ms,(unsigned)t.rejection);
        if(n<=0||n>=(int)sizeof(response)-(int)len+1)return false;
    }
    return enqueue(response,telemetry);
}
static void status(uint32_t request=0,bool telemetry=false) {
    D2Core c;bool healthy;int sd1,sd2;
    portENTER_CRITICAL(&output_mux);c=ctrl;healthy=timer_healthy;
    sd1=digitalRead(SD[0]);sd2=digitalRead(SD[1]);portEXIT_CRITICAL(&output_mux);
    int n=snprintf(response,sizeof(response),
        "{\"type\":\"status\",\"request\":%lu,\"build\":\"%s\",\"boot\":\"%s\",\"seq\":%lu,\"ts\":%lu,"
        "\"state\":\"%s\",\"fault\":%d,\"run_id\":%lu,\"dir\":%d,\"sd\":[%d,%d],"
        "\"rpm\":[%.3f,%.3f],\"raw_rpm\":[%.3f,%.3f],\"base\":[%.3f,%.3f],\"sp\":[%.3f,%.3f],"
        "\"pwm\":[%.1f,%.1f],\"counts\":[%lld,%lld],\"distance\":[%.5f,%.5f],\"trim\":%.3f,\"gov_scale\":%.4f,\"gov_used\":%s,\"gov_min\":%.4f,\"gov_reason\":%u,\"gov_active\":%s,\"gov_lead_cap\":%.2f,"
        "\"turning\":%s,\"accuracy_scale\":%.4f,\"pos_ref\":%.5f,\"pos_trim\":%.3f,\"pos_cutoff\":%s,\"dist_phase\":%u,\"dist_stop_reason\":%u,"
        "\"pid_p\":[%.3f,%.3f],\"pid_i\":[%.3f,%.3f],\"pid_ff\":[%.3f,%.3f],\"pid_sat\":[%s,%s],\"pid_slew\":[%s,%s],"
        "\"timer_ok\":%s,\"configured\":%s,\"stationary\":%s,\"max_dt_ms\":%u,\"tx_dropped\":%lu}",
        (unsigned long)request,D2_BUILD,boot,(unsigned long)++seq,(unsigned long)millis(),state_name(c.state),(int)c.fault,
        (unsigned long)c.run_id,c.direction,sd1,sd2,c.rpm[0],c.rpm[1],c.raw_rpm[0],c.raw_rpm[1],
        c.base[0],c.base[1],c.effective[0],c.effective[1],c.pwm[0],c.pwm[1],(long long)c.counts[0],(long long)c.counts[1],
        c.distance[0],c.distance[1],c.trim,c.governor_scale,c.governor_used?"true":"false",c.governor_min_scale,(unsigned)c.governor_reason,c.governor_last_pressure?"true":"false",c.lead_cap_rpm,c.turning?"true":"false",c.accuracy_scale,c.position_ref_m,c.position_trim_rpm,c.position_cutoff_latched?"true":"false",(unsigned)c.distance_phase,(unsigned)c.distance_stop_reason,
        c.result[0].proportional,c.result[1].proportional,c.result[0].integral,c.result[1].integral,c.result[0].feedforward,c.result[1].feedforward,
        c.result[0].amplitude_saturated?"true":"false",c.result[1].amplitude_saturated?"true":"false",c.result[0].slew_saturated?"true":"false",c.result[1].slew_saturated?"true":"false",healthy?"true":"false",c.configured?"true":"false",
        d2_stationary(c,millis())?"true":"false",c.max_dt_ms,(unsigned long)tx_dropped);
    if(n>0 && n<(int)sizeof(response)) {
        ImuObserver im=imu_snapshot();
        int extra=snprintf(response+n-1,sizeof(response)-n+1,
            ",\"enc_ts\":%lu,\"yaw_sign\":%d,\"run_start_ms\":%lu,\"run_duration_s\":%.4f,\"time_priority\":%s,\"imu\":{\"sample_ms\":%lu,\"seq\":%lu,\"epoch\":%lu,"
            "\"connected\":%s,\"calibrated\":%s,\"calibrating\":%s,\"valid\":%s,\"error\":%u,\"cal_samples\":%u,"
            "\"accel\":[%.4f,%.4f,%.4f],\"gyro\":[%.5f,%.5f,%.5f],\"bias\":[%.6f,%.6f,%.6f],"
            "\"q\":[%.6f,%.6f,%.6f,%.6f],\"rpy\":[%.5f,%.5f,%.5f],\"temp_c\":%.2f,\"probe68\":%u,\"probe69\":%u,\"whoami\":%u,\"config_reg\":%u}}",
            (unsigned long)c.last_step_ms,D2_YAW_SIGN,(unsigned long)c.start_ms,c.duration_s,c.time_priority?"true":"false",(unsigned long)im.sample_ms,(unsigned long)im.sequence,(unsigned long)im.calibration_id,
            im.connected?"true":"false",im.calibrated?"true":"false",im.calibrating?"true":"false",im.valid?"true":"false",
            im.error,im.calibration_samples,im.accel[0],im.accel[1],im.accel[2],im.gyro[0],im.gyro[1],im.gyro[2],
            im.bias[0],im.bias[1],im.bias[2],im.q[0],im.q[1],im.q[2],im.q[3],im.rpy[0],im.rpy[1],im.rpy[2],im.temperature,im.probe68,im.probe69,im.whoami,im.config_reg);
        if(extra>0&&extra<(int)sizeof(response)-n+1) {
            int used=n-1+extra;
            int gs=snprintf(response+used-1,sizeof(response)-used+1,
                ",\"gs\":{\"enabled\":%s,\"alpha\":%.5f,\"target_alpha\":%.5f,"
                "\"requested_mps\":%.6f,\"update_sequence\":%lu,\"integral_clamped\":%s}}",
                c.gains.enabled?"true":"false",c.gains.alpha,c.gains.target_alpha,
                c.gains.requested_mps,(unsigned long)c.gains.update_sequence,
                c.gains.integral_clamped?"true":"false");
            if(gs>0&&gs<(int)sizeof(response)-used+1) {
                int tail=used-1+gs;
                int hc=snprintf(response+tail-1,sizeof(response)-tail+1,
                    ",\"high_ctrl\":{\"mode\":%u,\"active\":%s,\"imu_ok\":%s,\"headroom_limited\":%s,"
                    "\"a_max\":%.3f,\"yaw_ref\":%.5f,\"yaw_err\":%.5f,\"omega\":%.5f,"
                    "\"offset\":[%.5f,%.5f],\"achieved_speed\":%.3f,"
                    "\"used_in_run\":%s,\"reference_valid\":%s,\"accel_limited\":%s,"
                    "\"high_weight\":%.4f,\"common_mps\":%.4f,\"gyro_z\":%.5f,"
                    "\"imu_epoch\":%lu,\"imu_sample_ms\":%lu,\"startup_guard\":%s,\"startup_accel_scale\":%.4f,\"observed_sp\":[%.3f,%.3f]}}",
                    (unsigned)c.high_ctrl.mode,c.high_ctrl.active?"true":"false",c.high_ctrl.imu_ok?"true":"false",
                    c.high_ctrl.headroom_limited?"true":"false",c.high_ctrl.a_max,c.high_ctrl.yaw_ref,
                    c.high_ctrl.yaw_err,c.high_ctrl.omega_cmd,c.high_ctrl.wheel_offset[0],c.high_ctrl.wheel_offset[1],
                    c.peak_achieved_mps,c.high_ctrl.used_in_run?"true":"false",c.high_ctrl.reference_valid?"true":"false",
                    c.high_ctrl.accel_limited?"true":"false",c.high_ctrl.high_weight,c.common_v,c.high_ctrl.gyro_z,
                    (unsigned long)c.high_ctrl.imu_epoch,(unsigned long)c.high_ctrl.last_imu_sample_ms,
                    c.startup_heading_guard?"true":"false",c.startup_accel_scale,
                    c.observed_reference_rpm[0],c.observed_reference_rpm[1]);
                if(hc>0&&hc<(int)sizeof(response)-tail+1) {
                    if(c.turn_feedback.used_in_run&&c.turning) {
                        int end=tail-1+hc;
                        int tf=snprintf(response+end-1,sizeof(response)-end+1,
                            ",\"turn_feedback\":{\"enabled\":%s,\"used_in_run\":true,\"active\":%s,\"valid\":%s,"
                            "\"yaw_ref\":%.7f,\"target_rad\":%.7f,\"angle_rad\":%.7f,\"yaw_rate\":%.6f,"
                            "\"offset_m\":%.7f,\"target_offset_m\":%.7f,\"stop_reason\":%u,\"epoch\":%lu,\"sample_ms\":%lu,"
                            "\"finish_phase\":%u,\"correction_used\":%s,\"pulse_ms\":%lu,\"correction_elapsed_ms\":%u,\"cutoff_angle_rad\":%.7f,\"correction_start_rad\":%.7f}}",
                            c.turn_feedback.enabled?"true":"false",c.turn_feedback.active?"true":"false",c.turn_feedback.valid?"true":"false",
                            c.turn_feedback.yaw_ref,c.turn_feedback.target_rad,c.turn_feedback.angle_rad,c.turn_feedback.yaw_rate,
                            c.turn_feedback.offset_m,c.turn_feedback.target_offset_m,(unsigned)c.turn_feedback.stop_reason,
                            (unsigned long)c.turn_feedback.epoch,(unsigned long)c.turn_feedback.sample_ms,
                            (unsigned)c.turn_feedback.finish_phase,c.turn_feedback.correction_used?"true":"false",
                            (unsigned long)c.turn_feedback.pulse_ms,(unsigned)c.turn_feedback.correction_elapsed_ms,c.turn_feedback.cutoff_angle_rad,c.turn_feedback.correction_start_rad);
                        if(tf>0&&tf<(int)sizeof(response)-end+1)enqueue_terminal_status(c,telemetry);
                    } else if(c.creep.used[0]||c.creep.used[1]) {
                        int end=tail-1+hc;
                        int ca=snprintf(response+end-1,sizeof(response)-end+1,
                            ",\"creep\":{\"used\":[%s,%s],\"boost\":[%.1f,%.1f],\"peak\":[%.1f,%.1f],\"spent\":[%s,%s],"
                            "\"elapsed_ms\":[%u,%u],\"net_m\":[%.6f,%.6f],\"failed\":[%s,%s]}}",
                            c.creep.used[0]?"true":"false",c.creep.used[1]?"true":"false",
                            c.creep.boost[0],c.creep.boost[1],c.creep.peak[0],c.creep.peak[1],
                            c.creep.spent[0]?"true":"false",c.creep.spent[1]?"true":"false",
                            (unsigned)c.creep.assisted_ms[0],(unsigned)c.creep.assisted_ms[1],
                            c.creep.used[0]?d2_wheel_direction(c,0)*(c.counts[0]-c.creep.start_count[0])*d2_terminal_quantum(c,0):0,
                            c.creep.used[1]?d2_wheel_direction(c,1)*(c.counts[1]-c.creep.start_count[1])*d2_terminal_quantum(c,1):0,
                            c.creep.failed[0]?"true":"false",c.creep.failed[1]?"true":"false");
                        if(ca>0&&ca<(int)sizeof(response)-end+1)enqueue_terminal_status(c,telemetry);
                    } else enqueue_terminal_status(c,telemetry);
                }
            }
        }
    }
}
static void turn_configuration(uint32_t request) {
    bool enabled;portENTER_CRITICAL(&output_mux);enabled=ctrl.turn_feedback.enabled;portEXIT_CRITICAL(&output_mux);
    snprintf(response,sizeof(response),"{\"type\":\"turn_config\",\"request\":%lu,\"boot\":\"%s\",\"enabled\":%s}",
        (unsigned long)request,boot,enabled?"true":"false");
    if(!enqueue(response))emergency();
}
static void configuration(uint32_t request) {
    D2Core c;portENTER_CRITICAL(&output_mux);c=ctrl;portEXIT_CRITICAL(&output_mux);
    snprintf(response,sizeof(response),
        "{\"type\":\"config\",\"request\":%lu,\"boot\":\"%s\",\"geometry\":[%.4f,%.4f,%.4f],"
        "\"limits\":[%.3f,%.1f,%.1f,%.3f,%.3f,%.4f],\"m1\":[%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f],"
        "\"m2\":[%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f]}",
        (unsigned long)request,boot,c.diameter[0],c.diameter[1],c.track,c.max_rpm,c.max_pwm,c.slew,c.sync_kp,c.sync_max,c.divergence,
        c.tune[0].kp,c.tune[0].ki,c.tune[0].kaw,c.tune[0].slope,c.tune[0].offset,c.tune[0].ramp,c.tune[0].jerk,
        c.tune[1].kp,c.tune[1].ki,c.tune[1].kaw,c.tune[1].slope,c.tune[1].offset,c.tune[1].ramp,c.tune[1].jerk);
    enqueue(response);
}
static void emergency() {
    portENTER_CRITICAL(&output_mux);disabled=true;d2_stop(ctrl);apply_locked();portEXIT_CRITICAL(&output_mux);
}
static void gain_configuration(uint32_t request) {
    D2Core c;portENTER_CRITICAL(&output_mux);c=ctrl;portEXIT_CRITICAL(&output_mux);
    const D2Tune &a=c.gains.high[0],&b=c.gains.high[1];
    int n=snprintf(response,sizeof(response),
        "{\"type\":\"gain_config\",\"request\":%lu,\"boot\":\"%s\",\"enabled\":%s,\"transition_ms\":%.3f,"
        "\"m1\":[%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f],"
        "\"m2\":[%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f]}",
        (unsigned long)request,boot,c.gains.enabled?"true":"false",c.gains.transition_s*1000,
        a.kp,a.ki,a.kaw,a.slope,a.offset,a.ramp,a.jerk,
        b.kp,b.ki,b.kaw,b.slope,b.offset,b.ramp,b.jerk);
    if(n>0&&n<(int)sizeof(response))enqueue(response);
}
static void high_ctrl_configuration(uint32_t request) {
    D2Core c;portENTER_CRITICAL(&output_mux);c=ctrl;portEXIT_CRITICAL(&output_mux);
    int n=snprintf(response,sizeof(response),
        "{\"type\":\"high_ctrl_config\",\"request\":%lu,\"boot\":\"%s\",\"mode\":%u,\"a_max\":%.3f,"
        "\"k_yaw\":%.3f,\"k_rate\":%.3f,\"max_omega\":%.3f,\"slew_omega\":%.3f}",
        (unsigned long)request,boot,(unsigned)c.high_ctrl.mode,c.high_ctrl.a_max,
        c.high_ctrl.k_yaw,c.high_ctrl.k_rate,c.high_ctrl.max_omega,c.high_ctrl.slew_omega);
    if(n>0&&n<(int)sizeof(response))enqueue(response);
}
static void handle(char *line) {
    char *end=nullptr;unsigned long id=strtoul(line,&end,10);if(end==line||*end!=' ')return;
    char *verb=end+1;char *args=strchr(verb,' ');if(args)*args++=0;
    double values[10];int count=0;bool numeric=true;
    while(args&&*args) {
        if(count==10){numeric=false;break;}
        char *tail=nullptr;double v=strtod(args,&tail);
        if(tail==args||!isfinite(v)||(*tail&&*tail!=' ')){numeric=false;break;}
        values[count++]=v;args=tail;while(*args==' ')args++;
    }
    ImuObserver im=imu_snapshot();
    bool ok=false;uint32_t ms=millis();
    if(!strcmp(verb,"STATUS")&&count==0){status(id);return;}
    if(!strcmp(verb,"CONFIG_GET")&&count==0){configuration(id);return;}
    if(!strcmp(verb,"GS_GET")&&count==0&&numeric){gain_configuration(id);return;}
    if(!strcmp(verb,"HIGH_CTRL_GET")&&count==0&&numeric){high_ctrl_configuration(id);return;}
    if(!strcmp(verb,"TURN_CTRL_GET")&&count==0&&numeric){turn_configuration(id);return;}
    portENTER_CRITICAL(&output_mux);
    if(numeric) {
        if(!strcmp(verb,"STOP")&&count==0){disabled=true;d2_stop(ctrl);ok=true;}
        else if(!strcmp(verb,"HEARTBEAT")&&count==0){ctrl.heartbeat_ms=ms;ok=true;}
        else if(!strcmp(verb,"CLEAR")&&count==0){ok=d2_clear(ctrl,ms,timer_healthy);}
        else if(!strcmp(verb,"IMU_CALIBRATE")&&count==0&&!d2_running(ctrl)&&ctrl.state==D2_IDLE&&d2_stationary(ctrl,ms)){ok=imu_request_calibration();}
        else if(!strcmp(verb,"SOFT_STOP")&&count==0){d2_soft_stop(ctrl,ms);ok=true;}
        else if(!strcmp(verb,"CONFIG")&&count==6) {
            ok=d2_limits(ctrl,values[0],values[1],values[2],values[3],values[4],values[5]);
            if(ok){ctrl.configured=true;d2_gs_invalidate(ctrl);}
        } else if(!strcmp(verb,"TUNE")&&count==8 && (values[0]==1||values[0]==2)&&ctrl.state==D2_IDLE) {
            D2Tune t={(float)values[1],(float)values[2],(float)values[3],(float)values[4],(float)values[5],(float)values[6],(float)values[7]};
            ok=d2_valid_tune(t);if(ok){ctrl.tune[(int)values[0]-1]=t;d2_gs_invalidate(ctrl);}
        } else if(!strcmp(verb,"GS_TUNE")&&count==8&&(values[0]==1||values[0]==2)&&!imu_busy()) {
            D2Tune t={(float)values[1],(float)values[2],(float)values[3],(float)values[4],(float)values[5],(float)values[6],(float)values[7]};
            ok=d2_gs_stage(ctrl,(int)values[0]-1,t,ms);
        } else if(!strcmp(verb,"HIGH_CTRL_SET")&&count==6&&ctrl.state==D2_IDLE&&d2_stationary(ctrl,ms)&&!imu_busy()) {
            if(values[0]==robot_config::HIGH_MODE) {
                uint8_t mode=(uint8_t)values[0];
                ok=d2_valid_high_ctrl(mode,values[1],values[2],values[3],values[4],values[5]);
                if(ok){ctrl.high_ctrl.mode=mode;ctrl.high_ctrl.a_max=values[1];ctrl.high_ctrl.k_yaw=values[2];
                    ctrl.high_ctrl.k_rate=values[3];ctrl.high_ctrl.max_omega=values[4];ctrl.high_ctrl.slew_omega=values[5];}
            }
        } else if(!strcmp(verb,"GS_ENABLE")&&count==2&&(values[0]==0||values[0]==1)&&!imu_busy()) {
            ok=d2_gs_enable(ctrl,values[0]==1,(float)values[1],ms);
        } else if(!strcmp(verb,"TURN_CTRL_SET")&&count==1&&values[0]==1&&
                  ctrl.state==D2_IDLE&&d2_stationary(ctrl,ms)&&!imu_busy()) {
            ok=d2_high_imu_ready(&im,ms);
            if(ok)ctrl.turn_feedback.enabled=true;
        } else if(!strcmp(verb,"SPEED_UPDATE")&&count==3&&timer_healthy&&!imu_busy()&&
                  values[1]>=1&&values[1]<=4294967295.0&&floor(values[1])==values[1]&&
                  values[2]>=1&&values[2]<=4294967295.0&&floor(values[2])==values[2]) {
            ok=d2_update_speed(ctrl,(float)values[0],(uint32_t)values[1],(uint32_t)values[2],ms,&im);
        } else if(!strcmp(verb,"TURN_TIME")&&count==3&&timer_healthy && !imu_busy() &&
                  values[2]>=1&&values[2]<=4294967295.0&&floor(values[2])==values[2]) {
            ok=d2_start_turn_time(ctrl,values[0],values[1],ms,(uint32_t)values[2],&im);
            if(ok)disabled=false;
        } else if(!strcmp(verb,"MOVE_DIST")&&count==4&&timer_healthy && !imu_busy() &&
                  values[3]>=1&&values[3]<=4294967295.0&&floor(values[3])==values[3]) {
            ok=d2_start_distance(ctrl,values[0],values[1],values[2],ms,(uint32_t)values[3],&im);
            if(ok)disabled=false;

        }
    }
    apply_locked();portEXIT_CRITICAL(&output_mux);
    snprintf(response,sizeof(response),"{\"type\":\"ack\",\"request\":%lu,\"boot\":\"%s\",\"ok\":%s,\"command\":\"%s\"}",id,boot,ok?"true":"false",verb);
    if(!enqueue(response)){emergency();}
}
void setup() {
    for(int i=0;i<2;i++){pinMode(SD[i],OUTPUT);digitalWrite(SD[i],LOW);}
    const uint8_t pins[4]={18,19,25,26};
    for(int i=0;i<4;i++){ledcSetup(i,20000,10);ledcAttachPin(pins[i],i);ledcWrite(i,0);}
    d2_init(ctrl,millis());snprintf(boot,sizeof(boot),"%08lX",(unsigned long)esp_random());
    for(int i=0;i<2;i++) {
        pinMode(A[i],INPUT);pinMode(B[i],INPUT);previous_ab[i]=(digitalRead(A[i])<<1)|digitalRead(B[i]);
        attachInterrupt(A[i],i==0?encoder1:encoder2,CHANGE);attachInterrupt(B[i],i==0?encoder1:encoder2,CHANGE);
    }
    Serial.begin(230400);
    esp_timer_create_args_t timer_args={};timer_args.callback=watchdog;timer_args.name="dual_safety";
    timer_args.dispatch_method=ESP_TIMER_TASK;
    bool healthy=esp_timer_create(&timer_args,&safety_timer)==ESP_OK;
    if(healthy)healthy=esp_timer_start_periodic(safety_timer,10000)==ESP_OK;
    portENTER_CRITICAL(&output_mux);timer_healthy=healthy;
    if(!healthy)d2_fault(ctrl,D2_TIMER);apply_locked();portEXIT_CRITICAL(&output_mux);
    imu_setup();
}
void loop() {
    ImuObserver im=imu_snapshot();
    uint32_t ms=millis();int64_t a,b;
    portENTER_CRITICAL(&encoder_mux);a=enc[0];b=enc[1];portEXIT_CRITICAL(&encoder_mux);
    portENTER_CRITICAL(&output_mux);
    if((uint32_t)(ms-ctrl.last_step_ms)>=20) {
        d2_step(ctrl,a,b,ms,&im);if(ctrl.state==D2_FAULT)disabled=true;apply_locked();
    }
    portEXIT_CRITICAL(&output_mux);
    // Immediate ! stop, bounded RX; overflow discards the complete line.
    for(int i=0;i<32&&Serial.available();i++) {
        char ch=Serial.read();
        if(ch=='!'){emergency();rx_len=0;rx_discard=false;continue;}
        if(ch=='\n') {
            if(!rx_discard&&rx_len){rx[rx_len]=0;handle(rx);}
            rx_len=0;rx_discard=false;
        } else if(ch!='\r'&&!rx_discard) {
            if(rx_len+1<sizeof(rx))rx[rx_len++]=ch;
            else{rx_len=0;rx_discard=true;emergency();}
        }
    }
    if((uint32_t)(ms-telem_ms)>=telemetry_period_ms(ctrl)){telem_ms=ms;status(0,true);}
    pump_tx();delay(1);
}
