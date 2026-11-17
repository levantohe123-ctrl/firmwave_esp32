#pragma once
#include <WiFi.h>
#include <esp_http_server.h>
#include <lwip/sockets.h>
#include <lwip/tcp.h>
#include <freertos/queue.h>
#include "wifi_config.h"
#include "wifi_control.h"
#include "dashboard_assets.h"
#include "battery_task.h"

static RemoteSession remote_session;
static QueueHandle_t remote_in=nullptr,remote_drive_in=nullptr,remote_stop_in=nullptr;
static httpd_handle_t remote_server=nullptr;
static const char *remote_startup="not_started";
static portMUX_TYPE remote_view_mux=portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE remote_link_mux=portMUX_INITIALIZER_UNLOCKED;
static char remote_view[1792]="{\"ready\":false}";
static uint32_t remote_view_ms=0,remote_ack_seq=0;
static const char *remote_ack_code="ok",*remote_cal_status="idle";
static bool remote_cal_pending=false,remote_cal_started=false,remote_tx_pending=false;
static uint32_t remote_cal_ms=0;
static int remote_fd=-1;
static uint64_t remote_link=0;
static void remote_schedule_publish();

static void remote_tick(const ImuObserver &im,uint32_t ms) {
    if(!remote_in||!remote_drive_in||!remote_stop_in)return;
    RemoteCommand q={};
    portENTER_CRITICAL(&remote_link_mux);uint64_t current_link=remote_link;portEXIT_CRITICAL(&remote_link_mux);
    portENTER_CRITICAL(&output_mux);
    if(remote_session.owner!=current_link) {
        remote_session.stop(ctrl);remote_session.owner=0;remote_session.sequence=0;
        remote_ack_seq=0;remote_ack_code="ok";remote_cal_pending=false;remote_cal_status="idle";
        if(current_link) {
            RemoteCommand hello={};hello.op=RemoteOp::Claim;hello.owner=current_link;hello.received_ms=ms;
            remote_session.apply(ctrl,hello,ms,im,timer_healthy,false);
        }
        disabled=true;apply_locked();
    }
    if(remote_session.expire(ctrl,ms)){disabled=true;apply_locked();}
    portEXIT_CRITICAL(&output_mux);
    auto execute=[&](const RemoteCommand &cmd) {
        bool busy=imu_busy()||remote_cal_pending;
        portENTER_CRITICAL(&output_mux);
        if(cmd.owner!=remote_session.owner&&cmd.op!=RemoteOp::Stop){portEXIT_CRITICAL(&output_mux);return;}
        const char *result=remote_session.apply(ctrl,cmd,ms,im,timer_healthy,busy);
        if(!strcmp(result,"calibrate")) {
            remote_cal_pending=true;remote_cal_started=false;remote_cal_ms=ms;
            remote_cal_status="warming";result="ok";
        }
        if(cmd.op==RemoteOp::Stop||cmd.op==RemoteOp::Release) {
            remote_cal_pending=false;remote_cal_status="idle";
        }
        if(cmd.owner==remote_session.owner&&cmd.op!=RemoteOp::Drive&&cmd.op!=RemoteOp::Pulse) {
            remote_ack_seq=cmd.sequence;remote_ack_code=result;
        }
        disabled=!d2_running(ctrl)||ctrl.fault!=D2_NONE;apply_locked();
        portEXIT_CRITICAL(&output_mux);
    };
    if(xQueueReceive(remote_stop_in,&q,0)==pdTRUE)execute(q);
    for(int i=0;i<4&&xQueueReceive(remote_in,&q,0)==pdTRUE;i++)execute(q);
    if(xQueueReceive(remote_drive_in,&q,0)==pdTRUE)execute(q);
    if(remote_cal_pending) {
        portENTER_CRITICAL(&output_mux);
        bool still=ctrl.state==D2_IDLE&&d2_stationary(ctrl,ms)&&!ctrl.outputs;
        portEXIT_CRITICAL(&output_mux);
        if(!still||uint32_t(ms-remote_cal_ms)>12000) {
            remote_cal_pending=false;remote_cal_status="failed";
        } else if(!remote_cal_started) {
            if(im.connected&&im.sequence&&im.error!=9&&uint32_t(ms-im.sample_ms)<=50&&imu_request_calibration()) {
                remote_cal_started=true;remote_cal_status="sampling";
            }
        } else if(im.calibrated&&im.valid&&!im.calibrating) {
            remote_cal_pending=false;remote_cal_status="complete";
        } else if(!imu_busy()&&im.error&&im.error!=9) {
            remote_cal_pending=false;remote_cal_status="failed";
        }
    }
    bool busy=imu_busy()||remote_cal_pending;
    if(uint32_t(ms-remote_view_ms)<100)return;
    remote_view_ms=ms;
    D2Core c;bool healthy;
    portENTER_CRITICAL(&output_mux);c=ctrl;healthy=timer_healthy;portEXIT_CRITICAL(&output_mux);
    bool imu_ok=d2_high_imu_ready(&im,ms);
    bool stationary=d2_stationary(c,ms);
    char json[sizeof(remote_view)]={};
    BatteryView bat=battery_snapshot(ms);
    char volts[16]="null",percent[16]="null";
    if(bat.valid)snprintf(volts,sizeof(volts),"%.3f",bat.voltage);
    if(bat.valid&&bat.soc_valid)snprintf(percent,sizeof(percent),"%.1f",bat.percent);
    portENTER_CRITICAL(&battery_mux);auto adc_source=battery_cal_source;
    int adc_raw=battery_adc_raw;uint32_t adc_mv=battery_adc_mv;const char *adc_status=battery_adc_status;
    portEXIT_CRITICAL(&battery_mux);
    int json_size=snprintf(json,sizeof(json),
        "{\"boot\":\"%s\",\"build\":\"%s\",\"ms\":%lu,\"revision\":%lu,"
        "\"state\":\"%s\",\"fault\":%u,\"ready\":%s,\"remote_ready\":%s,\"stationary\":%s,"
        "\"owned\":%s,\"manual\":%s,\"run\":%lu,\"turning\":%s,"
        "\"speed\":%.4f,\"rpm\":[%.2f,%.2f],\"sp\":[%.2f,%.2f],"
        "\"distance\":%.4f,\"target\":%.4f,\"yaw\":%.2f,"
        "\"imu_ok\":%s,\"imu_epoch\":%lu,\"imu_error\":%u,\"calibrating\":%s,\"cal_samples\":%u,"
        "\"ack\":%lu,\"ack_code\":\"%s\",\"cal_status\":\"%s\",\"link_lost\":%s,"
        "\"high_active\":%s,\"turn_imu\":%s,\"max_dt_ms\":%u,"
        "\"battery\":{\"valid\":%s,\"voltage\":%s,\"percent\":%s,\"held\":%s,"
        "\"warning\":%u,\"sample_ms\":%lu,\"soc_age_s\":%lu,\"adc_efuse\":%s,"
        "\"adc_raw\":%d,\"adc_mv\":%lu,\"status\":\"%s\"}}",
        boot,ROBOT_WIFI_BUILD,(unsigned long)ms,(unsigned long)remote_session.revision,
        remote_session.drive.active?"REMOTE":state_name(c.state),(unsigned)c.fault,
        c.state==D2_IDLE&&stationary&&healthy&&imu_ok&&!busy&&c.fault==D2_NONE?"true":"false",
        healthy&&!busy&&c.fault==D2_NONE?"true":"false",
        stationary?"true":"false",remote_session.owner?"true":"false",remote_session.manual?"true":"false",
        (unsigned long)c.run_id,c.turning?"true":"false",
        (c.rpm[0]*c.diameter[0]+c.rpm[1]*c.diameter[1])*D2_PI/120,
        c.rpm[0],c.rpm[1],c.effective[0],c.effective[1],
        (fabsf(c.distance[0])+fabsf(c.distance[1]))*.5f,c.target_m,im.rpy[2]*180/D2_PI,
        imu_ok?"true":"false",(unsigned long)im.calibration_id,im.error,busy?"true":"false",im.calibration_samples,
        (unsigned long)remote_ack_seq,remote_ack_code,remote_cal_status,remote_session.drive.neutral_required?"true":"false",
        c.high_ctrl.active?"true":"false",c.turn_feedback.enabled?"true":"false",c.max_dt_ms,
        bat.valid?"true":"false",volts,percent,bat.held?"true":"false",bat.warning,
        (unsigned long)bat.sample_ms,(unsigned long)(bat.soc_valid?uint32_t(ms-bat.soc_ms)/1000:0),
        adc_source!=ESP_ADC_CAL_VAL_DEFAULT_VREF?"true":"false",adc_raw,(unsigned long)adc_mv,adc_status);
    if(json_size<0||json_size>=(int)sizeof(json))return;
    portENTER_CRITICAL(&remote_view_mux);memcpy(remote_view,json,sizeof(json));portEXIT_CRITICAL(&remote_view_mux);
    remote_schedule_publish();
}

static esp_err_t remote_json(httpd_req_t *req,const char *body,const char *status="200 OK") {
    httpd_resp_set_status(req,status);httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");
    httpd_resp_set_hdr(req,"X-Content-Type-Options","nosniff");
    return httpd_resp_sendstr(req,body);
}
static esp_err_t remote_state(httpd_req_t *req) {
    char json[sizeof(remote_view)];portENTER_CRITICAL(&remote_view_mux);
    memcpy(json,remote_view,sizeof(json));portEXIT_CRITICAL(&remote_view_mux);
    return remote_json(req,json);
}
static bool remote_field(const char *body,const char *name,char *out,size_t cap) {
    return httpd_query_key_value(body,name,out,cap)==ESP_OK;
}
static bool remote_uint(const char *body,const char *name,uint32_t &out) {
    char s[16],*end;if(!remote_field(body,name,s,sizeof(s))||s[0]<'0'||s[0]>'9')return false;
    uint64_t v=strtoull(s,&end,10);if(*end||v>UINT32_MAX)return false;out=uint32_t(v);return true;
}
static bool remote_float(const char *body,const char *name,float &out) {
    char s[24],*end;if(!remote_field(body,name,s,sizeof(s)))return false;
    out=strtof(s,&end);return end!=s&&!*end&&isfinite(out);
}

static void remote_send_view(void*) {
    char json[sizeof(remote_view)];
    portENTER_CRITICAL(&remote_link_mux);int fd=remote_fd;uint64_t link=remote_link;portEXIT_CRITICAL(&remote_link_mux);
    if(fd>=0&&link&&httpd_ws_get_fd_info(remote_server,fd)==HTTPD_WS_CLIENT_WEBSOCKET) {
        portENTER_CRITICAL(&remote_view_mux);memcpy(json,remote_view,sizeof(json));portEXIT_CRITICAL(&remote_view_mux);
        httpd_ws_frame_t frame={};frame.type=HTTPD_WS_TYPE_TEXT;
        frame.payload=(uint8_t*)json;frame.len=strlen(json);
        if(httpd_ws_send_frame_async(remote_server,fd,&frame)!=ESP_OK)httpd_sess_trigger_close(remote_server,fd);
    }
    portENTER_CRITICAL(&remote_link_mux);remote_tx_pending=false;portEXIT_CRITICAL(&remote_link_mux);
}
static void remote_schedule_publish() {
    portENTER_CRITICAL(&remote_link_mux);
    bool send=remote_fd>=0&&!remote_tx_pending;
    if(send)remote_tx_pending=true;
    portEXIT_CRITICAL(&remote_link_mux);
    if(send&&httpd_queue_work(remote_server,remote_send_view,nullptr)!=ESP_OK) {
        portENTER_CRITICAL(&remote_link_mux);remote_tx_pending=false;portEXIT_CRITICAL(&remote_link_mux);
    }
}
static void remote_socket_close(httpd_handle_t,int fd) {
    portENTER_CRITICAL(&remote_link_mux);
    uint64_t link=fd==remote_fd?remote_link:0;
    if(link){remote_fd=-1;remote_link=0;}
    portEXIT_CRITICAL(&remote_link_mux);
    close(fd);
}
static esp_err_t remote_ws(httpd_req_t *req) {
    int fd=httpd_req_to_sockfd(req);
    if(req->method==HTTP_GET) {
        char origin[64];
        if(httpd_req_get_hdr_value_str(req,"Origin",origin,sizeof(origin))!=ESP_OK||strcmp(origin,"http://192.168.4.1"))return ESP_FAIL;
        portENTER_CRITICAL(&remote_link_mux);bool free=remote_fd<0;portEXIT_CRITICAL(&remote_link_mux);
        if(!free)return ESP_FAIL;
        uint64_t link=((uint64_t)esp_random()<<32)|esp_random();if(!link)link=1;
        portENTER_CRITICAL(&remote_link_mux);remote_fd=fd;remote_link=link;portEXIT_CRITICAL(&remote_link_mux);
        return ESP_OK;
    }
    httpd_ws_frame_t frame={};frame.type=HTTPD_WS_TYPE_TEXT;
    if(httpd_ws_recv_frame(req,&frame,0)!=ESP_OK||frame.len>320||frame.type!=HTTPD_WS_TYPE_TEXT)return ESP_FAIL;
    char body[321]={};frame.payload=(uint8_t*)body;
    if(httpd_ws_recv_frame(req,&frame,frame.len)!=ESP_OK)return ESP_FAIL;
    portENTER_CRITICAL(&remote_link_mux);uint64_t link=fd==remote_fd?remote_link:0;portEXIT_CRITICAL(&remote_link_mux);
    if(!link)return ESP_FAIL;
    char op[20];RemoteCommand q={};q.owner=link;q.received_ms=millis();
    if(!remote_field(body,"op",op,sizeof(op))||!remote_uint(body,"seq",q.sequence))return ESP_FAIL;
    if(!strcmp(op,"stop")){q.op=RemoteOp::Stop;xQueueOverwrite(remote_stop_in,&q);return ESP_OK;}
    if(!strcmp(op,"drive")) {
        q.op=RemoteOp::Drive;
        if(!remote_float(body,"throttle",q.throttle)||!remote_float(body,"steer",q.steer)||
           !remote_float(body,"speed",q.speed)||!remote_uint(body,"seen",q.seen_ms))return ESP_FAIL;
        xQueueOverwrite(remote_drive_in,&q);return ESP_OK;
    }
    if(!strcmp(op,"calibrate"))q.op=RemoteOp::Calibrate;
    else if(!strcmp(op,"clear"))q.op=RemoteOp::Clear;
    else if(!strcmp(op,"move")||!strcmp(op,"turn")) {
        char dir[12];q.op=!strcmp(op,"move")?RemoteOp::Move:RemoteOp::Turn;
        if(!remote_field(body,"dir",dir,sizeof(dir)))return ESP_FAIL;
        q.direction=!strcmp(dir,"forward")?1:!strcmp(dir,"reverse")?-1:!strcmp(dir,"left")?2:!strcmp(dir,"right")?-2:0;
        if(q.op==RemoteOp::Move&&(!remote_float(body,"speed",q.speed)||!remote_float(body,"distance",q.distance)))return ESP_FAIL;
        if(q.op==RemoteOp::Turn&&(!remote_float(body,"angle",q.angle)||!remote_float(body,"seconds",q.seconds)))return ESP_FAIL;
    } else return ESP_FAIL;
    return xQueueSend(remote_in,&q,0)==pdTRUE?ESP_OK:ESP_FAIL;
}
static esp_err_t remote_control(httpd_req_t *req) {
    // Keep an independent STOP path. Old POST motion clients cannot control R3.
    if(req->content_len>64)return remote_json(req,"{\"ok\":false}","400 Bad Request");
    char body[65]={};int n=httpd_req_recv(req,body,req->content_len);
    if(n<=0||!strstr(body,"op=stop"))return remote_json(req,"{\"ok\":false,\"code\":\"use_websocket\"}","409 Conflict");
    RemoteCommand q={};q.op=RemoteOp::Stop;q.received_ms=millis();xQueueOverwrite(remote_stop_in,&q);
    return remote_json(req,"{\"ok\":true}");
}
static esp_err_t remote_asset(httpd_req_t *req) {
    const DashboardAsset *asset=static_cast<const DashboardAsset*>(req->user_ctx);
    httpd_resp_set_type(req,asset->mime);
    httpd_resp_set_hdr(req,"Content-Encoding","gzip");
    httpd_resp_set_hdr(req,"Cache-Control","no-cache");
    httpd_resp_set_hdr(req,"X-Content-Type-Options","nosniff");
    httpd_resp_set_hdr(req,"Content-Security-Policy","default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; font-src 'self'; connect-src 'self' ws://192.168.4.1; frame-ancestors 'none'; base-uri 'none'; form-action 'none'");
    return httpd_resp_send(req,reinterpret_cast<const char*>(asset->bytes),asset->length);
}
static esp_err_t remote_socket_open(httpd_handle_t,int fd) {
    // Small ACK/status bodies must not wait for Nagle + delayed-ACK coalescing.
    int enabled=1;
    return setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&enabled,sizeof(enabled))==0?ESP_OK:ESP_FAIL;
}

static void remote_setup() {
    battery_setup();
    remote_in=xQueueCreate(4,sizeof(RemoteCommand));remote_drive_in=xQueueCreate(1,sizeof(RemoteCommand));remote_stop_in=xQueueCreate(1,sizeof(RemoteCommand));
    remote_startup="mailbox_failed";if(!remote_in||!remote_drive_in||!remote_stop_in)return;
    remote_startup="wifi_mode_failed";if(!WiFi.mode(WIFI_AP))return;
    WiFi.setSleep(false);remote_startup="ap_config_failed";
    if(!WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0)))return;
    remote_startup="ap_start_failed";if(!WiFi.softAP(ROBOT_WIFI_SSID,ROBOT_WIFI_PASSWORD,6,false,2))return;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();cfg.core_id=0;cfg.stack_size=8192;
    cfg.max_uri_handlers=12;cfg.max_open_sockets=7;cfg.lru_purge_enable=true;
    cfg.recv_wait_timeout=1;cfg.send_wait_timeout=1;cfg.open_fn=remote_socket_open;cfg.close_fn=remote_socket_close;
    remote_startup="http_start_failed";if(httpd_start(&remote_server,&cfg)!=ESP_OK)return;
    httpd_uri_t route={};route.uri="/api/state";route.method=HTTP_GET;route.handler=remote_state;httpd_register_uri_handler(remote_server,&route);
    route.uri="/api/control";route.method=HTTP_POST;route.handler=remote_control;httpd_register_uri_handler(remote_server,&route);
    route={};route.uri="/ws";route.method=HTTP_GET;route.handler=remote_ws;route.is_websocket=true;httpd_register_uri_handler(remote_server,&route);
    for(const auto &asset:DASHBOARD_ASSETS) {
        route={};route.uri=asset.path;route.method=HTTP_GET;route.handler=remote_asset;route.user_ctx=const_cast<DashboardAsset*>(&asset);
        httpd_register_uri_handler(remote_server,&route);
    }
    remote_startup="ready";
}
