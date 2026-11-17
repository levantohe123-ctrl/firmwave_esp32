#pragma once
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "battery_estimator.h"

static portMUX_TYPE battery_mux=portMUX_INITIALIZER_UNLOCKED;
static BatteryView battery_shared;
static esp_adc_cal_value_t battery_cal_source=ESP_ADC_CAL_VAL_DEFAULT_VREF;
static int battery_adc_raw=-1;
static uint32_t battery_adc_mv=0;
static const char *battery_adc_status="starting";

static BatteryView battery_snapshot(uint32_t ms) {
    portENTER_CRITICAL(&battery_mux);BatteryView v=battery_shared;portEXIT_CRITICAL(&battery_mux);
    if(!battery_sample_fresh(ms,v.sample_ms))v.valid=false;
    return v;
}
static void battery_task(void*) {
    BatteryEstimator estimator;
    esp_adc_cal_characteristics_t cal={};
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_DB_11); // GPIO32 / ADC1: Wi-Fi compatible.
    auto source=esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_DB_11,ADC_WIDTH_BIT_12,1100,&cal);
    portENTER_CRITICAL(&battery_mux);battery_cal_source=source;portEXIT_CRITICAL(&battery_mux);
    TickType_t wake=xTaskGetTickCount();
    for(;;) {
        uint32_t ms=millis();
        portENTER_CRITICAL(&output_mux);
        bool loaded=d2_running(ctrl)||ctrl.outputs||!d2_stationary(ctrl,ms);
        portEXIT_CRITICAL(&output_mux);
        // ADC conversion and filtering are outside every motor critical section.
        int raw=adc1_get_raw(ADC1_CHANNEL_4);
        uint32_t millivolts=raw<0?0:esp_adc_cal_raw_to_voltage(raw,&cal);
        float mv=raw<=0||raw>=4095?NAN:(float)millivolts;
        estimator.update(mv,loaded,ms);
        auto v=estimator.snapshot(ms);
        const char *reason=raw<0?"adc_error":raw==0?"no_voltage":raw>=4095?"adc_saturated":
            millivolts<500?"input_low":millivolts>2600?"input_high":v.valid?"ok":"warming_up";
        portENTER_CRITICAL(&battery_mux);
        battery_shared=v;battery_adc_raw=raw;battery_adc_mv=millivolts;battery_adc_status=reason;
        portEXIT_CRITICAL(&battery_mux);
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(50));
    }
}
static void battery_setup() {
    // Failure to create this optional observer leaves battery data unknown.
    xTaskCreatePinnedToCore(battery_task,"battery_adc",3072,nullptr,1,nullptr,0);
}
