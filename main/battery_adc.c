#include "battery_adc.h"

#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>

static const char *TAG = "battery";
#define BATTERY_ADC_UNIT     ADC_UNIT_1
#define BATTERY_ADC_CHANNEL  ADC_CHANNEL_1 /* H2 GPIO2 = ADC1_CH1 */
#define BATTERY_ADC_ATTEN    ADC_ATTEN_DB_12
#define BATTERY_ADC_SAMPLES  16

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;

bool battery_adc_init(void)
{
    if (s_adc_handle != NULL && s_cali_handle != NULL) {
        return true;
    }
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed: %s", esp_err_to_name(err));
        return false;
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);
    if (err == ESP_OK) {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = BATTERY_ADC_UNIT,
            .chan = BATTERY_ADC_CHANNEL,
            .atten = BATTERY_ADC_ATTEN,
            .bitwidth = ADC_BITWIDTH_12,
        };
        err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    }
    if (err != ESP_OK) {
        /* An uncalibrated full-scale estimate is not a voltage measurement. */
        ESP_LOGW(TAG, "battery calibration unavailable: %s; report n/a", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        s_cali_handle = NULL;
        return false;
    }
    ESP_LOGI(TAG, "battery ADC: GPIO%d, upper 13k / lower 15k, calibrated", BATTERY_ADC_GPIO);
    return true;
}

void battery_adc_deinit(void)
{
    if (s_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(s_cali_handle);
        s_cali_handle = NULL;
    }
    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

bool battery_adc_read_mv(uint32_t *battery_mv)
{
    if (battery_mv == NULL || s_adc_handle == NULL || s_cali_handle == NULL) {
        return false;
    }
    int sum_mv = 0;
    for (int i = 0; i < BATTERY_ADC_SAMPLES; ++i) {
        int raw = 0, node_mv = 0;
        if (adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw) != ESP_OK ||
            raw <= 0 || raw >= 4095 ||
            adc_cali_raw_to_voltage(s_cali_handle, raw, &node_mv) != ESP_OK || node_mv < 0) {
            return false;
        }
        sum_mv += node_mv;
    }
    /* Average calibrated battery readings; round to the nearest mV.
     * GPIO2 -- 15k -- GND; battery+ -- 13k -- GPIO2. */
    const uint32_t denominator = BATTERY_ADC_SAMPLES * BATTERY_DIV_DENOMINATOR;
    *battery_mv = ((uint32_t)sum_mv * BATTERY_DIV_NUMERATOR + denominator / 2) / denominator;
    return true;
}
