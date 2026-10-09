/* Standalone BMI270 bring-up. No ECG task, NVS, BLE or 802.15.4 init. */
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "bmi270_driver.h"

#if !CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#error "BMI270 validation requires the native USB Serial/JTAG console"
#endif

static const char *TAG = "bmi270_test";
#define USB_QUEUE_DEPTH 64
#define INTERRUPT_TIMEOUT_MS 60 /* >40 ms at 25 Hz; avoids timeout/DRDY deadline races. */
#define DIAGNOSTIC_INTERVAL_US 5000000LL

/* These are the existing board's ADS controls, not BMI270 pins. Holding RESET
 * LOW keeps the unused ADS unconfigured; GPIO13's ECG indicator stays LOW. */
#define UNUSED_ADS_RESET_IO 0
#define UNUSED_ADS_START_IO 23
#define UNUSED_ADS_CS_IO    1
#define ECG_INDICATOR_IO   13

typedef struct {
    uint32_t sequence;
    int64_t timestamp_us;      /* ESP uptime at register read, NOT host local time. */
    bool via_interrupt;
    bmi270_data_t sensor;
} usb_sample_t;

typedef struct {
    uint32_t samples, acc_ready, gyr_ready, irq_samples, irq_timeouts, empty_reads, read_errors, usb_drops;
    bool latest_valid, temp_valid, diagnostic_valid;
    bool has_irq, has_poll;
    int16_t temp_c100;
    uint32_t temp_updates;
    int64_t temp_timestamp_us;
    bmi270_data_t latest;
    bmi270_diagnostics_t diagnostic;
} statistics_t;

static QueueHandle_t s_usb_queue;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;
static statistics_t s_stats;

static void park_unused_ecg(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = BIT64(UNUSED_ADS_RESET_IO) | BIT64(UNUSED_ADS_START_IO) |
                        BIT64(UNUSED_ADS_CS_IO) | BIT64(ECG_INDICATOR_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_ERROR_CHECK(gpio_set_level(UNUSED_ADS_RESET_IO, 0));
    ESP_ERROR_CHECK(gpio_set_level(UNUSED_ADS_START_IO, 0));
    ESP_ERROR_CHECK(gpio_set_level(UNUSED_ADS_CS_IO, 1));
    ESP_ERROR_CHECK(gpio_set_level(ECG_INDICATOR_IO, 0));
}

static void acquisition_task(void *arg)
{
    (void)arg;
    int64_t last_diagnostic = 0;
    while (true) {
        bool interrupt = bmi270_driver_wait_data_ready(INTERRUPT_TIMEOUT_MS);
        usb_sample_t message = {.via_interrupt = interrupt};
        bool read_ok = bmi270_driver_read(&message.sensor);
        message.timestamp_us = esp_timer_get_time();
        /* Data-ready bits, not an I2C ACK alone, decide whether data is new.
         * Both axes are sent, but each sensor's validity is in its STATUS bit. */
        bool acc_ready = read_ok && (message.sensor.status & BMI270_DRDY_ACC);
        bool gyr_ready = read_ok && (message.sensor.status & BMI270_DRDY_GYR);
        portENTER_CRITICAL(&s_stats_mux);
        if (interrupt) { s_stats.has_irq = true; ++s_stats.irq_samples; }
        else { s_stats.has_poll = true; ++s_stats.irq_timeouts; }
        if (!read_ok) ++s_stats.read_errors;
        if (read_ok && !acc_ready && !gyr_ready) ++s_stats.empty_reads;
        if (acc_ready) ++s_stats.acc_ready;
        if (gyr_ready) ++s_stats.gyr_ready;
        if (acc_ready || gyr_ready) {
            message.sequence = ++s_stats.samples;
            s_stats.latest = message.sensor;
            s_stats.latest_valid = true;
        }
        portEXIT_CRITICAL(&s_stats_mux);
        if ((acc_ready || gyr_ready) && xQueueSend(s_usb_queue, &message, 0) != pdTRUE) {
            portENTER_CRITICAL(&s_stats_mux);
            ++s_stats.usb_drops;
            portEXIT_CRITICAL(&s_stats_mux);
        }
        if (message.timestamp_us - last_diagnostic >= DIAGNOSTIC_INTERVAL_US) {
            bmi270_diagnostics_t diagnostic;
            int16_t temperature = 0;
            bool temp_ok = bmi270_driver_read_temp(&temperature);
            bool diag_ok = bmi270_driver_diagnostics(&diagnostic);
            portENTER_CRITICAL(&s_stats_mux);
            s_stats.temp_valid = temp_ok;
            if (temp_ok) {
                s_stats.temp_c100 = temperature;
                s_stats.temp_timestamp_us = message.timestamp_us;
                ++s_stats.temp_updates;
            }
            s_stats.diagnostic_valid = diag_ok;
            if (diag_ok) s_stats.diagnostic = diagnostic;
            portEXIT_CRITICAL(&s_stats_mux);
            last_diagnostic = message.timestamp_us;
        }
        if (!read_ok) vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void print_summary(const statistics_t *s, const statistics_t *previous, double seconds)
{
    uint32_t irq_total = s->diagnostic_valid ? s->diagnostic.int1_events : s->irq_samples;
    printf("# STATS samples=%" PRIu32 " acc_hz=%.1f gyr_hz=%.1f irq_hz=%.1f "
           "irq_total=%" PRIu32 " irq_coalesced=%" PRIu32 " timeouts_total=%" PRIu32
           " empty_total=%" PRIu32 " read_errors_total=%" PRIu32 " usb_drops_total=%" PRIu32 "\n",
           s->samples, (s->acc_ready - previous->acc_ready) / seconds,
           (s->gyr_ready - previous->gyr_ready) / seconds,
           (s->irq_samples - previous->irq_samples) / seconds,
           irq_total, s->diagnostic_valid ? s->diagnostic.coalesced_events : 0,
           s->irq_timeouts, s->empty_reads, s->read_errors, s->usb_drops);
    if (s->latest_valid) {
        double ax = s->latest.acc_x * BMI270_ACC_RANGE_G / 32768.0;
        double ay = s->latest.acc_y * BMI270_ACC_RANGE_G / 32768.0;
        double az = s->latest.acc_z * BMI270_ACC_RANGE_G / 32768.0;
        printf("# VALUES acc_g=[%.4f,%.4f,%.4f] norm_g=%.4f "
               "gyro_dps=[%.3f,%.3f,%.3f] ", ax, ay, az, sqrt(ax*ax + ay*ay + az*az),
               s->latest.gyr_x * BMI270_GYR_RANGE_DPS / 32768.0,
               s->latest.gyr_y * BMI270_GYR_RANGE_DPS / 32768.0,
               s->latest.gyr_z * BMI270_GYR_RANGE_DPS / 32768.0);
        if (s->temp_valid) printf("temp_c=%.2f\n", s->temp_c100 / 100.0);
        else printf("temp_c=NA\n");
    }
    if (s->diagnostic_valid) {
        const bmi270_diagnostics_t *d = &s->diagnostic;
        printf("# REG addr=0x%02X chip_id=0x%02X internal=0x%02X err=0x%02X "
               "status=0x%02X pwr=0x%02X int_status=0x%04X int1_ctrl=0x%02X map=0x%02X int1_level=%d\n",
               d->address, d->chip_id, d->internal_status, d->error_reg,
               d->status, d->power_ctrl, d->int_status, d->int1_ctrl, d->int_map_data, d->int1_level);
    } else {
        printf("# WARNING sensor diagnostic read unavailable\n");
    }
    if (s->irq_timeouts != previous->irq_timeouts) {
        printf("# WARNING INT1 timeout: POLL rows are fallback, not an interrupt validation PASS. "
               "Check BMI270 INT1 -> GPIO10.\n");
    }
}

static void usb_output_task(void *arg)
{
    (void)arg;
    printf("# BMI270_USB_V1 raw signed16; source=IRQ or POLL; uptime_us is ESP time\n");
    printf("type,seq,uptime_us,sensor_time_ticks,status,source,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw\n");
    printf("# TEMP format: TEMP,uptime_us,temp_c100,temp_c\n");
    int64_t last_summary = esp_timer_get_time();
    uint32_t last_temp_updates = 0;
    statistics_t previous = {0};
    while (true) {
        usb_sample_t message;
        if (xQueueReceive(s_usb_queue, &message, pdMS_TO_TICKS(50)) == pdTRUE) {
            const bmi270_data_t *s = &message.sensor;
            printf("IMU,%" PRIu32 ",%" PRId64 ",%" PRIu32 ",0x%02X,%s,%d,%d,%d,%d,%d,%d\n",
                   message.sequence, message.timestamp_us, s->sensor_time, s->status,
                   message.via_interrupt ? "IRQ" : "POLL",
                   s->acc_x, s->acc_y, s->acc_z, s->gyr_x, s->gyr_y, s->gyr_z);
        }
        statistics_t current;
        portENTER_CRITICAL(&s_stats_mux);
        current = s_stats;
        portEXIT_CRITICAL(&s_stats_mux);
        if (current.temp_valid && current.temp_updates != last_temp_updates) {
            printf("TEMP,%" PRId64 ",%d,%.2f\n", current.temp_timestamp_us,
                   current.temp_c100, current.temp_c100 / 100.0);
            last_temp_updates = current.temp_updates;
        }
        int64_t now = esp_timer_get_time();
        if (now - last_summary >= 1000000) {
            print_summary(&current, &previous, (now - last_summary) / 1000000.0);
            previous = current;
            last_summary = now;
        }
    }
}

void app_main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    park_unused_ecg();
    ESP_LOGI(TAG, "BMI270 standalone test: native USB only; wireless is not initialized");
    ESP_LOGI(TAG, "SDA=GPIO%d SCL=GPIO%d INT1=GPIO%d I2C=%d Hz",
             BMI270_I2C_SDA_IO, BMI270_I2C_SCL_IO, BMI270_INT1_IO, BMI270_I2C_FREQ_HZ);
    vTaskDelay(pdMS_TO_TICKS(500));
    bmi270_driver_i2c_scan();
    while (true) {
        if (bmi270_driver_init() && bmi270_driver_start_data_ready_int()) break;
        bmi270_driver_deinit();
        ESP_LOGE(TAG, "Bring-up failed; retry in 5s. Check power, CSB=HIGH, SDO=GND, external I2C pull-ups and wiring.");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    s_usb_queue = xQueueCreate(USB_QUEUE_DEPTH, sizeof(usb_sample_t));
    if (!s_usb_queue ||
        xTaskCreate(usb_output_task, "bmi_usb", 5120, NULL, 4, NULL) != pdPASS ||
        xTaskCreate(acquisition_task, "bmi_acquire", 6144, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to allocate USB/acquisition queue or task; sensor test NOT running");
        bmi270_driver_deinit();
        while (true) vTaskDelay(pdMS_TO_TICKS(5000));
    }
    /* Returning deletes the ESP-IDF main task; acquisition and USB tasks remain. */
}
