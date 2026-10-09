#include "power_shutdown.h"

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "ads1292r_driver.h"
#include "battery_adc.h"
#include "ble_link.h"
#include "bmi270_driver.h"

#define SHUTDOWN_REQUESTED   BIT0
#define BOOT_COMPLETE        BIT1
#define SHUTDOWN_POLL_MS      20
#define RUN_STATUS_GPIO      GPIO_NUM_13
static const char *TAG = "power_shutdown";
static EventGroupHandle_t s_events;
static EventBits_t s_workers;

bool power_shutdown_requested(void)
{
    return s_events && (xEventGroupGetBits(s_events) & SHUTDOWN_REQUESTED) != 0;
}

void power_shutdown_worker_started(EventBits_t worker)
{
    s_workers |= worker;
}

void power_shutdown_boot_complete(void)
{
    xEventGroupSetBits(s_events, BOOT_COMPLETE);
}

void power_shutdown_checkpoint(EventBits_t worker)
{
    if (!power_shutdown_requested()) return;
    if (worker != 0) xEventGroupSetBits(s_events, worker);
    /* No code can resume a parked worker, even if GPIO9 goes HIGH again. */
    for (;;) vTaskSuspend(NULL);
}

void power_shutdown_boot_checkpoint(void)
{
    if (!power_shutdown_requested()) return;
    power_shutdown_boot_complete();
    power_shutdown_checkpoint(0);
}

void power_shutdown_wait(uint32_t ms)
{
    (void)xEventGroupWaitBits(s_events, SHUTDOWN_REQUESTED, pdFALSE, pdFALSE,
                             pdMS_TO_TICKS(ms));
}

static void check_gpio(esp_err_t err, const char *operation, int pin)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s GPIO%d failed: %s", operation, pin, esp_err_to_name(err));
    }
}

static void hold_output(gpio_num_t pin, uint32_t level)
{
    /* Preload the latch before connecting the GPIO output mux. */
    check_gpio(gpio_set_level(pin, level), "set", pin);
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    check_gpio(gpio_config(&config), "output", pin);
    check_gpio(gpio_hold_en(pin), "hold", pin);
}

static void hold_unpulled_inputs(void)
{
    /* SDA/SCL have EXTERNAL pull-ups: never drive either LOW in sleep.
     * Also remove the GPIO9 pull-up so a held button does not waste current.
     * Disable digital input buffers as well; none is used as a wake source. */
    const gpio_num_t pins[] = {POWER_SHUTDOWN_GPIO, BMI270_INT1_IO,
        BMI270_I2C_SDA_IO, BMI270_I2C_SCL_IO, ADS1292R_DRDY_GPIO,
        ADS1292R_SPI_MISO_GPIO, BATTERY_ADC_GPIO};
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        const gpio_config_t config = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode = GPIO_MODE_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        check_gpio(gpio_config(&config), "input", pins[i]);
        check_gpio(gpio_hold_en(pins[i]), "hold", pins[i]);
    }
}

static void enter_power_off(void) __attribute__((noreturn));
static void enter_power_off(void)
{
    /* All peripheral owners (including app_main) are now quiescent. */
    (void)gpio_intr_disable(ADS1292R_DRDY_GPIO);
    (void)gpio_intr_disable(BMI270_INT1_IO);
    bool ads_off = ads1292r_power_down();
    bool bmi_off = bmi270_driver_power_down();
    battery_adc_deinit();
    esp_err_t ble_err = ble_link_shutdown();
    ESP_LOGI(TAG, "shutdown: ADS PWDN=%s BMI270 suspend=%s BLE=%s",
             ads_off ? "LOW" : "FAILED", bmi_off ? "verified" : "FAILED",
             esp_err_to_name(ble_err));
    if (!ads_off || !bmi_off) {
        ESP_LOGE(TAG, "sensor power-down was not verified; board current must be checked");
    }

    hold_output(ADS1292R_RESET_GPIO, 0); /* PWDN stays LOW, NOT a reset pulse. */
    hold_output(ADS1292R_START_GPIO, 0);
    hold_output(ADS1292R_CS_GPIO, 1);
    hold_output(ADS1292R_SPI_SCLK_GPIO, 0);
    hold_output(ADS1292R_SPI_MOSI_GPIO, 0);
    hold_output(RUN_STATUS_GPIO, 0);
    hold_unpulled_inputs();
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_en();
#endif
    /* H2 supports single-pin holds across Deep-sleep. No timer, GPIO,
     * UART, USB, BLE or other wake source is enabled anywhere here. */
    esp_err_t err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    if (err != ESP_OK) ESP_LOGE(TAG, "disable wake sources: %s", esp_err_to_name(err));
    ESP_LOGI(TAG, "POWER OFF: no wake sources; release GPIO9 then use hardware RST to boot");
    fflush(stdout);
    esp_deep_sleep_start();
}

static void shutdown_task(void *arg)
{
    (void)arg;
    int64_t low_since = -1;
    for (;;) {
        if (gpio_get_level(POWER_SHUTDOWN_GPIO) == 0) {
            int64_t now = esp_timer_get_time();
            if (low_since < 0) low_since = now;
            if (now - low_since >= POWER_SHUTDOWN_DEBOUNCE_MS * 1000LL) break;
        } else {
            low_since = -1;
        }
        vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_POLL_MS));
    }

    xEventGroupSetBits(s_events, SHUTDOWN_REQUESTED);
    ESP_LOGW(TAG, "GPIO9 LOW: shutdown latched; waiting for safe task boundaries");
    /* Boot may be inside ADS reset or BMI config upload. Do NOT interrupt
     * those transactions or tear down their buses underneath them. */
    xEventGroupWaitBits(s_events, BOOT_COMPLETE, pdFALSE, pdTRUE, portMAX_DELAY);
    EventBits_t required = s_workers;
    while (required != 0) {
        EventBits_t parked = xEventGroupWaitBits(s_events, required, pdFALSE, pdTRUE,
                                                pdMS_TO_TICKS(1000));
        if ((parked & required) == required) break;
        ESP_LOGW(TAG, "waiting for workers: pending=0x%02lX",
                 (unsigned long)(required & ~parked));
    }
    enter_power_off();
}

void power_shutdown_init(void)
{
    /* Release our retained pads on a real reset, before drivers reconfigure
     * them. GPIO0 is still LOW until the ADS startup sequence raises it. */
    const gpio_num_t pins[] = {ADS1292R_RESET_GPIO, ADS1292R_START_GPIO,
        ADS1292R_CS_GPIO, ADS1292R_SPI_SCLK_GPIO, ADS1292R_SPI_MOSI_GPIO,
        RUN_STATUS_GPIO, POWER_SHUTDOWN_GPIO, BMI270_INT1_IO, BMI270_I2C_SDA_IO,
        BMI270_I2C_SCL_IO, ADS1292R_DRDY_GPIO, ADS1292R_SPI_MISO_GPIO, BATTERY_ADC_GPIO};
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        check_gpio(gpio_hold_dis(pins[i]), "release hold", pins[i]);
    }
#if !SOC_GPIO_SUPPORT_HOLD_SINGLE_IO_IN_DSLP
    gpio_deep_sleep_hold_dis();
#endif
    const gpio_config_t input = {
        .pin_bit_mask = 1ULL << POWER_SHUTDOWN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&input);
    s_events = xEventGroupCreate();
    if (err != ESP_OK || !s_events ||
        xTaskCreate(shutdown_task, "power_off", 4096, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "cannot start shutdown monitor; fail safe into power-off");
        /* No other peripheral tasks have been started at this point. */
        enter_power_off();
    }
    ESP_LOGI(TAG, "GPIO9 shutdown input armed: pull-up, LOW debounce %d ms, reset-only recovery",
             POWER_SHUTDOWN_DEBOUNCE_MS);
}
