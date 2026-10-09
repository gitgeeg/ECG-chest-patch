#include "bmi270_driver.h"

#include <limits.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "bmi270.h"

static const char *TAG = "bmi270";
#define I2C_TIMEOUT_MS 100
#define READ_WRITE_LEN 46 /* Even-sized chunks for Bosch's 8192-byte config. */
#define BMI270_ERROR_REG_ADDR 0x02

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_device;
static struct bmi2_dev s_bmi;
static SemaphoreHandle_t s_ready;
static bool s_initialized, s_isr_added;
static uint8_t s_address, s_last_sample_status;
static int8_t s_last_api_error;
static esp_err_t s_last_i2c_error;
static portMUX_TYPE s_isr_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_int1_events, s_coalesced_events;

static void IRAM_ATTR int1_isr(void *arg)
{
    (void)arg;
    BaseType_t wake = pdFALSE;
    BaseType_t given = xSemaphoreGiveFromISR(s_ready, &wake);
    portENTER_CRITICAL_ISR(&s_isr_mux);
    ++s_int1_events;
    if (given != pdTRUE) ++s_coalesced_events;
    portEXIT_CRITICAL_ISR(&s_isr_mux);
    if (wake) portYIELD_FROM_ISR();
}

static BMI2_INTF_RETURN_TYPE sensor_read(uint8_t reg, uint8_t *data, uint32_t len, void *ctx)
{
    esp_err_t err = i2c_master_transmit_receive((i2c_master_dev_handle_t)ctx,
                                               &reg, 1, data, len, I2C_TIMEOUT_MS);
    s_last_i2c_error = err;
    return err == ESP_OK ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE sensor_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ctx)
{
    uint8_t buffer[READ_WRITE_LEN + 1];
    if (len > READ_WRITE_LEN) {
        s_last_i2c_error = ESP_ERR_INVALID_SIZE;
        return BMI2_E_COM_FAIL;
    }
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);
    esp_err_t err = i2c_master_transmit((i2c_master_dev_handle_t)ctx,
                                      buffer, len + 1, I2C_TIMEOUT_MS);
    s_last_i2c_error = err;
    return err == ESP_OK ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static void sensor_delay(uint32_t us, void *ctx)
{
    (void)ctx;
    /* Long init waits should yield to FreeRTOS, short bus waits stay precise. */
    TickType_t ticks = pdMS_TO_TICKS(us / 1000);
    if (ticks > 1) {
        vTaskDelay(ticks - 1);
        us -= (ticks - 1) * (1000000U / configTICK_RATE_HZ);
    }
    esp_rom_delay_us(us);
}

static bool api_ok(const char *operation, int8_t result)
{
    s_last_api_error = result;
    if (result == BMI2_OK) return true;
    ESP_LOGE(TAG, "%s: SensorAPI=%d, I2C=%s", operation, result,
             esp_err_to_name(s_last_i2c_error));
    return false;
}

static bool ensure_bus(void)
{
    if (s_bus) return true;
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BMI270_I2C_SDA_IO,
        .scl_io_num = BMI270_I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        /* The board already has external SDA/SCL pull-ups. */
        .flags.enable_internal_pullup = false,
    };
    esp_err_t err = i2c_new_master_bus(&config, &s_bus);
    if (err != ESP_OK) ESP_LOGE(TAG, "I2C bus creation: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

void bmi270_driver_i2c_scan(void)
{
    bool temporary_bus = !s_bus;
    if (!ensure_bus()) return;
    unsigned found = 0;
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d", BMI270_I2C_SDA_IO, BMI270_I2C_SCL_IO);
    for (uint16_t addr = 0x03; addr <= 0x77; ++addr) {
        esp_err_t err = i2c_master_probe(s_bus, addr, 20);
        if (err == ESP_OK) {
            ++found;
            ESP_LOGI(TAG, "I2C ACK 0x%02X%s", addr,
                     addr == 0x68 || addr == 0x69 ? " (BMI270 candidate)" : "");
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "I2C bus stuck / timeout; check SDA/SCL/pull-ups/power");
            break;
        }
    }
    if (!found) ESP_LOGW(TAG, "No I2C device detected; expected 0x68 or 0x69");
    if (temporary_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

void bmi270_driver_deinit(void)
{
    if (s_isr_added) {
        gpio_intr_disable(BMI270_INT1_IO);
        gpio_isr_handler_remove(BMI270_INT1_IO);
        s_isr_added = false;
    }
    if (s_ready) {
        vSemaphoreDelete(s_ready);
        s_ready = NULL;
    }
    if (s_device) {
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_initialized = false;
    s_address = s_last_sample_status = 0;
    memset(&s_bmi, 0, sizeof(s_bmi));
}

static bool detect_sensor(void)
{
    const uint8_t addresses[] = {BMI2_I2C_PRIM_ADDR, BMI2_I2C_SEC_ADDR};
    for (unsigned i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        uint8_t address = addresses[i];
        if (i2c_master_probe(s_bus, address, I2C_TIMEOUT_MS) != ESP_OK) continue;
        const i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = BMI270_I2C_FREQ_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(s_bus, &config, &s_device);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C add device: %s", esp_err_to_name(err));
            return false;
        }
        uint8_t reg = BMI2_CHIP_ID_ADDR, chip_id = 0;
        err = i2c_master_transmit_receive(s_device, &reg, 1, &chip_id, 1, I2C_TIMEOUT_MS);
        ESP_LOGI(TAG, "address=0x%02X CHIP_ID=0x%02X (expected 0x%02X), %s",
                 address, chip_id, BMI270_CHIP_ID, esp_err_to_name(err));
        if (err == ESP_OK && chip_id == BMI270_CHIP_ID) {
            s_address = address;
            return true;
        }
        i2c_master_bus_rm_device(s_device);
        s_device = NULL;
    }
    ESP_LOGE(TAG, "BMI270 not identified at 0x68 or 0x69");
    return false;
}

bool bmi270_driver_init(void)
{
    if (s_initialized) return true;
    bmi270_driver_deinit();
    s_last_api_error = BMI2_OK;
    s_last_i2c_error = ESP_OK;
    portENTER_CRITICAL(&s_isr_mux);
    s_int1_events = s_coalesced_events = 0;
    portEXIT_CRITICAL(&s_isr_mux);
    if (!ensure_bus() || !detect_sensor()) goto fail;

    s_bmi.intf = BMI2_I2C_INTF;
    s_bmi.read = sensor_read;
    s_bmi.write = sensor_write;
    s_bmi.delay_us = sensor_delay;
    s_bmi.read_write_len = READ_WRITE_LEN;
    s_bmi.intf_ptr = s_device;
    /* NULL makes bmi270_init use Bosch's bundled config image. */
    if (!api_ok("bmi270_init/config upload", bmi270_init(&s_bmi))) goto fail;
    if (!api_ok("disable advanced power save", bmi2_set_adv_power_save(BMI2_DISABLE, &s_bmi))) goto fail;

    uint8_t internal = 0;
    if (!api_ok("read internal status", bmi2_get_internal_status(&internal, &s_bmi))) goto fail;
    ESP_LOGI(TAG, "CHIP_ID=0x%02X config_size=%u INTERNAL_STATUS=0x%02X",
             s_bmi.chip_id, s_bmi.config_size, internal);
    if (internal != BMI2_INIT_OK) {
        ESP_LOGE(TAG, "Configuration did not report INIT_OK=0x01");
        goto fail;
    }

    struct bmi2_sens_config config[2] = {{.type = BMI2_ACCEL}, {.type = BMI2_GYRO}};
    if (!api_ok("get sensor config", bmi2_get_sensor_config(config, 2, &s_bmi))) goto fail;
    config[0].cfg.acc.odr = BMI2_ACC_ODR_25HZ;
    config[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    config[1].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;
    config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    if (!api_ok("set sensor config", bmi2_set_sensor_config(config, 2, &s_bmi))) goto fail;
    if (!api_ok("verify sensor config", bmi2_get_sensor_config(config, 2, &s_bmi))) goto fail;
    if (config[0].cfg.acc.odr != BMI2_ACC_ODR_25HZ || config[0].cfg.acc.range != BMI2_ACC_RANGE_2G ||
        config[1].cfg.gyr.odr != BMI2_GYR_ODR_25HZ || config[1].cfg.gyr.range != BMI2_GYR_RANGE_2000) {
        ESP_LOGE(TAG, "Sensor configuration read-back mismatch");
        goto fail;
    }
    /* Live-register bring-up, NOT a FIFO watermark interrupt. Configure the
     * BMI270 interrupt route before enabling the sensors, matching Bosch's
     * accel_gyro_hw_int example and preventing the first DRDY event from
     * occurring before the route is active. */
    const struct bmi2_int_pin_config int_config = {
        .pin_type = BMI2_INT1,
        .int_latch = BMI2_INT_NON_LATCH,
        .pin_cfg = {[0] = {
            .lvl = BMI2_INT_ACTIVE_LOW,
            .od = BMI2_INT_PUSH_PULL,
            .output_en = BMI2_INT_OUTPUT_ENABLE,
            .input_en = BMI2_INT_INPUT_DISABLE,
        }},
    };
    /* SensorAPI takes a non-const pointer, so use a mutable local copy. */
    struct bmi2_int_pin_config mutable_config = int_config;
    if (!api_ok("configure INT1", bmi2_set_int_pin_config(&mutable_config, &s_bmi))) goto fail;
    if (!api_ok("map DRDY to INT1", bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, &s_bmi))) goto fail;
    const uint8_t sensors[] = {BMI2_ACCEL, BMI2_GYRO, BMI2_TEMP};
    if (!api_ok("enable accel/gyro/temp", bmi2_sensor_enable(sensors, 3, &s_bmi))) goto fail;
    s_initialized = true;
    ESP_LOGI(TAG, "Ready: accel 25Hz +/-2g, gyro 25Hz +/-2000dps, INT1=GPIO%d active-low", BMI270_INT1_IO);
    return true;
fail:
    bmi270_driver_deinit();
    return false;
}

bool bmi270_driver_start_data_ready_int(void)
{
    if (!s_initialized) return false;
    if (s_isr_added) return true;
    s_ready = xSemaphoreCreateBinary();
    if (!s_ready) return false;
    const gpio_config_t config = {
        .pin_bit_mask = BIT64(BMI270_INT1_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) goto fail;
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) goto fail;
    err = gpio_isr_handler_add(BMI270_INT1_IO, int1_isr, NULL);
    if (err != ESP_OK) goto fail;
    s_isr_added = true;
    err = gpio_set_intr_type(BMI270_INT1_IO, GPIO_INTR_NEGEDGE);
    if (err == ESP_OK) err = gpio_intr_enable(BMI270_INT1_IO);
    if (err != ESP_OK) goto fail;
    ESP_LOGI(TAG, "INT1 ISR enabled on GPIO%d (falling edge)", BMI270_INT1_IO);
    return true;
fail:
    ESP_LOGE(TAG, "INT1 setup: %s", esp_err_to_name(err));
    if (s_isr_added) {
        gpio_isr_handler_remove(BMI270_INT1_IO);
        s_isr_added = false;
    }
    vSemaphoreDelete(s_ready);
    s_ready = NULL;
    return false;
}

bool bmi270_driver_wait_data_ready(uint32_t timeout_ms)
{
    return s_ready && xSemaphoreTake(s_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool bmi270_driver_read(bmi270_data_t *out)
{
    if (!out || !s_initialized) return false;
    struct bmi2_sens_data data = {0};
    if (!api_ok("read sensor data", bmi2_get_sensor_data(&data, &s_bmi))) return false;
    s_last_sample_status = data.status;
    *out = (bmi270_data_t) {
        .acc_x = data.acc.x, .acc_y = data.acc.y, .acc_z = data.acc.z,
        .gyr_x = data.gyr.x, .gyr_y = data.gyr.y, .gyr_z = data.gyr.z,
        .status = data.status, .sensor_time = data.sens_time,
    };
    return true;
}

bool bmi270_driver_read_temp(int16_t *temp_c100)
{
    if (!temp_c100 || !s_initialized) return false;
    int16_t raw = 0;
    if (!api_ok("read temperature", bmi2_get_temperature_data(&raw, &s_bmi))) return false;
    /* -32768 is the sensor's invalid temperature sentinel, not -41 degrees. */
    if (raw == INT16_MIN) return false;
    *temp_c100 = (int16_t)(2300 + ((int32_t)raw * 100) / 512);
    return true;
}

static bool shutdown_write(uint8_t reg, uint8_t value)
{
    bool ok = sensor_write(reg, &value, 1, s_device) == BMI2_INTF_RET_SUCCESS;
    /* Be conservative with APS on/off transitions (Bosch requires 450 us
     * for accesses in low-power mode). No config-image upload is needed. */
    sensor_delay(1000, NULL);
    return ok;
}

bool bmi270_driver_power_down(void)
{
    if (s_isr_added) {
        gpio_intr_disable(BMI270_INT1_IO);
        gpio_isr_handler_remove(BMI270_INT1_IO);
        s_isr_added = false;
    }
    bool verified = false;
    if (ensure_bus() && (s_device || detect_sensor())) {
        for (unsigned attempt = 0; attempt < 3 && !verified; ++attempt) {
            bool ok = shutdown_write(BMI2_PWR_CONF_ADDR, 0x00); /* APS off while programming. */
            /* Do not short-circuit: try shutting down every block even when
             * one transaction fails. PWR_CTRL=0 turns AUX/GYR/ACC/TEMP off. */
            ok = shutdown_write(BMI2_INT_MAP_DATA_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_INT1_MAP_FEAT_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_INT2_MAP_FEAT_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_INT1_IO_CTRL_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_INT2_IO_CTRL_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_PWR_CTRL_ADDR, 0x00) && ok;
            ok = shutdown_write(BMI2_PWR_CONF_ADDR, BMI2_ADV_POW_EN_MASK) && ok;
            uint8_t power = 0xFF, conf = 0;
            bool read_power = sensor_read(BMI2_PWR_CTRL_ADDR, &power, 1, s_device) == BMI2_INTF_RET_SUCCESS;
            sensor_delay(1000, NULL);
            bool read_conf = sensor_read(BMI2_PWR_CONF_ADDR, &conf, 1, s_device) == BMI2_INTF_RET_SUCCESS;
            sensor_delay(1000, NULL);
            verified = ok && read_power && read_conf && power == 0 &&
                       (conf & BMI2_ADV_POW_EN_MASK) != 0;
        }
    }
    if (verified) ESP_LOGI(TAG, "suspended: PWR_CTRL=0x00, advanced power save enabled");
    else ESP_LOGE(TAG, "suspend NOT verified: I2C=%s", esp_err_to_name(s_last_i2c_error));
    bmi270_driver_deinit();
    return verified;
}

bool bmi270_driver_diagnostics(bmi270_diagnostics_t *out)
{
    if (!out || !s_initialized) return false;
    memset(out, 0, sizeof(*out));
    out->last_api_error = s_last_api_error;
    out->last_i2c_error = s_last_i2c_error;
    out->address = s_address;
    out->chip_id = s_bmi.chip_id;
    out->status = s_last_sample_status;
    out->int1_level = gpio_get_level(BMI270_INT1_IO);
    portENTER_CRITICAL(&s_isr_mux);
    out->int1_events = s_int1_events;
    out->coalesced_events = s_coalesced_events;
    portEXIT_CRITICAL(&s_isr_mux);
    bool ok = api_ok("read error", bmi2_get_regs(BMI270_ERROR_REG_ADDR, &out->error_reg, 1, &s_bmi)) &&
              api_ok("read internal status", bmi2_get_internal_status(&out->internal_status, &s_bmi)) &&
              api_ok("read power", bmi2_get_regs(BMI2_PWR_CTRL_ADDR, &out->power_ctrl, 1, &s_bmi)) &&
              api_ok("read INT1 config", bmi2_get_regs(BMI2_INT1_IO_CTRL_ADDR, &out->int1_ctrl, 1, &s_bmi)) &&
              api_ok("read INT mapping", bmi2_get_regs(BMI2_INT_MAP_DATA_ADDR, &out->int_map_data, 1, &s_bmi));
    /* INT_STATUS is useful evidence, but a failure here must not hide the
     * GPIO edge counters and the other diagnostics. */
    if (!api_ok("read interrupt status", bmi2_get_int_status(&out->int_status, &s_bmi))) {
        ESP_LOGW(TAG, "INT_STATUS read unavailable; keeping other diagnostics");
    }
    return ok;
}
