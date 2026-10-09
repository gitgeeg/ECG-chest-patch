#include "ads1292r_driver.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <string.h>

static const char *TAG = "ads1292r";

/* Local datasheet 8.5.1.1: keep CS LOW for >=4 tCLK after SCLK.
 * Table 6.6 allows tCLK up to 2.170 us; 10 us covers 4 cycles.
 * Also leave this guard after CS HIGH before the next transaction. */
#define ADS1292R_SPI_GUARD_US 10

static bool ads1292r_configure_registers(ads1292r_dev_t *dev, const ads_control_config_t *config);
static bool ads1292r_write_register(ads1292r_dev_t *dev, uint8_t reg_addr, uint8_t value);
static bool ads1292r_read_register(ads1292r_dev_t *dev, uint8_t reg_addr, uint8_t *value);
static bool ads1292r_write_register_burst(ads1292r_dev_t *dev, uint8_t start_addr,
                                          uint8_t count, const uint8_t *data);
static bool ads1292r_read_register_burst(ads1292r_dev_t *dev, uint8_t start_addr,
                                         uint8_t count, uint8_t *data);
static bool ads1292r_send_command(ads1292r_dev_t *dev, uint8_t cmd);
static bool ads1292r_read_raw_data(ads1292r_dev_t *dev, uint8_t *buffer, uint8_t len);
static bool ads1292r_is_supported_device_id(uint8_t id);
static void ads1292r_cleanup_spi(ads1292r_dev_t *dev);

/* ADS1292R has no reason to use ESP-internal bias resistors on its digital
 * interface.  Keep every signal pad floating internally and rely on the ADS
 * outputs / board-level termination for the actual line levels. */
static bool ads1292r_disable_internal_pull(int pin, const char *signal)
{
    esp_err_t ret = gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "disable internal pull for %s GPIO%d failed: %s",
                 signal, pin, esp_err_to_name(ret));
        return false;
    }
    return true;
}

static bool ads1292r_spi_transaction(spi_device_handle_t handle, int cs_pin,
                                     const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
{
    if (handle == NULL || txbuf == NULL || rxbuf == NULL || len == 0) {
        return false;
    }

    /* Revert to the earlier H2 SPI transaction that produced real data frames.
     * Manual CS stays low for the complete command / register / 9-byte frame. */
    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = txbuf,
        .rx_buffer = rxbuf,
    };

    esp_err_t cs_low = gpio_set_level(cs_pin, 0);
    if (cs_low != ESP_OK) {
        ESP_LOGE(TAG, "assert CS failed: %s", esp_err_to_name(cs_low));
        return false;
    }
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    esp_err_t ret = spi_device_transmit(handle, &trans);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    esp_err_t cs_high = gpio_set_level(cs_pin, 1);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    if (ret != ESP_OK || cs_high != ESP_OK) {
        ESP_LOGE(TAG, "SPI (%u bytes) failed: transfer=%s CS=%s",
                 (unsigned)len, esp_err_to_name(ret), esp_err_to_name(cs_high));
        return false;
    }
    return true;
}

static bool ads1292r_is_supported_device_id(uint8_t id)
{
    return id == DEVICE_ID_ADS1292R || id == DEVICE_ID_ADS1292;
}

static void ads1292r_cleanup_spi(ads1292r_dev_t *dev)
{
    if (dev == NULL) {
        return;
    }

    if (dev->spi_data != NULL) {
        spi_bus_remove_device(dev->spi_data);
        dev->spi_data = NULL;
    }
    if (dev->spi_reg != NULL) {
        spi_bus_remove_device(dev->spi_reg);
        dev->spi_reg = NULL;
    }

    /* The bus is owned by this driver after a successful spi_bus_initialize(). */
    esp_err_t ret = spi_bus_free(dev->host);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus cleanup failed: %s", esp_err_to_name(ret));
    }

    if (GPIO_IS_VALID_GPIO(dev->cs_pin)) {
        gpio_set_level(dev->cs_pin, 1);
    }
    if (GPIO_IS_VALID_GPIO(dev->start_pin)) {
        gpio_set_level(dev->start_pin, 0);
    }
    if (GPIO_IS_VALID_GPIO(dev->reset_pin)) {
        gpio_set_level(dev->reset_pin, 1);
    }
}

bool ads1292r_init(ads1292r_dev_t *dev, int cs_pin, int drdy_pin, int reset_pin,
                   int start_pin, spi_host_device_t host)
{
    if (dev == NULL) {
        return false;
    }


    /* SPI command control still requires the physical START input to be
     * connected to GPIO23 and held LOW. Do not accept an unassigned pin. */
    if (!GPIO_IS_VALID_GPIO(cs_pin) || !GPIO_IS_VALID_GPIO(drdy_pin) ||
        !GPIO_IS_VALID_GPIO(reset_pin) || !GPIO_IS_VALID_GPIO(start_pin)) {
        ESP_LOGE(TAG, "invalid ADS1292R GPIO assignment for SPI command START: CS=%d DRDY=%d RESET=%d START=%d",
                 cs_pin, drdy_pin, reset_pin, start_pin);
        return false;
    }

    memset(dev, 0, sizeof(*dev));
    dev->cs_pin = cs_pin;
    dev->drdy_pin = drdy_pin;
    dev->reset_pin = reset_pin;
    dev->start_pin = start_pin;
    dev->last_drdy = true;
    dev->host = host;

    /* Enable input sensing on output pins too, so the ECG stall diagnostic
     * can read their actual pad levels with gpio_get_level(). START is kept
     * LOW during reset, register setup and SPI-started acquisition. */
    esp_err_t ret = gpio_reset_pin(cs_pin);
    if (ret == ESP_OK) {
        ret = gpio_set_direction(cs_pin, GPIO_MODE_INPUT_OUTPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_level(cs_pin, 1);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "configure CS GPIO%d failed: %s", cs_pin, esp_err_to_name(ret));
        return false;
    }

    ret = gpio_reset_pin(reset_pin);
    if (ret == ESP_OK) {
        ret = gpio_set_direction(reset_pin, GPIO_MODE_INPUT_OUTPUT);
    }
    if (ret == ESP_OK) {
        ret = gpio_set_level(reset_pin, 1);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "configure RESET GPIO%d failed: %s", reset_pin, esp_err_to_name(ret));
        return false;
    }

    if (GPIO_IS_VALID_GPIO(start_pin)) {
        ret = gpio_reset_pin(start_pin);
        if (ret == ESP_OK) {
            ret = gpio_set_direction(start_pin, GPIO_MODE_INPUT_OUTPUT);
        }
        if (ret == ESP_OK) {
            ret = gpio_set_level(start_pin, 0);
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "configure START GPIO%d failed: %s", start_pin, esp_err_to_name(ret));
            return false;
        }
        ESP_LOGI(TAG, "START GPIO%d configured LOW for SPI command control; acquisition task sends START after DRDY ISR is ready", start_pin);
    }

    ret = gpio_reset_pin(drdy_pin);
    if (ret == ESP_OK) {
        ret = gpio_set_direction(drdy_pin, GPIO_MODE_INPUT);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "configure DRDY GPIO%d failed: %s", drdy_pin, esp_err_to_name(ret));
        return false;
    }

    /* gpio_reset_pin() currently leaves a pad in its default floating state,
     * but do not depend on that implementation detail.  Explicitly disable
     * both internal pulls on every ADS and SPI signal.  The directions remain
     * role-specific: CS/RESET/START/SCLK/MOSI are driven outputs, while
     * DRDY/MISO are inputs. */
    if (!ads1292r_disable_internal_pull(cs_pin, "CS") ||
        !ads1292r_disable_internal_pull(reset_pin, "RESET") ||
        !ads1292r_disable_internal_pull(start_pin, "START") ||
        !ads1292r_disable_internal_pull(drdy_pin, "DRDY") ||
        !ads1292r_disable_internal_pull(ADS1292R_SPI_SCLK_GPIO, "SCLK") ||
        !ads1292r_disable_internal_pull(ADS1292R_SPI_MOSI_GPIO, "MOSI") ||
        !ads1292r_disable_internal_pull(ADS1292R_SPI_MISO_GPIO, "MISO")) {
        return false;
    }
    ESP_LOGI(TAG, "ADS GPIO internal pull-up/pull-down disabled on CS/DRDY/RESET/START/SCLK/MISO/MOSI");

    spi_bus_config_t buscfg = {
        .mosi_io_num = ADS1292R_SPI_MOSI_GPIO,
        .miso_io_num = ADS1292R_SPI_MISO_GPIO,
        .sclk_io_num = ADS1292R_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };

    /* ADS1292R uses SPI mode 1 (CPOL=0, CPHA=1).  CS is deliberately manual
     * so the required CS-high gaps between commands are explicit. */
    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    /* Transactions are only a few bytes and use task-local buffers; disable DMA so
     * the bring-up path does not depend on DMA-capable heap/stack buffers. */
    ret = spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return false;
    }

    devcfg.clock_speed_hz = ADS1292R_SPI_REG_HZ;
    ret = spi_bus_add_device(host, &devcfg, &dev->spi_reg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device(register) failed: %s", esp_err_to_name(ret));
        ads1292r_cleanup_spi(dev);
        return false;
    }

    devcfg.clock_speed_hz = ADS1292R_SPI_HZ;
    ret = spi_bus_add_device(host, &devcfg, &dev->spi_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device(data) failed: %s", esp_err_to_name(ret));
        ads1292r_cleanup_spi(dev);
        return false;
    }

    ESP_LOGI(TAG, "SPI ready: host=%d SCLK=%d MOSI=%d MISO=%d mode=1 command=%dHz data=%dHz CS_guard=%dus",
             (int)host, ADS1292R_SPI_SCLK_GPIO, ADS1292R_SPI_MOSI_GPIO,
             ADS1292R_SPI_MISO_GPIO, ADS1292R_SPI_REG_HZ, ADS1292R_SPI_HZ,
             ADS1292R_SPI_GUARD_US);

    /* Hardware reset.  The long delay is intentional: it also covers slow
     * analog power/reference startup on the ADS1292R board. */
    ESP_LOGI(TAG, "RESET GPIO%d pad before assert=%d (expect 1)", reset_pin, gpio_get_level(reset_pin));
    vTaskDelay(pdMS_TO_TICKS(10));
    if (gpio_set_level(reset_pin, 0) != ESP_OK) {
        ESP_LOGE(TAG, "assert RESET failed");
        ads1292r_cleanup_spi(dev);
        return false;
    }
    esp_rom_delay_us(50);
    int rst_low_1 = gpio_get_level(reset_pin);
    vTaskDelay(pdMS_TO_TICKS(1000));
    int rst_low_2 = gpio_get_level(reset_pin);
    if (gpio_set_level(reset_pin, 1) != ESP_OK) {
        ESP_LOGE(TAG, "release RESET failed");
        ads1292r_cleanup_spi(dev);
        return false;
    }
    esp_rom_delay_us(50);
    int rst_high_1 = gpio_get_level(reset_pin);
    vTaskDelay(pdMS_TO_TICKS(100));
    int rst_high_2 = gpio_get_level(reset_pin);
    ESP_LOGI(TAG, "RESET pad readback: low@50us=%d low@1s=%d (expect 0,0); high@50us=%d high@100ms=%d (expect 1,1)",
             rst_low_1, rst_low_2, rst_high_1, rst_high_2);
    if (rst_low_1 != 0 || rst_low_2 != 0 || rst_high_1 != 1 || rst_high_2 != 1) {
        ESP_LOGE(TAG, "RESET pad does not follow the driven level -> external circuit is holding/loading GPIO%d", reset_pin);
    }

    /* After either reset the device starts in RDATAC.  Stop continuous data
     * mode before issuing RREG/WREG commands. */
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!ads1292r_send_command(dev, ADS1292_CMD_RESET)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    /* Match the proven ESP32 reference: allow analog startup after SPI RESET. */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Software reset returns to RDATAC, so issue SDATAC once more. */
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* ID read is the communication gate: if this does not return a known
     * value, do not continue into normal operation. */
    uint8_t id = ads1292r_read_device_id(dev);
    if (!ads1292r_is_supported_device_id(id)) {
        ESP_LOGE(TAG, "ADS1292R ID check failed: read 0x%02X, expected 0x%02X (ADS1292R)",
                 id, DEVICE_ID_ADS1292R);
        ads1292r_cleanup_spi(dev);
        return false;
    }
    if (id == DEVICE_ID_ADS1292) {
        ESP_LOGW(TAG, "ADS1292-compatible device detected (ID=0x%02X); RESP registers may differ", id);
    } else {
        ESP_LOGI(TAG, "ADS1292R device ID verified: 0x%02X", id);
    }

    /* ads1292r_read_device_id() restores RDATAC; return to SDATAC for WREG. */
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!ads1292r_configure_registers(dev, NULL)) {
        ESP_LOGE(TAG, "ADS1292R register configuration transaction failed");
        ads1292r_cleanup_spi(dev);
        return false;
    }

    /* SPI command mode: keep hardware START LOW during setup and streaming.
     * Only the acquisition task sends START after its DRDY handler is armed. */
    if (GPIO_IS_VALID_GPIO(start_pin) && gpio_set_level(start_pin, 0) != ESP_OK) {
        ESP_LOGE(TAG, "hold START GPIO%d low failed", start_pin);
        ads1292r_cleanup_spi(dev);
        return false;
    }

    /* RESP2.CALIB_ON is enabled above. Run the ADC offset calibration on every
     * boot before conversion starts, then honor the required 150 ms settling
     * time before sending the next command. */
    ESP_LOGI(TAG, "starting ADS1292R offset calibration: OFFSETCAL (0x1A)");
    if (!ads1292r_send_command(dev, ADS1292_CMD_OFFSETCAL)) {
        ESP_LOGE(TAG, "ADS1292R OFFSETCAL command failed");
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "ADS1292R offset calibration complete; waited 150 ms");

    if (!ads1292r_send_command(dev, ADS1292_CMD_WAKEUP)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!ads1292r_send_command(dev, ADS1292_CMD_RDATAC)) {
        ads1292r_cleanup_spi(dev);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Do not start conversion here. The ECG task must first install its
     * DRDY handler; post-init register/ID checks also run while START is LOW. */
    ESP_LOGI(TAG, "ADS1292R prepared; RDATAC active, START GPIO%d=%d, DRDY GPIO%d=%d; waiting for acquisition task",
             start_pin, gpio_get_level(start_pin), drdy_pin, gpio_get_level(drdy_pin));
    return true;
}

bool ads1292r_begin_stream(ads1292r_dev_t *dev)
{
    if (!dev || !dev->spi_reg || !dev->spi_data || !GPIO_IS_VALID_GPIO(dev->start_pin)) {
        return false;
    }
    /* Local datasheet 8.5.2.4: command-controlled conversions require the
     * hardware START pin to remain LOW until STOP. Do not pulse it HIGH. */
    esp_err_t err = gpio_set_level(dev->start_pin, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hold START GPIO%d LOW failed: %s", dev->start_pin, esp_err_to_name(err));
        return false;
    }
    esp_rom_delay_us(50);
    int start_level = gpio_get_level(dev->start_pin);
    if (start_level != 0) {
        ESP_LOGE(TAG, "START GPIO%d pad=%d, expected LOW for SPI command control",
                 dev->start_pin, start_level);
        return false;
    }

    /* init/reconfigure leave RDATAC enabled. Exit it before START, then
     * restore continuous reads. send_command supplies the >=4 tCLK guards.
     * The caller has already enabled DRDY; no task delay follows START. */
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        ESP_LOGE(TAG, "ADS SDATAC (0x11) transfer failed before SPI START");
        return false;
    }
    if (!ads1292r_send_command(dev, ADS1292_CMD_START)) {
        ESP_LOGE(TAG, "ADS SPI START (0x08) transfer failed");
        return false;
    }
    if (!ads1292r_send_command(dev, ADS1292_CMD_RDATAC)) {
        ESP_LOGE(TAG, "ADS RDATAC (0x10) transfer failed after SPI START");
        return false;
    }
    dev->last_drdy = true;
    /* SPI completion is not an ADS acknowledgement. Actual conversions are
     * confirmed by DRDY edges and valid data frames in the acquisition task. */
    ESP_LOGI(TAG, "ADS SPI START (0x08) sent; RDATAC (0x10) sent; hardware START GPIO%d drive=LOW pad=%d; DRDY GPIO%d=%d RESET GPIO%d=%d",
             dev->start_pin, start_level, dev->drdy_pin, gpio_get_level(dev->drdy_pin),
             dev->reset_pin, gpio_get_level(dev->reset_pin));
    return true;
}

bool ads1292r_read_samples(ads1292r_dev_t *dev, ads1292r_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return false;
    }

    int drdy = gpio_get_level(dev->drdy_pin);
    if (drdy == 0) {
        if (dev->last_drdy) {
            bool ok = ads1292r_read_raw_data(dev, data->raw, sizeof(data->raw));
            if (ok) {
                /* The reference waits for DRDY to go high before arming the
                 * next read. On H2, check it immediately after SPI as well:
                 * USB output may otherwise hide that short high interval. */
                dev->last_drdy = gpio_get_level(dev->drdy_pin) != 0;
            }
            return ok;
        }
        return false;
    }

    dev->last_drdy = true;
    return false;
}

bool ads1292r_read_data(ads1292r_dev_t *dev, ads1292r_data_t *data)
{
    if (dev == NULL || data == NULL) {
        return false;
    }
    /* Caller must have observed DRDY before entering this function. */
    return ads1292r_read_raw_data(dev, data->raw, sizeof(data->raw));
}

bool ads1292r_reset_only(ads1292r_dev_t *dev)
{
    if (!dev || !GPIO_IS_VALID_GPIO(dev->start_pin) || !GPIO_IS_VALID_GPIO(dev->reset_pin)) return false;
    bool ok = gpio_set_level(dev->start_pin, 0) == ESP_OK;
    ok = gpio_set_level(dev->cs_pin, 1) == ESP_OK && ok;
    ok = gpio_set_level(dev->reset_pin, 0) == ESP_OK && ok;
    vTaskDelay(pdMS_TO_TICKS(1000));
    ok = gpio_set_level(dev->reset_pin, 1) == ESP_OK && ok;
    vTaskDelay(pdMS_TO_TICKS(100));
    dev->last_drdy = true;
    return ok;
}

bool ads1292r_reconfigure(ads1292r_dev_t *dev, const ads_control_config_t *config)
{
    if (!dev || !dev->spi_reg || !config) return false;
    for (int i = 0; i < ADS_CONTROL_REG_COUNT; ++i)
        if (!ads_control_valid_register(ads_control_addresses[i], config->values[i])) return false;
    if (!ads1292r_reset_only(dev)) return false;
    /* Reset defaults to RDATAC. Hardware START stays LOW throughout setup. */
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) return false;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t id = 0;
    if (!ads1292r_read_register(dev, ADS1292_REG_ID, &id) || !ads1292r_is_supported_device_id(id)) return false;
    if (!ads1292r_configure_registers(dev, config)) return false;
    /* Honor remotely selected CALIB_ON rather than forcing it back on. */
    int resp2 = ads_control_reg_index(ADS1292_REG_RESP2);
    if (config->values[resp2] & 0x80) {
        if (!ads1292r_send_command(dev, ADS1292_CMD_OFFSETCAL)) return false;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    if (!ads1292r_send_command(dev, ADS1292_CMD_WAKEUP)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!ads1292r_send_command(dev, ADS1292_CMD_RDATAC)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    /* The acquisition task re-arms DRDY before calling begin_stream(). */
    ESP_LOGI(TAG, "ADS reconfigured; START GPIO%d=%d DRDY GPIO%d=%d; waiting for acquisition task",
             dev->start_pin, gpio_get_level(dev->start_pin),
             dev->drdy_pin, gpio_get_level(dev->drdy_pin));
    return true;
}

void ads1292r_reset(ads1292r_dev_t *dev)
{
    if (dev == NULL) {
        return;
    }
    gpio_set_level(dev->reset_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(dev->reset_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

uint8_t ads1292r_read_device_id(ads1292r_dev_t *dev)
{
    if (dev == NULL || dev->spi_reg == NULL) {
        return 0;
    }

    uint8_t id = 0;
    bool ok = ads1292r_send_command(dev, ADS1292_CMD_SDATAC);
    if (ok) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ok = ads1292r_read_register(dev, ADS1292_REG_ID, &id);
    }

    /* Always try to restore the streaming mode for callers that use this API
     * during normal operation, even when the register read failed. */
    bool restore_ok = ads1292r_send_command(dev, ADS1292_CMD_RDATAC);
    if (!ok || !restore_ok) {
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    return id;
}

void ads1292r_set_data_rate(ads1292r_dev_t *dev, uint8_t dr)
{
    if (dev == NULL) {
        return;
    }
    dr &= 0x07;
    if (ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ads1292r_write_register(dev, ADS1292_REG_CONFIG1, dr);
        vTaskDelay(pdMS_TO_TICKS(10));
        ads1292r_send_command(dev, ADS1292_CMD_RDATAC);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ads1292r_wakeup(ads1292r_dev_t *dev)
{
    if (dev != NULL && ads1292r_send_command(dev, ADS1292_CMD_WAKEUP)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ads1292r_standby(ads1292r_dev_t *dev)
{
    if (dev != NULL && ads1292r_send_command(dev, ADS1292_CMD_STANDBY)) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ads1292r_start_conv(ads1292r_dev_t *dev)
{
    (void)ads1292r_begin_stream(dev);
}

void ads1292r_stop_conv(ads1292r_dev_t *dev)
{
    if (!dev || !dev->spi_reg || !GPIO_IS_VALID_GPIO(dev->start_pin)) return;
    /* Legacy low-level helper only. The BLE STOP path still uses hardware
     * reset_only(), with no SPI command or register write. The caller must
     * disable DRDY before using this helper. Leave the interface in SDATAC. */
    esp_err_t err = gpio_set_level(dev->start_pin, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hold START LOW before SPI STOP failed: %s", esp_err_to_name(err));
        return;
    }
    esp_rom_delay_us(50);
    if (gpio_get_level(dev->start_pin) != 0) {
        ESP_LOGE(TAG, "START pad is not LOW; SPI STOP command control is not available");
        return;
    }
    if (!ads1292r_send_command(dev, ADS1292_CMD_SDATAC)) {
        ESP_LOGE(TAG, "ADS SDATAC transfer failed before SPI STOP");
        return;
    }
    if (!ads1292r_send_command(dev, ADS1292_CMD_STOP)) {
        ESP_LOGE(TAG, "ADS SPI STOP (0x0A) transfer failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    dev->last_drdy = true;
}

/* Register names used by the startup read-back log. */
static const char *const s_reg_names[12] = {
    "ID", "CONFIG1", "CONFIG2", "LOFF", "CH1SET", "CH2SET",
    "RLDSENS", "LOFFSENS", "LOFFSTAT", "RESP1", "RESP2", "GPIO",
};

static bool ads1292r_configure_registers(ads1292r_dev_t *dev, const ads_control_config_t *config)
{
    /* CONFIG1 through GPIO, 11 bytes.  LOFF_STAT bits[4:0] are read-only;
     * its byte is retained in the burst only for address alignment. */
    uint8_t regs[11] = {
        0b00000010,   /* CONFIG1: continuous, DR=010 -> 500 SPS */
        0b11100000,   /* CONFIG2: lead-off comparator + reference buffer on */
        0b11111001,   /* LOFF: COMP_TH=111, ILEAD_OFF=10, AC lead-off */
        0b01000000,   /* CH1SET: gain 4, normal electrode input */
        0b00000000,   /* CH2SET: gain 6, normal electrode input */
        0b10101100,   /* RLD_SENS: CHOP=10, RLD1N/RLD1P=0 */
        0b00000000,   /* LOFF_SENS: lead-off sensing disabled by default */
        0b00000000,   /* LOFF_STAT: bits[4:0] read-only; alignment byte */
        0b11110010,   /* RESP1: respiration modulation/demodulation enabled */
        0b10000011,   /* RESP2 (0x0A): CALIB_ON=1, bits[6:3]=0, RESP_FREQ=0, RLDREF_INT=1, bit0=1 */
        0b00000000,   /* GPIO1/2: output data low */
    };

    if (config != NULL) {
        for (int i = 0; i < ADS_CONTROL_REG_COUNT; ++i) {
            uint8_t address = ads_control_addresses[i];
            if (!ads_control_valid_register(address, config->values[i])) return false;
            regs[address - 1] = config->values[i];
        }
    }

    if (!ads1292r_write_register_burst(dev, ADS1292_REG_CONFIG1, 11, regs)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t verify[12] = {0};
    if (!ads1292r_read_register_burst(dev, ADS1292_REG_ID, 12, verify)) {
        return false;
    }

    bool all_match = true;
    ESP_LOGI(TAG, "ADS1292R register read-back:");
    for (int i = 0; i < 11; ++i) {
        uint8_t got = verify[i + 1];
        /* LOFF_STAT bits[4:0] are read-only/runtime status; GPIO read-back
         * can reflect the actual pad state. Log both without failing compare. */
        bool skip_compare = (i == 7 || i == 10);
        bool match = skip_compare || (regs[i] == got);
        if (!match) {
            all_match = false;
        }
        ESP_LOGI(TAG, "  %-8s 0x%02X = 0x%02X  %s",
                 s_reg_names[i + 1], i + 1, got,
                 skip_compare ? "runtime" : (match ? "OK" : "MISMATCH"));
    }

    if (all_match) {
        ESP_LOGI(TAG, "ADS1292R register configuration verified");
    } else {
        ESP_LOGE(TAG, "register read-back mismatch; configuration is NOT accepted");
    }
    return all_match;
}

static bool ads1292r_write_register(ads1292r_dev_t *dev, uint8_t reg_addr, uint8_t value)
{
    uint8_t tx[3] = { (uint8_t)(ADS1292_CMD_WREG | (reg_addr & 0x1F)), 0x00, value };
    uint8_t rx[3] = {0};

    bool ok = ads1292r_spi_transaction(dev->spi_reg, dev->cs_pin, tx, rx, sizeof(tx));
    return ok;
}

static bool ads1292r_read_register(ads1292r_dev_t *dev, uint8_t reg_addr, uint8_t *value)
{
    if (value == NULL) {
        return false;
    }

    uint8_t tx[3] = { (uint8_t)(ADS1292_CMD_RREG | (reg_addr & 0x1F)), 0x00, 0xFF };
    uint8_t rx[3] = {0};

    bool ok = ads1292r_spi_transaction(dev->spi_reg, dev->cs_pin, tx, rx, sizeof(tx));
    if (ok) {
        *value = rx[2];
    }
    return ok;
}

static bool ads1292r_write_register_burst(ads1292r_dev_t *dev, uint8_t start_addr,
                                          uint8_t count, const uint8_t *data)
{
    if (dev == NULL || data == NULL || count == 0 || count > 11) {
        return false;
    }

    uint8_t tx[2 + 11] = {0};
    uint8_t rx[2 + 11] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_WREG | (start_addr & 0x1F));
    tx[1] = (uint8_t)(count - 1);
    memcpy(tx + 2, data, count);

    bool ok = ads1292r_spi_transaction(dev->spi_reg, dev->cs_pin, tx, rx, 2 + count);
    return ok;
}

static bool ads1292r_read_register_burst(ads1292r_dev_t *dev, uint8_t start_addr,
                                         uint8_t count, uint8_t *data)
{
    if (dev == NULL || data == NULL || count == 0 || count > 12) {
        return false;
    }

    uint8_t tx[2 + 12] = {0};
    uint8_t rx[2 + 12] = {0};
    tx[0] = (uint8_t)(ADS1292_CMD_RREG | (start_addr & 0x1F));
    tx[1] = (uint8_t)(count - 1);
    memset(tx + 2, 0xFF, count);

    bool ok = ads1292r_spi_transaction(dev->spi_reg, dev->cs_pin, tx, rx, 2 + count);
    if (ok) {
        memcpy(data, rx + 2, count);
    }
    return ok;
}

static bool ads1292r_send_command(ads1292r_dev_t *dev, uint8_t cmd)
{
    if (dev == NULL || dev->spi_reg == NULL) {
        return false;
    }

    uint8_t tx[1] = { cmd };
    uint8_t rx[1] = {0};
    bool ok = ads1292r_spi_transaction(dev->spi_reg, dev->cs_pin, tx, rx, sizeof(tx));
    return ok;
}

static bool ads1292r_read_raw_data(ads1292r_dev_t *dev, uint8_t *buffer, uint8_t len)
{
    if (dev == NULL || buffer == NULL || len != 9 || dev->spi_data == NULL) {
        return false;
    }

    /* Datasheet section 8.5.1.5: DIN must stay LOW throughout the 72-bit
     * RDATAC data read. These are dummy clocks, not command bytes. */
    uint8_t tx[9] = {0};
    uint8_t rx[9] = {0};

    bool ok = ads1292r_spi_transaction(dev->spi_data, dev->cs_pin, tx, rx, sizeof(rx));
    if (ok) {
        memcpy(buffer, rx, sizeof(rx));
    }
    return ok;
}

/* Optional helper retained for future firmware-side signal processing. */
__attribute__((unused))
static int32_t ads1292r_convert24bit_signed(const uint8_t *data)
{
    int32_t raw = ((int32_t)data[0] << 16) |
                  ((int32_t)data[1] << 8) |
                  ((int32_t)data[2]);
    if (raw & 0x800000) {
        raw |= ~0xFFFFFF;
    }
    return raw;
}

/* ═══════════════════════════════════════════════════════════════════════
 * GPIO bit-bang diagnostic
 *
 * This is intentionally independent of the ESP-IDF SPI driver.  It is useful
 * after init fails: a valid ID here points to an SPI-driver/configuration
 * problem; all-zero MISO points to power, CS, DOUT or chip-reset wiring.
 * SPI mode 1: SCLK idle low, sample during the high phase.
 * ═══════════════════════════════════════════════════════════════════════ */
static uint8_t ads1292r_diag_bitbang_xfer(uint8_t txbyte, int sclk, int mosi,
                                          int miso, int *high_count)
{
    uint8_t rx = 0;
    for (int i = 7; i >= 0; --i) {
        gpio_set_level(mosi, (txbyte >> i) & 1);
        esp_rom_delay_us(2);
        gpio_set_level(sclk, 1);
        esp_rom_delay_us(2);
        int level = gpio_get_level(miso);
        if (high_count != NULL && level != 0) {
            (*high_count)++;
        }
        if (level != 0) {
            rx |= (uint8_t)(1u << i);
        }
        gpio_set_level(sclk, 0);
        esp_rom_delay_us(2);
    }
    return rx;
}

bool ads1292r_power_down(void)
{
    /* Board's shared PWDN/RESET goes LOW permanently. Holding it beyond
     * 2^9 fMOD cycles powers down ADS, unlike STANDBY or reset-only STOP. */
    esp_err_t err_reset = gpio_set_level(ADS1292R_RESET_GPIO, 0);
    esp_err_t err_start = gpio_set_level(ADS1292R_START_GPIO, 0);
    esp_err_t err_cs = gpio_set_level(ADS1292R_CS_GPIO, 1);
    const gpio_config_t outputs = {
        .pin_bit_mask = BIT64(ADS1292R_RESET_GPIO) | BIT64(ADS1292R_START_GPIO) |
                        BIT64(ADS1292R_CS_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&outputs);
    vTaskDelay(pdMS_TO_TICKS(20));
    return err == ESP_OK && err_reset == ESP_OK && err_start == ESP_OK && err_cs == ESP_OK &&
           gpio_get_level(ADS1292R_RESET_GPIO) == 0 &&
           gpio_get_level(ADS1292R_START_GPIO) == 0;
}

void ads1292r_diag_pins(void)
{
    const int cs = ADS1292R_CS_GPIO;
    const int drdy = ADS1292R_DRDY_GPIO;
    const int reset = ADS1292R_RESET_GPIO;
    const int sclk = ADS1292R_SPI_SCLK_GPIO;
    const int miso = ADS1292R_SPI_MISO_GPIO;
    const int mosi = ADS1292R_SPI_MOSI_GPIO;

    ESP_LOGI(TAG, "===== ADS1292R GPIO bit-bang diagnostic =====");

    gpio_config_t out_cfg = {
        .pin_bit_mask = BIT64(cs) | BIT64(reset) | BIT64(sclk) | BIT64(mosi),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    gpio_config_t in_cfg = {
        .pin_bit_mask = BIT64(drdy) | BIT64(miso),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in_cfg);

    gpio_set_level(cs, 1);
    gpio_set_level(reset, 1);
    gpio_set_level(sclk, 0);
    gpio_set_level(mosi, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    gpio_set_pull_mode(miso, GPIO_PULLUP_ONLY);
    esp_rom_delay_us(10);
    int miso_pullup = gpio_get_level(miso);
    gpio_set_pull_mode(miso, GPIO_FLOATING);
    esp_rom_delay_us(10);
    int miso_floating = gpio_get_level(miso);
    ESP_LOGI(TAG, "[1] MISO pull-up=%d floating=%d%s",
             miso_pullup, miso_floating,
             (miso_pullup != miso_floating)
                 ? " -> line appears floating/not driven"
                 : " -> line is driven or externally biased");

    int drdy_level = gpio_get_level(drdy);
    ESP_LOGI(TAG, "[2] DRDY initial level=%d", drdy_level);

    gpio_set_level(reset, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(reset, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "[3] DRDY after reset=%d", gpio_get_level(drdy));

    gpio_set_level(cs, 1);
    esp_rom_delay_us(50);
    int miso_cs_high = gpio_get_level(miso);
    gpio_set_level(cs, 0);
    esp_rom_delay_us(50);
    int miso_cs_low = gpio_get_level(miso);
    gpio_set_level(cs, 1);
    ESP_LOGI(TAG, "[A] CS high -> MISO=%d, CS low -> MISO=%d%s",
             miso_cs_high, miso_cs_low,
             (miso_cs_high != miso_cs_low)
                 ? " -> CS/DOUT changes"
                 : " -> no CS/DOUT change observed");

    int drdy_before = gpio_get_level(drdy);
    gpio_set_level(cs, 0);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    ads1292r_diag_bitbang_xfer(ADS1292_CMD_SDATAC, sclk, mosi, miso, NULL);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    gpio_set_level(cs, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    int drdy_after = gpio_get_level(drdy);
    ESP_LOGI(TAG, "[B] SDATAC DRDY before=%d after=%d%s",
             drdy_before, drdy_after,
             (drdy_before != drdy_after)
                 ? " -> command path appears active"
                 : " -> command path not confirmed");

    int high_count = 0;
    gpio_set_level(cs, 0);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    ads1292r_diag_bitbang_xfer((uint8_t)(ADS1292_CMD_RREG | ADS1292_REG_ID),
                               sclk, mosi, miso, NULL);
    ads1292r_diag_bitbang_xfer(0x00, sclk, mosi, miso, NULL);
    uint8_t id = ads1292r_diag_bitbang_xfer(0xFF, sclk, mosi, miso, &high_count);
    esp_rom_delay_us(ADS1292R_SPI_GUARD_US);
    gpio_set_level(cs, 1);

    ESP_LOGI(TAG, "[4] bit-bang ID=0x%02X, MISO high samples=%d/24", id, high_count);
    if (id == DEVICE_ID_ADS1292R || id == DEVICE_ID_ADS1292) {
        ESP_LOGI(TAG, "     -> chip responded with a supported ID");
    } else if (high_count > 0) {
        ESP_LOGW(TAG, "     -> MISO has activity, but ID is invalid; check SPI phase/timing");
    } else {
        ESP_LOGE(TAG, "     -> MISO stayed low; check ADS power, DOUT(GPIO%d), CS(GPIO%d)",
                 miso, cs);
    }

    ESP_LOGI(TAG, "[5] CS=%d SCLK=%d MOSI=%d MISO=%d DRDY=%d",
             gpio_get_level(cs), gpio_get_level(sclk), gpio_get_level(mosi),
             gpio_get_level(miso), gpio_get_level(drdy));
}





