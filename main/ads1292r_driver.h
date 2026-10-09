#ifndef ADS1292R_DRIVER_H_
#define ADS1292R_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>
#include "ads_control.h"

/* ADS1292/ADS1292R SPI command set. Conversions start with the SPI START
 * opcode; hardware START GPIO23 remains LOW for command control. */
#define ADS1292_CMD_WAKEUP    0x02
#define ADS1292_CMD_STANDBY   0x04
#define ADS1292_CMD_RESET     0x06
#define ADS1292_CMD_START     0x08
#define ADS1292_CMD_STOP      0x0A
#define ADS1292_CMD_RDATAC    0x10
#define ADS1292_CMD_SDATAC    0x11
#define ADS1292_CMD_RDATA     0x12
#define ADS1292_CMD_RREG      0x20
#define ADS1292_CMD_WREG      0x40
#define ADS1292_CMD_OFFSETCAL 0x1A

/* ADS1292/ADS1292R register map. */
#define ADS1292_REG_ID        0
#define ADS1292_REG_CONFIG1   1
#define ADS1292_REG_CONFIG2   2
#define ADS1292_REG_LOFF      3
#define ADS1292_REG_CH1SET    4
#define ADS1292_REG_CH2SET    5
#define ADS1292_REG_RLDSENS   6
#define ADS1292_REG_LOFFSENS  7
#define ADS1292_REG_LOFFSTAT  8
#define ADS1292_REG_RESP1     9
#define ADS1292_REG_RESP2     10
#define ADS1292_REG_GPIO      11

#define DEVICE_ID_ADS1292     0x53
#define DEVICE_ID_ADS1292R    0x73

/* ESP32-H2 board wiring used by this project. */
#define ADS1292R_CS_GPIO      1
#define ADS1292R_DRDY_GPIO    11
#define ADS1292R_RESET_GPIO   0
#define ADS1292R_START_GPIO   23    /* Held LOW for SPI START/STOP command control. */
#define ADS1292R_SPI_SCLK_GPIO 4
#define ADS1292R_SPI_MISO_GPIO 3
#define ADS1292R_SPI_MOSI_GPIO 5

/* 500 kHz gives multi-byte command decode time margin with the 512-kHz
 * ADS clock. RDATAC sample shifts keep their separate 1-MHz device. */
#define ADS1292R_SPI_REG_HZ   500000
#define ADS1292R_SPI_HZ       1000000

/* Legacy sentinel retained for source compatibility; init rejects it
 * because command mode still requires a physical START pin driven LOW. */
#define ADS1292R_PIN_UNUSED   (-1)

typedef struct {
    uint8_t raw[9];
    int32_t channelData[2];
    int32_t respirationData;
    bool leadOffDetected;
} ads1292r_data_t;

typedef struct {
    int cs_pin;
    int drdy_pin;
    int reset_pin;
    int start_pin;              /* Held LOW, including during SPI-started conversions. */
    bool last_drdy;
    spi_host_device_t host;
    spi_device_handle_t spi_reg;   /* Register/command transactions. */
    spi_device_handle_t spi_data;  /* Continuous data transactions. */
} ads1292r_dev_t;

/*
 * Initialize GPIO, SPI, hardware reset, software reset, ID check and register
 * configuration. The function returns false if SPI transactions fail or the
 * device ID is not 0x73 (ADS1292R) / 0x53 (ADS1292). The hardware START
 * pin is held LOW on return, with RDATAC enabled. The caller must install
 * the DRDY handler before calling ads1292r_begin_stream().
 */
bool ads1292r_init(ads1292r_dev_t *dev, int cs_pin, int drdy_pin, int reset_pin,
                   int start_pin, spi_host_device_t host);

/*
 * Low-level wiring diagnostic.  It is intended to be called after a failed
 * ads1292r_init(); it bit-bangs an SPI ID read and leaves the pins as GPIOs.
 */
void ads1292r_diag_pins(void);

/* After DRDY ISR is armed, hold hardware START LOW and send SDATAC,
 * START (0x08), then RDATAC. Returns false on GPIO/pad/transfer failure.
 * Successful SPI transmission is not proof of conversions; check DRDY/data. */
bool ads1292r_begin_stream(ads1292r_dev_t *dev);

/* Directly read one 9-byte raw frame after DRDY is already asserted. */
bool ads1292r_read_data(ads1292r_dev_t *dev, ads1292r_data_t *data);

/* Main acquisition path, matching the working ESP32 project. Read only
 * while DRDY is low and after a high level has re-armed the next sample. */
bool ads1292r_read_samples(ads1292r_dev_t *dev, ads1292r_data_t *data);

/* Runtime APIs reuse existing SPI handles; call only from the ECG task with
 * DRDY disabled. reset_only does not write ANY register or SPI command.
 * reconfigure leaves START LOW; re-arm DRDY then call begin_stream(). */
bool ads1292r_reset_only(ads1292r_dev_t *dev);
/* Final board shutdown only, with the SPI owner parked. Works before init.
 * Hold PWDN/RESET LOW indefinitely; caller retains pads in Deep-sleep. */
bool ads1292r_power_down(void);
bool ads1292r_reconfigure(ads1292r_dev_t *dev, const ads_control_config_t *config);
void ads1292r_reset(ads1292r_dev_t *dev);
uint8_t ads1292r_read_device_id(ads1292r_dev_t *dev);
void ads1292r_set_data_rate(ads1292r_dev_t *dev, uint8_t dr);
void ads1292r_wakeup(ads1292r_dev_t *dev);
void ads1292r_standby(ads1292r_dev_t *dev);
void ads1292r_start_conv(ads1292r_dev_t *dev);
/* Low-level SPI STOP helper; disable DRDY first. Leaves SDATAC enabled.
 * Remote BLE STOP instead calls reset_only(), with no SPI commands. */
void ads1292r_stop_conv(ads1292r_dev_t *dev);

#endif /* ADS1292R_DRIVER_H_ */


