#ifndef BMI270_DRIVER_H_
#define BMI270_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

/* Board wiring confirmed for the standalone USB validation image. */
#define BMI270_I2C_SDA_IO     12
#define BMI270_I2C_SCL_IO     14
#define BMI270_INT1_IO        10
#define BMI270_I2C_ADDR       0x68 /* Try 0x69 automatically if SDO is high. */
#define BMI270_I2C_FREQ_HZ    400000
#define BMI270_ACC_ODR_HZ    25
#define BMI270_GYR_ODR_HZ    25
#define BMI270_ACC_RANGE_G   2
#define BMI270_GYR_RANGE_DPS 2000
#define BMI270_DRDY_ACC      0x80
#define BMI270_DRDY_GYR      0x40

typedef struct {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    uint8_t status;             /* Raw STATUS; DRDY bits validate each sensor. */
    uint32_t sensor_time;       /* 24-bit hardware sensor counter, not USB time. */
} bmi270_data_t;

typedef struct {
    uint32_t int1_events;
    uint32_t coalesced_events;  /* ISR edges received while semaphore was full. */
    int int1_level;
    uint8_t address, chip_id, error_reg, status, internal_status;
    uint8_t power_ctrl, int1_ctrl, int_map_data;
    uint16_t int_status;         /* BMI270 INT_STATUS_0/1, ACC/GYR DRDY bits 15/14. */
    int8_t last_api_error;
    int32_t last_i2c_error;
} bmi270_diagnostics_t;

/* Use from one acquisition task; never perform I2C transactions in the ISR. */
void bmi270_driver_i2c_scan(void);
bool bmi270_driver_init(void);
void bmi270_driver_deinit(void);
/* Final shutdown with acquisition parked. Also usable before normal init;
 * disables sensor engines/interrupts, enables APS, verifies then deinitializes. */
bool bmi270_driver_power_down(void);
bool bmi270_driver_start_data_ready_int(void);
bool bmi270_driver_wait_data_ready(uint32_t timeout_ms);
bool bmi270_driver_read(bmi270_data_t *out);
bool bmi270_driver_read_temp(int16_t *temp_c100);
bool bmi270_driver_diagnostics(bmi270_diagnostics_t *out);

#endif
