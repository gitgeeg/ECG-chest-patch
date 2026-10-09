#ifndef ADS_CONTROL_H_
#define ADS_CONTROL_H_
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ADS_CONTROL_VERSION 1
#define ADS_CONTROL_REG_COUNT 7
#define ADS_CONTROL_COMMAND_MAX 19
#define ADS_CONTROL_REPLY_SIZE 29
#define ADS_CONTROL_QUERY 1
#define ADS_CONTROL_START 2
#define ADS_CONTROL_STOP 3
#define ADS_CONTROL_SET 4
#define ADS_CONTROL_OK 0
#define ADS_CONTROL_STAGED 1
#define ADS_CONTROL_HW_ERROR 2
#define ADS_CONTROL_STOPPED 0
#define ADS_CONTROL_RUNNING 1
#define ADS_CONTROL_FAULT 2

typedef struct { uint8_t values[ADS_CONTROL_REG_COUNT]; } ads_control_config_t;
typedef struct {
    uint8_t opcode;
    uint16_t request_id;
    uint8_t count;
    uint8_t addresses[ADS_CONTROL_REG_COUNT];
    uint8_t values[ADS_CONTROL_REG_COUNT];
} ads_control_command_t;

extern const uint8_t ads_control_addresses[ADS_CONTROL_REG_COUNT];
void ads_control_defaults(ads_control_config_t *config);
int ads_control_reg_index(uint8_t address);
bool ads_control_valid_register(uint8_t address, uint8_t value);
bool ads_control_decode(const uint8_t *data, size_t len, ads_control_command_t *out);
bool ads_control_candidate(const ads_control_config_t *current,
                           const ads_control_command_t *command, ads_control_config_t *candidate);
size_t ads_control_reply(uint8_t *out, const ads_control_command_t *command,
                         uint8_t result, uint8_t state, uint32_t revision,
                         uint32_t epoch, const ads_control_config_t *config);
#endif
