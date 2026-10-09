#include "ads_control.h"
#include <string.h>

/* Wire order: LOFF, CH1SET, CH2SET, RLD_SENS, LOFF_SENS, RESP1, RESP2. */
const uint8_t ads_control_addresses[ADS_CONTROL_REG_COUNT] = {3, 4, 5, 6, 7, 9, 10};
static const uint8_t defaults[ADS_CONTROL_REG_COUNT] = {0xF9, 0x40, 0x00, 0xAC, 0x00, 0xF2, 0x83};

void ads_control_defaults(ads_control_config_t *config) { memcpy(config->values, defaults, sizeof(defaults)); }
int ads_control_reg_index(uint8_t address)
{
    for (int i = 0; i < ADS_CONTROL_REG_COUNT; ++i) if (ads_control_addresses[i] == address) return i;
    return -1;
}

bool ads_control_valid_register(uint8_t address, uint8_t value)
{
    switch (address) {
    case 3: return (value & 0x12) == 0x10; /* LOFF fixed bits 4=1, 1=0. */
    case 4: case 5:
        return ((value >> 4) & 7) != 7 && (value & 15) <= 9 &&
               (!(value & 0x80) || (value & 15) == 1);
    case 6: return (value >> 6) != 1; /* RLD_SENS CHOP=01 is reserved. */
    case 7: return (value & 0xC0) == 0;
    case 9: return (value & 2) != 0;
    case 10: return (value & 0x79) == 1;
    default: return false;
    }
}

bool ads_control_decode(const uint8_t *data, size_t len, ads_control_command_t *out)
{
    if (!data || !out || len < 5 || len > ADS_CONTROL_COMMAND_MAX || data[0] != ADS_CONTROL_VERSION) return false;
    uint8_t opcode = data[1], count = data[4];
    if (opcode < ADS_CONTROL_QUERY || opcode > ADS_CONTROL_SET || count > ADS_CONTROL_REG_COUNT ||
        len != (size_t)(5 + 2 * count) ||
        (opcode == ADS_CONTROL_SET ? count == 0 : count != 0)) return false;
    memset(out, 0, sizeof(*out));
    out->opcode = opcode;
    out->request_id = data[2] | ((uint16_t)data[3] << 8);
    out->count = count;
    uint16_t seen = 0;
    for (int i = 0; i < count; ++i) {
        uint8_t address = data[5 + 2 * i], value = data[6 + 2 * i];
        int index = ads_control_reg_index(address);
        if (index < 0 || (seen & (1U << index)) || !ads_control_valid_register(address, value)) return false;
        seen |= 1U << index;
        out->addresses[i] = address;
        out->values[i] = value;
    }
    return true;
}

bool ads_control_candidate(const ads_control_config_t *current,
                           const ads_control_command_t *command, ads_control_config_t *candidate)
{
    if (!current || !command || !candidate || command->count > ADS_CONTROL_REG_COUNT ||
        command->opcode < ADS_CONTROL_QUERY || command->opcode > ADS_CONTROL_SET ||
        (command->opcode == ADS_CONTROL_SET ? command->count == 0 : command->count != 0)) return false;
    *candidate = *current;
    for (int i = 0; i < command->count; ++i) {
        int index = ads_control_reg_index(command->addresses[i]);
        if (index < 0 || !ads_control_valid_register(command->addresses[i], command->values[i])) return false;
        candidate->values[index] = command->values[i];
    }
    return true;
}

size_t ads_control_reply(uint8_t *out, const ads_control_command_t *command,
                         uint8_t result, uint8_t state, uint32_t revision,
                         uint32_t epoch, const ads_control_config_t *config)
{
    out[0] = ADS_CONTROL_VERSION; out[1] = command->opcode;
    out[2] = (uint8_t)command->request_id; out[3] = (uint8_t)(command->request_id >> 8);
    out[4] = result; out[5] = state;
    for (int i = 0; i < 4; ++i) { out[6+i] = (uint8_t)(revision >> (8*i)); out[10+i] = (uint8_t)(epoch >> (8*i)); }
    out[14] = ADS_CONTROL_REG_COUNT;
    for (int i = 0; i < ADS_CONTROL_REG_COUNT; ++i) {
        out[15+2*i] = ads_control_addresses[i]; out[16+2*i] = config->values[i];
    }
    return ADS_CONTROL_REPLY_SIZE;
}
