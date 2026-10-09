#ifndef BLE_LINK_H_
#define BLE_LINK_H_

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "ads_control.h"

/* Start NimBLE as a connectable peripheral named "ECG-H2" and advertise.
 * GATT: one NOTIFY characteristic and one WRITE control characteristic. The
 * notification characteristic carries the
 * byte stream of packets built in main.c (A5 5A | type | len | payload | crc16).
 * A packet larger than (ATT_MTU - 3) is split over several notifications; the
 * PC side must reassemble by scanning for the A5 5A sync + CRC. */
typedef bool (*ble_control_receiver_t)(const ads_control_command_t *command);
/* Install before init; receiver only enqueues validated requests, never SPI. */
void ble_link_set_control_receiver(ble_control_receiver_t receiver);
esp_err_t ble_link_init(void);
/* Final shutdown only; never called from the NimBLE host or packet writer.
 * Latches radio-off, stops host then deinitializes host AND controller. */
esp_err_t ble_link_shutdown(void);

/* Queue one complete packet for notification. Returns false if no client is
 * subscribed or the stack could not accept it (the packet is then dropped). */
bool ble_link_send(const uint8_t *data, size_t len);

bool ble_link_is_streaming(void);
uint32_t ble_link_dropped(void);

#endif /* BLE_LINK_H_ */
