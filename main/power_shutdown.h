#ifndef POWER_SHUTDOWN_H_
#define POWER_SHUTDOWN_H_

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define POWER_SHUTDOWN_GPIO          9
#define POWER_SHUTDOWN_DEBOUNCE_MS   60
#define POWER_WORKER_ECG             BIT2
#define POWER_WORKER_BMI270          BIT3
#define POWER_WORKER_BATTERY         BIT4
#define POWER_WORKER_BLE_TX          BIT5

/* Integrated image only. Start BEFORE any peripheral initialization. GPIO9
 * has an internal pull-up; a debounced LOW latches the request until reset.
 * There is deliberately no clear-request, restart or wake-enable API. */
void power_shutdown_init(void);
bool power_shutdown_requested(void);
/* app_main is the sole writer; register only successfully created workers. */
void power_shutdown_worker_started(EventBits_t worker);
void power_shutdown_boot_complete(void);
/* Stop boot at a safe boundary if shutdown was requested during init. */
void power_shutdown_boot_checkpoint(void);
/* Workers call only BETWEEN transactions. Acknowledges then self-parks;
 * the coordinator never forcibly suspends a task holding a bus/stack lock. */
void power_shutdown_checkpoint(EventBits_t worker);
/* Interruptible task delay; does not synthesize any sensor notification. */
void power_shutdown_wait(uint32_t ms);

#endif
