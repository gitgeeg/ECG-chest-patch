/*
 * Integrated ADS1292R ECG/respiration + BMI270 acquisition over BLE for ESP32-H2.
 *
 * Pipeline:
 *   DRDY falling edge (ISR) -> ecg_task reads 9-byte frame over SPI
 *   -> CH1 (respiration impedance) boxcar-decimated 500 -> 50 SPS (x10)
 *   -> CH2 (ECG) kept at 500 SPS
 *   -> lead-off / saturation / drop statistics aggregated locally
 *   -> binary packets (see PACKET FORMAT) sent to sinks.
 *
 * PACKET FORMAT (little endian):
 *   A5 5A | type u8 | len u8 | payload[len] | crc16 u16
 *   crc16 = CRC-16/CCITT-FALSE over type,len,payload.
 *
 *   type 0x01 DATA   payload = seq u32 (first block index)
 *                              ts_us u32 (esp_timer at last sample, wraps)
 *                              flags u8 (bit0 lead-off seen, bit1 samples filled after drop)
 *                              nblk u8
 *                              nblk * block
 *                    block   = 10 * CH2 int24 (500 SPS) + 1 * CH1 int24 (avg of the 10, 50 SPS)
 *                              -> 33 bytes = 20 ms of data
 *   type 0x04 DATA_V2 = same as DATA, flags also bit2=CH1 lead-off,
 *                      bit3=RLD lead-off; insert config_revision u32 and
 *                      acquisition_epoch u32 after nblk (header 18 bytes).
 *                      Firmware now emits DATA_V2; raw blocks are unchanged.
 *   type 0x02 STATUS payload = sample_idx u32, ts_us u32, loff_or u8, loff_now u8,
 *                              sat u16, dropped_total u32, bad_frames u32,
 *                              ch1_min i32, ch1_max i32, ch2_min i32, ch2_max i32,
 *                              batt_mv u16 (0xFFFF = n/a), reason u8 (0 periodic/battery, 1 lead-off change, 2 DRDY stall)
 *
 *   type 0x05 IMU = sequence u32, device_ts_us u32, sensor_time u32, status u8, flags u8,
 *                 accel x/y/z i16, gyro x/y/z i16, latest temperature centi-C i16.
 *                 flags: bit0 IRQ source, bit1 temperature valid, bit2 accel DRDY, bit3 gyro DRDY.
 *
 *   type 0x03 CONTROL reply = version u8, opcode u8, request_id u16,
 *                 result u8 (0 applied, 1 staged, 2 hardware failure), state u8,
 *                 config_revision u32, acquisition_epoch u32, count u8,
 *                 7 * (register address u8, value u8); payload 29 bytes.
 * CONTROL WRITE ...0003 = version u8, opcode u8, request_id u16, count u8,
 *                 count * (address,value); <=19 bytes, ATT Write with response.
 *                 opcodes: 1 QUERY, 2 START, 3 STOP, 4 SET_REGISTERS.
 *
 * Default sink: the existing ECG-H2 BLE notification stream carries all packet types.
 * Optional USB debug mirror: one text line per packet, "P,<HEX>\r\n"; disabled
 * by default. Native USB Serial/JTAG remains available for flashing and logs.
 */

#include <inttypes.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "ads1292r_driver.h"
#include "ble_link.h"
#include "battery_adc.h"
#include "bmi270_driver.h"
#include "power_shutdown.h"

/* USB text output is debug only; the normal data path is BLE. */
#ifndef USB_DEBUG_OUTPUT
#define USB_DEBUG_OUTPUT 0
#endif

static const char *TAG = "main";
static ads1292r_dev_t s_ads1292r;
static TaskHandle_t s_ecg_task_handle;
/* One ISR writer, one task reader on the single-core H2. Never reset this
 * counter during reconfiguration; each sensor has its own GPIO handler. */
static volatile uint32_t s_drdy_irq_total;
static bool s_drdy_isr_ready;
static bool set_drdy_interrupt_enabled(bool enabled);
static bool start_ads_acquisition(void);

#define ADS_DRDY_TIMEOUT_MS 200
#define ADS_DRDY_LOG_MS     5000
static struct {
    uint32_t spi_reads_total, spi_errors_total, invalid_total, valid_total;
    uint32_t poll_total, timeouts_total, last_status;
    uint32_t log_irq, log_valid;
    int64_t log_us;
    bool stall_warned;
} s_drdy_diag;

#define RUN_STATUS_GPIO      GPIO_NUM_13
#define RUN_STATUS_ENABLE    1          /* set 0 for production / low power */

static QueueHandle_t s_control_queue;
static ads_control_config_t s_control_config;
static uint8_t s_control_state = ADS_CONTROL_STOPPED;
static uint32_t s_config_revision, s_acquisition_epoch;
static int64_t s_last_control_report_us;

#define ECG_TASK_STACK_SIZE  5120
/* Above the NimBLE host task (21) so a DRDY notification is serviced within
 * the 2 ms window; the ADS only keeps the latest frame in RDATAC mode. The
 * task blocks on the notification and each read is short, so it will not
 * starve the host. BLE controller ISR work still preempts it. */
#define ECG_TASK_PRIORITY    22

#define ADS_SPS              500
#define RESP_DECIM           10         /* CH1: 500 -> 50 SPS */
#define BLOCKS_PER_PACKET    5          /* 100 ms per DATA packet */
#define BATTERY_REPORT_MS    5000
/* The ADC worker only updates a mailbox; ecg_task is the sole packet writer. */
static portMUX_TYPE s_battery_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_battery_mv = 0xFFFF;
static uint32_t s_battery_generation;
static uint32_t s_battery_reported;
static int64_t s_battery_last_report_us;
#define LOFF_EVENT_MIN_GAP   100        /* samples between event packets */
#define MAX_FILL_SAMPLES     50

#define PKT_SYNC0            0xA5
#define PKT_SYNC1            0x5A
#define PKT_TYPE_DATA        0x04 /* v2 DATA: legacy 10-byte header + revision,epoch u32 */
#define PKT_TYPE_STATUS      0x02
#define PKT_TYPE_CONTROL     0x03
#define PKT_TYPE_IMU         0x05 /* BMI270: raw accel/gyro + latest temperature. */
#define BLOCK_BYTES          (RESP_DECIM * 3 + 3)
#define DATA_PAYLOAD_MAX     (18 + BLOCKS_PER_PACKET * BLOCK_BYTES)
#define IMU_PAYLOAD_BYTES    28
#define PKT_MAX              (4 + DATA_PAYLOAD_MAX + 2)

/* STATUS word: 1100 + LOFF_STAT[4:0] + GPIO[1:0] + 0...; bits 19..15 = LOFF_STAT. */
#define LOFF_MASK_CH2        0x0C       /* IN2N_OFF | IN2P_OFF (only CH2 electrodes are sensed) */

static const char s_hex[] = "0123456789ABCDEF";

/* ------------------------------------------------------------------ */
/* Utilities                                                           */
/* ------------------------------------------------------------------ */

/* CRC-16/CCITT-FALSE, table driven (table built once at startup). */
static uint16_t s_crc_table[256];

static void crc16_init_table(void)
{
    for (int b = 0; b < 256; ++b) {
        uint16_t crc = (uint16_t)(b << 8);
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
        s_crc_table[b] = crc;
    }
}

static uint16_t crc16_ccitt(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc = (uint16_t)((crc << 8) ^ s_crc_table[(uint8_t)((crc >> 8) ^ *d++)]);
    }
    return crc;
}

static inline int32_t sign24(const uint8_t *p)
{
    int32_t v = ((int32_t)p[0] << 16) | ((int32_t)p[1] << 8) | p[2];
    return (v & 0x800000) ? (v | ~0xFFFFFF) : v;
}

static inline void put24(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(v >> 16);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)v;
}

static inline uint8_t *put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return p + 2; }
static inline uint8_t *put_u32(uint8_t *p, uint32_t v) { p = put_u16(p, (uint16_t)v); return put_u16(p, (uint16_t)(v >> 16)); }
static inline uint8_t *put_i16(uint8_t *p, int16_t v) { return put_u16(p, (uint16_t)v); }
static inline uint8_t *put_i32(uint8_t *p, int32_t v) { return put_u32(p, (uint32_t)v); }

/* ------------------------------------------------------------------ */
/* Sinks                                                               */
/* ------------------------------------------------------------------ */

static void sink_usb(const uint8_t *pkt, size_t len)
{
    char line[PKT_MAX * 2 + 8];
    size_t pos = 0;
    line[pos++] = 'P';
    line[pos++] = ',';
    for (size_t i = 0; i < len; ++i) {
        line[pos++] = s_hex[pkt[i] >> 4];
        line[pos++] = s_hex[pkt[i] & 0x0F];
    }
    line[pos++] = '\r';
    line[pos++] = '\n';
    if (fwrite(line, 1, pos, stdout) != pos) {
        clearerr(stdout);
    }
}

static uint8_t s_pkt[PKT_MAX];

/* BLE transmit is decoupled from acquisition: ecg_task only enqueues finished
 * packets; ble_tx_task (low priority) hands them to the NimBLE host. */
#define TX_QUEUE_DEPTH       24
#define BLE_TX_TASK_PRIORITY 4
#define BLE_TX_STACK_SIZE    3072

typedef struct {
    uint16_t len;
    uint8_t  data[PKT_MAX];
} tx_item_t;

static QueueHandle_t s_tx_queue;
static volatile uint32_t s_tx_queue_full;

static void ble_tx_task(void *arg)
{
    (void)arg;
    static tx_item_t item;
    while (true) {
        power_shutdown_checkpoint(POWER_WORKER_BLE_TX);
        if (xQueueReceive(s_tx_queue, &item, pdMS_TO_TICKS(50)) == pdTRUE) {
            power_shutdown_checkpoint(POWER_WORKER_BLE_TX);
            (void)ble_link_send(item.data, item.len);
        }
    }
}

/* Dispatch one complete framed packet.  This is deliberately independent
 * of the ECG staging buffer so the BMI270 task can enqueue IMU packets while
 * the ECG task is assembling its next DATA/STATUS packet. */
static void dispatch_packet(const uint8_t *packet, size_t total, TickType_t wait)
{
    if (power_shutdown_requested()) return;
    if (s_tx_queue != NULL && ble_link_is_streaming()) {
        tx_item_t item;
        if (total > sizeof(item.data)) {
            ESP_LOGE(TAG, "packet too large: %u", (unsigned)total);
            return;
        }
        item.len = (uint16_t)total;
        memcpy(item.data, packet, total);
        if (xQueueSend(s_tx_queue, &item, wait) != pdTRUE) {
            (void)__atomic_fetch_add(&s_tx_queue_full, 1, __ATOMIC_RELAXED);
        }
    }

#if USB_DEBUG_OUTPUT
    /* USB text output is debug-only and disabled by default. */
    sink_usb(packet, total);
#else
    (void)sink_usb;
#endif
}

static void emit_packet_bytes(uint8_t type, const uint8_t *payload,
                              size_t payload_len, TickType_t wait)
{
    uint8_t packet[PKT_MAX];
    if (payload_len > PKT_MAX - 6) {
        ESP_LOGE(TAG, "payload too large: %u", (unsigned)payload_len);
        return;
    }
    packet[0] = PKT_SYNC0;
    packet[1] = PKT_SYNC1;
    packet[2] = type;
    packet[3] = (uint8_t)payload_len;
    if (payload_len != 0) memcpy(&packet[4], payload, payload_len);
    uint16_t crc = crc16_ccitt(&packet[2], 2 + payload_len);
    put_u16(&packet[4 + payload_len], crc);
    dispatch_packet(packet, 4 + payload_len + 2, wait);
}

/* Build framing around payload already placed at s_pkt[4..] and dispatch. */
static void emit_packet(uint8_t type, size_t payload_len, TickType_t wait)
{
    emit_packet_bytes(type, &s_pkt[4], payload_len, wait);
}

/* ------------------------------------------------------------------ */
/* BMI270 integrated stream                                            */
/* ------------------------------------------------------------------ */

#define BMI270_TASK_STACK_SIZE       6144
#define BMI270_TASK_PRIORITY         10
#define BMI270_INT_TIMEOUT_MS        60
#define BMI270_TEMP_INTERVAL_US      5000000LL
#define BMI270_DIAG_INTERVAL_US      5000000LL

/* IMU payload (little endian, 28 bytes):
 *   sequence u32, timestamp_us u32, sensor_time u32,
 *   status u8, flags u8,
 *   accel x/y/z i16, gyro x/y/z i16, temperature_c100 i16.
 * flags bit0=sample came from INT1, bit1=temperature valid,
 * bit2=accelerometer DRDY, bit3=gyroscope DRDY. */
static void emit_imu_packet(const bmi270_data_t *sample, uint32_t sequence,
                            uint32_t timestamp_us, bool via_interrupt,
                            bool temp_valid, int16_t temp_c100)
{
    uint8_t payload[IMU_PAYLOAD_BYTES];
    uint8_t *p = payload;
    uint8_t flags = (via_interrupt ? 0x01 : 0) |
                    (temp_valid ? 0x02 : 0) |
                    ((sample->status & BMI270_DRDY_ACC) ? 0x04 : 0) |
                    ((sample->status & BMI270_DRDY_GYR) ? 0x08 : 0);
    p = put_u32(p, sequence);
    p = put_u32(p, timestamp_us);
    p = put_u32(p, sample->sensor_time);
    *p++ = sample->status;
    *p++ = flags;
    p = put_i16(p, sample->acc_x);
    p = put_i16(p, sample->acc_y);
    p = put_i16(p, sample->acc_z);
    p = put_i16(p, sample->gyr_x);
    p = put_i16(p, sample->gyr_y);
    p = put_i16(p, sample->gyr_z);
    p = put_i16(p, temp_valid ? temp_c100 : INT16_MIN);
    emit_packet_bytes(PKT_TYPE_IMU, payload, (size_t)(p - payload), 0);
}

static void bmi270_task(void *arg)
{
    (void)arg;
    bool ready = false;
    bool temp_valid = false;
    int16_t temp_c100 = 0;
    uint32_t sequence = 0;
    uint32_t read_failures = 0;
    int64_t next_temp = 0;
    int64_t next_diag = 0;

    while (true) {
        power_shutdown_checkpoint(POWER_WORKER_BMI270);
        if (!ready) {
            ESP_LOGI(TAG, "initializing BMI270 (I2C SDA=%d SCL=%d INT1=GPIO%d)",
                     BMI270_I2C_SDA_IO, BMI270_I2C_SCL_IO, BMI270_INT1_IO);
            if (!bmi270_driver_init() || !bmi270_driver_start_data_ready_int()) {
                ESP_LOGW(TAG, "BMI270 unavailable; ECG/BLE continues, retrying in 5 s");
                bmi270_driver_deinit();
                power_shutdown_wait(5000);
                continue;
            }
            ready = true;
            temp_valid = false;
            next_temp = 0;
            next_diag = 0;
            read_failures = 0;
            ESP_LOGI(TAG, "BMI270 integrated stream enabled: accel/gyro 25 Hz, temp 5 s");
        }

        /* A timeout deliberately falls back to a direct read.  The packet
         * records POLL in flags, so the PC side can distinguish a missing
         * interrupt from a normal INT1-triggered sample. */
        bool via_interrupt = bmi270_driver_wait_data_ready(BMI270_INT_TIMEOUT_MS);
        power_shutdown_checkpoint(POWER_WORKER_BMI270);
        bmi270_data_t sample;
        if (!bmi270_driver_read(&sample)) {
            if (++read_failures >= 5) {
                ESP_LOGW(TAG, "BMI270 read failed repeatedly; restarting driver");
                bmi270_driver_deinit();
                ready = false;
            }
            power_shutdown_wait(20);
            continue;
        }
        read_failures = 0;
        if ((sample.status & (BMI270_DRDY_ACC | BMI270_DRDY_GYR)) == 0) {
            continue;
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_temp) {
            int16_t candidate = 0;
            if (bmi270_driver_read_temp(&candidate)) {
                temp_c100 = candidate;
                temp_valid = true;
            } else {
                temp_valid = false;
            }
            next_temp = now + BMI270_TEMP_INTERVAL_US;
        }

        emit_imu_packet(&sample, ++sequence, (uint32_t)now, via_interrupt,
                        temp_valid, temp_c100);

        if (now >= next_diag) {
            bmi270_diagnostics_t diagnostic;
            if (bmi270_driver_diagnostics(&diagnostic)) {
                ESP_LOGI(TAG, "BMI270 irq=%" PRIu32 " coalesced=%" PRIu32
                         " status=0x%04X int1=%d", diagnostic.int1_events,
                         diagnostic.coalesced_events, diagnostic.int_status,
                         diagnostic.int1_level);
            }
            next_diag = now + BMI270_DIAG_INTERVAL_US;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Acquisition state                                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    /* decimation block */
    uint8_t blk[BLOCK_BYTES];
    int     blk_n;
    int64_t ch1_sum;

    /* current DATA packet */
    uint8_t pkt_payload[DATA_PAYLOAD_MAX];
    int     pkt_blocks;
    uint32_t pkt_first_seq;
    uint8_t pkt_flags;
    uint32_t next_block_seq;

    /* statistics */
    uint32_t sample_idx;
    uint32_t dropped_total;
    uint32_t bad_frames;
    uint16_t sat_count;
    uint8_t  loff_or;
    uint8_t  loff_now;
    uint8_t  loff_prev;
    uint32_t last_event_sample;
    int32_t  ch1_min, ch1_max, ch2_min, ch2_max;
    bool     stats_valid;

    int32_t  last_ch1, last_ch2;
} acq_t;

static acq_t g;

static void stats_reset_window(void)
{
    g.sat_count = 0;
    g.loff_or = g.loff_now;
    g.stats_valid = false;
}

static void emit_status(uint8_t reason)
{
    uint8_t *p = &s_pkt[4];
    p = put_u32(p, g.sample_idx);
    p = put_u32(p, (uint32_t)esp_timer_get_time());
    *p++ = g.loff_or;
    *p++ = g.loff_now;
    p = put_u16(p, g.sat_count);
    p = put_u32(p, g.dropped_total);
    p = put_u32(p, g.bad_frames);
    p = put_i32(p, g.stats_valid ? g.ch1_min : 0);
    p = put_i32(p, g.stats_valid ? g.ch1_max : 0);
    p = put_i32(p, g.stats_valid ? g.ch2_min : 0);
    p = put_i32(p, g.stats_valid ? g.ch2_max : 0);
    taskENTER_CRITICAL(&s_battery_mux);
    uint16_t battery_mv = s_battery_mv;
    taskEXIT_CRITICAL(&s_battery_mux);
    p = put_u16(p, battery_mv);
    *p++ = reason;
    emit_packet(PKT_TYPE_STATUS, (size_t)(p - &s_pkt[4]), 0);
    stats_reset_window();
}

static void flush_data_packet(void)
{
    if (g.pkt_blocks == 0) {
        return;
    }
    uint8_t *p = &s_pkt[4];
    p = put_u32(p, g.pkt_first_seq);
    p = put_u32(p, (uint32_t)esp_timer_get_time());
    *p++ = g.pkt_flags;
    *p++ = (uint8_t)g.pkt_blocks;
    p = put_u32(p, s_config_revision);
    p = put_u32(p, s_acquisition_epoch);
    size_t body = (size_t)g.pkt_blocks * BLOCK_BYTES;
    memcpy(p, g.pkt_payload, body);
    p += body;
    emit_packet(PKT_TYPE_DATA, (size_t)(p - &s_pkt[4]), 0);
    g.pkt_blocks = 0;
    g.pkt_flags = 0;
}

static void finish_block(void)
{
    /* CH1 block average (round to nearest) placed after the 10 CH2 samples. */
    int64_t s = g.ch1_sum;
    int32_t avg = (int32_t)((s >= 0 ? s + RESP_DECIM / 2 : s - RESP_DECIM / 2) / RESP_DECIM);
    put24(&g.blk[RESP_DECIM * 3], avg);

    if (avg < g.ch1_min || !g.stats_valid) g.ch1_min = avg;
    if (avg > g.ch1_max || !g.stats_valid) g.ch1_max = avg;

    if (g.pkt_blocks == 0) {
        g.pkt_first_seq = g.next_block_seq;
    }
    memcpy(&g.pkt_payload[g.pkt_blocks * BLOCK_BYTES], g.blk, BLOCK_BYTES);
    g.pkt_blocks++;
    g.next_block_seq++;
    g.blk_n = 0;
    g.ch1_sum = 0;

    if (g.pkt_blocks >= BLOCKS_PER_PACKET) {
        flush_data_packet();
    }
}

static void push_sample(int32_t ch1, int32_t ch2)
{
    put24(&g.blk[g.blk_n * 3], ch2);
    g.ch1_sum += ch1;
    g.blk_n++;
    g.sample_idx++;
    g.last_ch1 = ch1;
    g.last_ch2 = ch2;

    if (!g.stats_valid) {
        g.ch2_min = g.ch2_max = ch2;
        g.ch1_min = g.ch1_max = ch1;
        g.stats_valid = true;
    } else {
        if (ch2 < g.ch2_min) g.ch2_min = ch2;
        if (ch2 > g.ch2_max) g.ch2_max = ch2;
    }
    if (ch2 >= 0x7FFF00 || ch2 <= -0x7FFF00) {
        if (g.sat_count < 0xFFFF) g.sat_count++;
    }

    if (g.blk_n >= RESP_DECIM) {
        finish_block();
    }
}

static void handle_frame(const uint8_t raw[9], uint32_t missed)
{
    /* STATUS word must start with 1100b; anything else is a corrupt frame. */
    if ((raw[0] >> 4) != 0x0C) {
        g.bad_frames++;
        return;
    }

    uint32_t status = ((uint32_t)raw[0] << 16) | ((uint32_t)raw[1] << 8) | raw[2];
    uint8_t loff = (uint8_t)((status >> 15) & 0x1F);
    g.loff_now = loff;
    g.loff_or |= loff;

    int32_t ch1 = sign24(&raw[3]);
    int32_t ch2 = sign24(&raw[6]);

    /* Keep the timeline intact after missed DRDY events by repeating the
     * previous sample; the packet flag tells the PC that fill was used. */
    if (missed > 0) {
        uint32_t fill = missed > MAX_FILL_SAMPLES ? MAX_FILL_SAMPLES : missed;
        g.dropped_total += missed;
        g.pkt_flags |= 0x02;
        for (uint32_t i = 0; i < fill; ++i) {
            push_sample(g.last_ch1, g.last_ch2);
        }
    }

    uint8_t sense = s_control_config.values[ads_control_reg_index(ADS1292_REG_LOFFSENS)];
    uint8_t ecg_loff_mask = sense & LOFF_MASK_CH2;
    if (loff & ecg_loff_mask) {
        g.pkt_flags |= 0x01;
    }
    if (loff & sense & 0x03) g.pkt_flags |= 0x04; /* CH1 electrode off */
    if ((s_control_config.values[ads_control_reg_index(ADS1292_REG_RLDSENS)] & 0x10) &&
        (loff & 0x10)) g.pkt_flags |= 0x08; /* RLD off when its sense is enabled */
    push_sample(ch1, ch2);

    /* Lead-off state change (CH2 electrodes) -> immediate status event. */
    if ((loff & 0x1F) != (g.loff_prev & 0x1F) &&
        g.sample_idx - g.last_event_sample >= LOFF_EVENT_MIN_GAP) {
        g.loff_prev = loff;
        g.last_event_sample = g.sample_idx;
        flush_data_packet();
        emit_status(1);
    }
}

/* All commands are serialized on the acquisition task. The NimBLE callback
 * validates the complete batch and copies it into this bounded mailbox. */
static bool receive_control(const ads_control_command_t *command)
{
    return !power_shutdown_requested() && s_control_queue &&
           xQueueSend(s_control_queue, command, 0) == pdTRUE;
}

static void emit_control(const ads_control_command_t *command, uint8_t result)
{
    size_t n = ads_control_reply(&s_pkt[4], command, result, s_control_state,
                                s_config_revision, s_acquisition_epoch, &s_control_config);
    /* Unsolicited periodic snapshots must never stall acquisition. */
    TickType_t wait = (command->opcode == ADS_CONTROL_QUERY && command->request_id == 0) ? 0 : pdMS_TO_TICKS(500);
    emit_packet(PKT_TYPE_CONTROL, n, wait);
}

static void clear_acquisition_window(void)
{
    /* Completed packets precede the reply in the FIFO. Drop only the partial
     * decimation block; keep wire seq monotonic over pauses/reconfiguration. */
    uint32_t seq = g.next_block_seq, dropped = g.dropped_total, bad = g.bad_frames;
    memset(&g, 0, sizeof(g));
    g.next_block_seq = seq;
    g.sample_idx = seq * RESP_DECIM;
    g.dropped_total = dropped;
    g.bad_frames = bad;
    stats_reset_window();
}

static void apply_control(const ads_control_command_t *command)
{
    if (command->opcode == ADS_CONTROL_QUERY) {
        emit_control(command, ADS_CONTROL_OK);
        return;
    }
    ads_control_config_t candidate = s_control_config;
    if (!ads_control_candidate(&s_control_config, command, &candidate)) return;
    if (command->opcode == ADS_CONTROL_SET && s_control_state != ADS_CONTROL_RUNNING) {
        /* STOP remains latched until START or power-cycle. No WREG here. */
        s_control_config = candidate;
        ++s_config_revision;
        emit_control(command, ADS_CONTROL_STAGED);
        return;
    }
    bool ok = !s_drdy_isr_ready || set_drdy_interrupt_enabled(false);
    flush_data_packet();
    s_control_state = ADS_CONTROL_STOPPED;
    gpio_set_level(RUN_STATUS_GPIO, 0);
    ++s_acquisition_epoch;
    if (ok) {
        if (command->opcode == ADS_CONTROL_STOP) {
            /* No WREG, OFFSETCAL, RDATAC or software RESET; only GPIO reset. */
            ok = ads1292r_reset_only(&s_ads1292r);
        } else {
            /* Reconfigure leaves START LOW, just like the boot path. */
            ok = ads1292r_reconfigure(&s_ads1292r, &candidate);
        }
    }
    clear_acquisition_window();
    (void)ulTaskNotifyTake(pdTRUE, 0);
    if (ok && command->opcode != ADS_CONTROL_STOP) {
        ok = start_ads_acquisition();
        if (ok) {
            s_control_state = ADS_CONTROL_RUNNING;
            if (command->opcode == ADS_CONTROL_SET) {
                s_control_config = candidate;
                ++s_config_revision;
            }
        }
    }
    if (!ok) {
        (void)ads1292r_reset_only(&s_ads1292r);
        s_control_state = ADS_CONTROL_FAULT;
        ESP_LOGE(TAG, "remote ADS command failed; left reset/unconfigured, GPIO13 LOW");
    }
    /* This task owns acquisition and cannot read notified samples until this
     * function returns. Enqueue the reply before any new-config DATA packet. */
    emit_control(command, ok ? ADS_CONTROL_OK : ADS_CONTROL_HW_ERROR);
    emit_status(0);
}

static void report_control_if_ready(void)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_control_report_us >= BATTERY_REPORT_MS * 1000LL) {
        s_last_control_report_us = now;
        const ads_control_command_t query = {.opcode = ADS_CONTROL_QUERY, .request_id = 0};
        emit_control(&query, ADS_CONTROL_OK);
    }
}

static void update_run_status(void)
{
#if RUN_STATUS_ENABLE
    int level = s_control_state == ADS_CONTROL_RUNNING ? (int)((esp_timer_get_time() / 2000000LL) & 1) : 0;
    gpio_set_level(RUN_STATUS_GPIO, level);
#endif
}

/* ADC sampling never runs in the high-priority 2 ms ECG path. */
static void battery_task(void *arg)
{
    (void)arg;
    power_shutdown_checkpoint(POWER_WORKER_BATTERY);
    bool ready = battery_adc_init();
    int64_t next_read_us = esp_timer_get_time();
    while (true) {
        power_shutdown_checkpoint(POWER_WORKER_BATTERY);
        uint32_t mv = 0;
        bool valid = ready && battery_adc_read_mv(&mv) && mv < 0xFFFF;
        taskENTER_CRITICAL(&s_battery_mux);
        s_battery_mv = valid ? (uint16_t)mv : 0xFFFF;
        ++s_battery_generation;
        taskEXIT_CRITICAL(&s_battery_mux);
        next_read_us += BATTERY_REPORT_MS * 1000LL;
        int64_t remaining_us = next_read_us - esp_timer_get_time();
        if (remaining_us > 0) power_shutdown_wait((uint32_t)((remaining_us + 999) / 1000));
        else next_read_us = esp_timer_get_time();
        power_shutdown_checkpoint(POWER_WORKER_BATTERY);
        if (!ready) {
            ready = battery_adc_init();
        }
    }
}

static void report_battery_if_ready(void)
{
    taskENTER_CRITICAL(&s_battery_mux);
    uint32_t generation = s_battery_generation;
    taskEXIT_CRITICAL(&s_battery_mux);
    int64_t now = esp_timer_get_time();
    /* Even a failed ADC task allocation must not suppress periodic status. */
    if (generation != s_battery_reported ||
        (generation == 0 && now - s_battery_last_report_us >= BATTERY_REPORT_MS * 1000LL)) {
        s_battery_reported = generation;
        s_battery_last_report_us = now;
        /* Kept on the ECG task, including when DRDY has stalled. */
        emit_status(0);
    }
}

/* ------------------------------------------------------------------ */
/* DRDY interrupt + acquisition task                                   */
/* ------------------------------------------------------------------ */

static void IRAM_ATTR drdy_isr(void *arg)
{
    TaskHandle_t task = (TaskHandle_t)arg;
    ++s_drdy_irq_total;
    if (task == NULL) return;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

static bool set_drdy_interrupt_enabled(bool enabled)
{
    if (!s_drdy_isr_ready) {
        ESP_LOGE(TAG, "ADS DRDY GPIO%d handler is not ready", s_ads1292r.drdy_pin);
        return false;
    }
    esp_err_t err = enabled ? gpio_intr_enable(s_ads1292r.drdy_pin)
                            : gpio_intr_disable(s_ads1292r.drdy_pin);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADS DRDY GPIO%d interrupt %s failed: %s",
                 s_ads1292r.drdy_pin, enabled ? "enable" : "disable", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool setup_drdy_interrupt(void)
{
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "ADS gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return false;
    }
    /* ESP_ERR_INVALID_STATE here only means the shared service exists. BMI270
     * owns GPIO10, ADS owns GPIO11. Never uninstall the shared service. */
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "ADS reuses existing GPIO ISR service (BMI270 has a separate handler)");
    }
    err = gpio_set_intr_type(s_ads1292r.drdy_pin, GPIO_INTR_NEGEDGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADS DRDY GPIO%d falling-edge setup failed: %s",
                 s_ads1292r.drdy_pin, esp_err_to_name(err));
        return false;
    }
    err = gpio_isr_handler_add(s_ads1292r.drdy_pin, drdy_isr, (void *)xTaskGetCurrentTaskHandle());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADS DRDY GPIO%d handler registration failed: %s",
                 s_ads1292r.drdy_pin, esp_err_to_name(err));
        return false;
    }
    s_drdy_isr_ready = true;
    /* handler_add() enables the pin; hold it disabled until the start helper. */
    if (!set_drdy_interrupt_enabled(false)) return false;
    ESP_LOGI(TAG, "ADS DRDY ISR registered: GPIO%d falling edge, initial level=%d; BMI270 INT1 GPIO%d independent",
             s_ads1292r.drdy_pin, gpio_get_level(s_ads1292r.drdy_pin), BMI270_INT1_IO);
    return true;
}

static bool start_ads_acquisition(void)
{
    /* Called only with START LOW and stale notifications already drained. */
    if (!set_drdy_interrupt_enabled(true)) return false;
    if (!ads1292r_begin_stream(&s_ads1292r)) {
        (void)set_drdy_interrupt_enabled(false);
        return false;
    }
    s_drdy_diag.stall_warned = false;
    return true;
}

static void log_ads_drdy(bool force)
{
    int64_t now = esp_timer_get_time();
    int64_t elapsed = now - s_drdy_diag.log_us;
    if (!force && elapsed < ADS_DRDY_LOG_MS * 1000LL) return;
    uint32_t irq = s_drdy_irq_total;
    double seconds = elapsed > 0 ? elapsed / 1000000.0 : 1.0;
    ESP_LOGI(TAG, "ADS DRDY: state=%u GPIO%d=%d START GPIO%d=%d RESET GPIO%d=%d | irq_total=%" PRIu32
             " irq_hz=%.1f spi_reads_total=%" PRIu32 " valid_total=%" PRIu32 " valid_hz=%.1f"
             " poll_total=%" PRIu32 " timeouts_total=%" PRIu32 " spi_errors_total=%" PRIu32
             " invalid_total=%" PRIu32 " last_status=0x%06" PRIX32,
             s_control_state, s_ads1292r.drdy_pin, gpio_get_level(s_ads1292r.drdy_pin),
             s_ads1292r.start_pin, gpio_get_level(s_ads1292r.start_pin),
             s_ads1292r.reset_pin, gpio_get_level(s_ads1292r.reset_pin),
             irq, (irq - s_drdy_diag.log_irq) / seconds, s_drdy_diag.spi_reads_total,
             s_drdy_diag.valid_total, (s_drdy_diag.valid_total - s_drdy_diag.log_valid) / seconds,
             s_drdy_diag.poll_total, s_drdy_diag.timeouts_total, s_drdy_diag.spi_errors_total,
             s_drdy_diag.invalid_total, s_drdy_diag.last_status);
    s_drdy_diag.log_us = now;
    s_drdy_diag.log_irq = irq;
    s_drdy_diag.log_valid = s_drdy_diag.valid_total;
}

static void ecg_task(void *arg)
{
    (void)arg;
    ads1292r_data_t data;
    power_shutdown_checkpoint(POWER_WORKER_ECG);
    stats_reset_window();
    /* GPIO11 is armed BEFORE SPI START, on both boot and remote restart. */
    if (setup_drdy_interrupt() && start_ads_acquisition()) {
        s_control_state = ADS_CONTROL_RUNNING;
    } else {
        (void)ads1292r_reset_only(&s_ads1292r);
        s_control_state = ADS_CONTROL_FAULT;
        ESP_LOGE(TAG, "ADS acquisition not started; initialization/IRQ error, GPIO13 stays LOW");
    }
    ESP_LOGI(TAG, "BLE ECG stream: CH2 %d SPS, CH1 block averages %d SPS",
             ADS_SPS, ADS_SPS / RESP_DECIM);
    log_ads_drdy(true);

    while (true) {
        power_shutdown_checkpoint(POWER_WORKER_ECG);
        ads_control_command_t command;
        if (xQueueReceive(s_control_queue, &command, 0) == pdTRUE) {
            apply_control(&command);
            continue;
        }
        update_run_status();
        report_battery_if_ready();
        report_control_if_ready();
        log_ads_drdy(false);
        if (s_control_state != ADS_CONTROL_RUNNING) {
            /* A stopped ADS is intentional, not DRDY stall/bad-frame spam. */
            power_shutdown_wait(100);
            continue;
        }
        uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ADS_DRDY_TIMEOUT_MS));
        power_shutdown_checkpoint(POWER_WORKER_ECG);
        bool poll = false;
        if (n == 0) {
            ++s_drdy_diag.timeouts_total;
            int level = gpio_get_level(s_ads1292r.drdy_pin);
            if (!s_drdy_diag.stall_warned) {
                s_drdy_diag.stall_warned = true;
                ESP_LOGW(TAG, "ADS no DRDY notification for %d ms; GPIO%d=%d: %s",
                         ADS_DRDY_TIMEOUT_MS, s_ads1292r.drdy_pin, level,
                         level == 0 ? "ready LOW, diagnostic POLL only (not an IRQ PASS or 500-SPS stream)"
                                    : "HIGH, no data-ready observed; not forcing a SPI data read");
                log_ads_drdy(true);
            }
            /* Catch a ready level missed before the ISR was armed. Never read
             * just because DRDY is HIGH, and never mistake POLL for IRQ. */
            if (level != 0) {
                flush_data_packet();
                g.bad_frames++;
                emit_status(2);
                continue;
            }
            /* An ISR may have arrived while the timeout diagnostic ran. */
            n = ulTaskNotifyTake(pdTRUE, 0);
            poll = n == 0;
            if (poll && gpio_get_level(s_ads1292r.drdy_pin) != 0) {
                flush_data_packet();
                g.bad_frames++;
                emit_status(2);
                continue;
            }
        }

        if (ads1292r_read_data(&s_ads1292r, &data)) {
            ++s_drdy_diag.spi_reads_total;
            if (poll) ++s_drdy_diag.poll_total;
            s_drdy_diag.last_status = ((uint32_t)data.raw[0] << 16) |
                                     ((uint32_t)data.raw[1] << 8) | data.raw[2];
            bool valid = (data.raw[0] >> 4) == 0x0C;
            if (valid) {
                if (!poll) {
                    ++s_drdy_diag.valid_total;
                    s_drdy_diag.stall_warned = false;
                }
                if ((!poll && s_drdy_diag.valid_total == 1) ||
                    (poll && s_drdy_diag.poll_total == 1)) {
                    ESP_LOGI(TAG, "ADS first valid frame: source=%s status=0x%06" PRIX32
                             " CH1=%" PRId32 " CH2=%" PRId32,
                             poll ? "POLL" : "IRQ", s_drdy_diag.last_status,
                             sign24(&data.raw[3]), sign24(&data.raw[6]));
                }
            } else {
                ++s_drdy_diag.invalid_total;
                if (s_drdy_diag.invalid_total == 1) {
                    ESP_LOGW(TAG, "ADS first invalid frame: status=0x%06" PRIX32 " (expected 0xCxxxxx)",
                             s_drdy_diag.last_status);
                }
            }
            if (poll) {
                /* A 200-ms diagnostic probe has unknown sample timing. Do not
                 * send it as if it were a normal contiguous 500-SPS sample. */
                flush_data_packet();
                g.bad_frames++;
                emit_status(2);
            } else {
                handle_frame(data.raw, n - 1);
            }
        } else {
            ++s_drdy_diag.spi_errors_total;
            g.bad_frames++;
            if (s_drdy_diag.spi_errors_total == 1) {
                ESP_LOGE(TAG, "ADS SPI data read failed; DRDY GPIO%d=%d START GPIO%d=%d",
                         s_ads1292r.drdy_pin, gpio_get_level(s_ads1292r.drdy_pin),
                         s_ads1292r.start_pin, gpio_get_level(s_ads1292r.start_pin));
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Run indicator and app_main                                          */
/* ------------------------------------------------------------------ */

#if RUN_STATUS_ENABLE
static void start_run_status_gpio(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = BIT64(RUN_STATUS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        return;
    }
    gpio_set_level(RUN_STATUS_GPIO, 0);
    /* ECG task drives the indicator, so STOP cannot race a blinking task. */
}
#else
static void start_run_status_gpio(void) {}
#endif

static void hold_after_init_failure(void)
{
    /* app_main will perform no more peripheral I/O. GPIO9 must still work
     * when an ADS/BLE/task initialization fails. */
    power_shutdown_boot_complete();
    while (true) {
        power_shutdown_checkpoint(0);
        ESP_LOGE(TAG, "ADS bring-up halted; check the previous error and wiring");
        power_shutdown_wait(5000);
    }
}

void app_main(void)
{
    power_shutdown_init();
    power_shutdown_boot_checkpoint();
    ESP_LOGI(TAG, "ESP32-H2 integrated ECG/respiration + BMI270 firmware; BLE data, USB logs");

    ads_control_defaults(&s_control_config);
    s_control_queue = xQueueCreate(8, sizeof(ads_control_command_t));
    if (!s_control_queue) hold_after_init_failure();
    ble_link_set_control_receiver(receive_control);

    /* Bring up BLE first so PHY/radio init runs before other peripherals. */
    esp_err_t ble_err = ble_link_init();
    power_shutdown_boot_checkpoint();
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "BLE init failed: %s", esp_err_to_name(ble_err));
        hold_after_init_failure();
    }

    ESP_LOGI(TAG, "BLE initialized; ECG-H2 carries ECG/respiration, IMU, battery and ADS control");

    ESP_LOGI(TAG, "waiting 2000 ms for ADS1292R power-up");
    power_shutdown_wait(2000);
    power_shutdown_boot_checkpoint();

    if (!ads1292r_init(&s_ads1292r, ADS1292R_CS_GPIO, ADS1292R_DRDY_GPIO,
                       ADS1292R_RESET_GPIO, ADS1292R_START_GPIO, SPI2_HOST)) {
        power_shutdown_boot_checkpoint();
        ESP_LOGE(TAG, "ADS1292R initialization failed; running GPIO bit-bang diagnostic");
        ads1292r_diag_pins();
        hold_after_init_failure();
    }
    power_shutdown_boot_checkpoint();

    uint8_t id = ads1292r_read_device_id(&s_ads1292r);
    if (id != DEVICE_ID_ADS1292R && id != DEVICE_ID_ADS1292) {
        ESP_LOGE(TAG, "post-init ADS ID check failed: 0x%02X", id);
        hold_after_init_failure();
    }
    ESP_LOGI(TAG, "ADS communication check passed: device ID=0x%02X", id);

    /* Start the GPIO13 run indicator only after ADS init succeeded. */
    start_run_status_gpio();
    /* ECG task sets RUNNING after DRDY is armed and SPI START/RDATAC are sent. */

    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_LOGI(TAG, "ADS configuration complete; acquisition task will arm DRDY then send SPI START (0x08); hardware START stays LOW");

    memset(&g, 0, sizeof(g));
    crc16_init_table();

    power_shutdown_boot_checkpoint();
    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_item_t));
    if (s_tx_queue == NULL ||
        xTaskCreate(ble_tx_task, "ble_tx", BLE_TX_STACK_SIZE, NULL,
                    BLE_TX_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create BLE TX queue/task");
        hold_after_init_failure();
    }

    power_shutdown_worker_started(POWER_WORKER_BLE_TX);
    power_shutdown_boot_checkpoint();
    ESP_LOGI(TAG, "Shared BLE packet stream ready; USB packet mirror=%d", USB_DEBUG_OUTPUT);

    if (xTaskCreate(bmi270_task, "bmi270", BMI270_TASK_STACK_SIZE, NULL,
                    BMI270_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGW(TAG, "BMI270 task unavailable; ECG/BLE continues without IMU");
    } else {
        power_shutdown_worker_started(POWER_WORKER_BMI270);
    }
    power_shutdown_boot_checkpoint();

    if (xTaskCreate(battery_task, "battery", 3072, NULL, 2, NULL) != pdPASS) {
        ESP_LOGW(TAG, "battery task unavailable; ECG still runs, voltage n/a");
    } else {
        power_shutdown_worker_started(POWER_WORKER_BATTERY);
    }
    power_shutdown_boot_checkpoint();

    BaseType_t ok = xTaskCreate(ecg_task, "ecg_task", ECG_TASK_STACK_SIZE, NULL,
                                ECG_TASK_PRIORITY, &s_ecg_task_handle);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create ECG acquisition task");
        hold_after_init_failure();
    }
    power_shutdown_worker_started(POWER_WORKER_ECG);
    power_shutdown_boot_complete();

    vTaskDelete(NULL);
}
