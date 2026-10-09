#include "ble_link.h"

#include <string.h>

#include <esp_log.h>
#include <nvs_flash.h>

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_link";

#define BLE_DEVICE_NAME "ECG-H2"

static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_notify_handle;
static volatile bool s_subscribed;
static volatile uint32_t s_dropped;
static ble_control_receiver_t s_control_receiver;
static bool s_initialized, s_host_started;
static volatile bool s_shutting_down;

/* Custom 128-bit UUIDs (little-endian byte order as required by NimBLE). */
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0xE3, 0xA1, 0x5C, 0x7B, 0x4D, 0x21,
                     0x9A, 0x3E, 0x6F, 0x10, 0x00, 0x00, 0xEC, 0x6C);
static const ble_uuid128_t s_chr_uuid =
    BLE_UUID128_INIT(0x02, 0x00, 0xE3, 0xA1, 0x5C, 0x7B, 0x4D, 0x21,
                     0x9A, 0x3E, 0x6F, 0x10, 0x00, 0x00, 0xEC, 0x6C);

static const ble_uuid128_t s_control_uuid =
    BLE_UUID128_INIT(0x03, 0x00, 0xE3, 0xA1, 0x5C, 0x7B, 0x4D, 0x21,
                     0x9A, 0x3E, 0x6F, 0x10, 0x00, 0x00, 0xEC, 0x6C);

void ble_link_set_control_receiver(ble_control_receiver_t receiver) { s_control_receiver = receiver; }

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;
    if (s_shutting_down) return BLE_ATT_ERR_UNLIKELY;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_READ_NOT_PERMITTED;
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 5 || len > ADS_CONTROL_COMMAND_MAX) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    uint8_t data[ADS_CONTROL_COMMAND_MAX];
    if (os_mbuf_copydata(ctxt->om, 0, len, data) != 0) return BLE_ATT_ERR_UNLIKELY;
    ads_control_command_t command;
    if (!ads_control_decode(data, len, &command)) return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    if (!s_control_receiver || !s_control_receiver(&command)) return BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0; /* ATT acceptance is NOT the hardware result; wait for type 0x03. */
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_chr_uuid.u,
                .access_cb = gatt_access_cb,
                .val_handle = &s_notify_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &s_control_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            { 0 },
        },
    },
    { 0 },
};

static void start_advertising(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (s_shutting_down) return 0;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "connected, handle=%d", s_conn_handle);
            /* Connection parameters: 30-50 ms interval (fewer connection events
             * competing with the ADS acquisition task), 4 s timeout. */
            struct ble_gap_upd_params p = {
                .itvl_min = 24,   /* 1.25 ms units -> 30 ms */
                .itvl_max = 40,   /* 50 ms */
                .latency = 0,
                .supervision_timeout = 400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            ble_gap_update_params(s_conn_handle, &p);
        } else {
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected, reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed = false;
        start_advertising();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_notify_handle) {
            s_subscribed = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG, "notify %s", s_subscribed ? "enabled" : "disabled");
            if (s_subscribed && s_control_receiver) {
                const ads_control_command_t query = {.opcode = ADS_CONTROL_QUERY, .request_id = 0};
                (void)s_control_receiver(&query);
            }
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU=%d", event->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    if (s_shutting_down) return;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv = {0};
    adv.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv.itvl_min = 800;  /* 500 ms, low-power advertising */
    adv.itvl_max = 1600; /* 1 s */
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv,
                           gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    } else if (rc == 0) {
        ESP_LOGI(TAG, "advertising as \"%s\"", BLE_DEVICE_NAME);
    }
}

static void on_sync(void)
{
    if (s_shutting_down) return;
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "address setup rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_link_init(void)
{
    if (s_shutting_down) return ESP_ERR_INVALID_STATE;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    ble_att_set_preferred_mtu(247);

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_gatt_svcs);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT setup rc=%d", rc);
        return ESP_FAIL;
    }

    nimble_port_freertos_init(host_task);
    s_host_started = true;
    return ESP_OK;
}

esp_err_t ble_link_shutdown(void)
{
    s_shutting_down = true; /* Disconnect/ADV_COMPLETE must not re-advertise. */
    s_subscribed = false;
    s_control_receiver = NULL;
    if (!s_initialized) return ESP_OK;
    if (s_host_started) {
        (void)ble_gap_adv_stop();
        int rc = nimble_port_stop(); /* Stops links and waits for the host. */
        if (rc != 0) {
            ESP_LOGE(TAG, "NimBLE stop failed: %d; proceeding to Deep-sleep", rc);
            return ESP_FAIL; /* Do not free a live host's resources. */
        }
        s_host_started = false;
    }
    /* ESP-IDF v6.0.1 deinit disables/deinitializes the controller too. */
    esp_err_t err = nimble_port_deinit();
    if (err == ESP_OK) s_initialized = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    return err;
}

bool ble_link_is_streaming(void)
{
    return !s_shutting_down && s_subscribed && s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}

uint32_t ble_link_dropped(void)
{
    return s_dropped;
}

bool ble_link_send(const uint8_t *data, size_t len)
{
    if (!ble_link_is_streaming() || data == NULL || len == 0) {
        return false;
    }

    uint16_t conn = s_conn_handle;
    uint16_t mtu = ble_att_mtu(conn);
    size_t chunk_max = (mtu > 3) ? (size_t)(mtu - 3) : 20;

    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > chunk_max) {
            n = chunk_max;
        }
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data + off, (uint16_t)n);
        if (om == NULL) {
            s_dropped++;
            return false;
        }
        int rc = ble_gatts_notify_custom(conn, s_notify_handle, om);
        if (rc != 0) {
            /* om is consumed by the call even on failure. */
            s_dropped++;
            return false;
        }
        off += n;
    }
    return true;
}
