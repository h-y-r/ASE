/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* Includes */
#include "gap.h"
#include "common.h"
#include "host/ble_hs.h"
#include "nimble/ble.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* Private function declarations */
inline static void format_addr(char *addr_str, uint8_t addr[]);
static void start_advertising(void);
static int uart_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int uart_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_handle;


int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle_global = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected, handle %d", conn_handle_global);
            } else {
                ESP_LOGI(TAG, "Connection failed; status=%d", event->connect.status);
                /* Restart advertising */
                adv_init();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
            if (conn_handle_global == event->disconnect.conn.conn_handle) {
                conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
            }
            /* Restart advertising */
            adv_init();
            break;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event: attr_handle=%d, reason=%d, prev=%d, cur=%d",
                event->subscribe.attr_handle,
                event->subscribe.reason,
                event->subscribe.prev_notify,
                event->subscribe.cur_notify);
            break;

        default:
            break;
    }
    return 0;
}
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID128_DECLARE(0x6E, 0x40, 0x00, 0x00, 0xB5, 0xA3, 0xF3, 0x93,
            0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0x41, 0x3E),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID128_DECLARE(0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93,
                    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0x41, 0x3E),
                .access_cb = uart_rx_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = BLE_UUID128_DECLARE(0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93,
                    0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0x41, 0x3E),
                .access_cb = uart_tx_cb,
                .val_handle = &tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0}
        },
    },
    {0}
};

/* GATT server definition */


inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static int uart_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const struct os_mbuf *om = ctxt->om;
    ESP_LOGI(TAG, "Received from phone: %.*s", om->om_len, om->om_data);
    return 0;
}

static int uart_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0; // Not readable
}

static void start_advertising(void) {
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_gap_adv_params adv_params = {0};
    const char *name = ble_svc_gap_device_name();
    int rc;

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "set adv fields failed: %d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
        ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising started.");
}

/* Public functions */

void adv_init(void) {
    char addr_str[18];
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "no BT address!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "address infer failed: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "address copy failed: %d", rc);
        return;
    }

    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "Device address: %s", addr_str);

    start_advertising();
}

int gap_init(void) {
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "device name set failed: %d", rc);
        return rc;
    }

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) return rc;

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) return rc;

    return 0;
}

/* Optional: callable function to send data */
void ble_uart_send(const uint8_t *data, size_t len) {
    if (conn_handle_global == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "No connection; cannot send.");
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for send.");
        return;
    }

    int rc = ble_gattc_notify_custom(conn_handle_global, tx_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Notify failed: %d", rc);
    }
}
