
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include <string.h>


#define TAG "NimBLE_Beacon"
#define DEVICE_NAME "NimBLE_Beacon"


void ble_uart_send(const uint8_t *data, size_t len);
void ble_store_config_init(void);
void adv_init(void);
int gap_init(void);

inline static void format_addr(char *addr_str, uint8_t addr[]);
static void start_advertising(void);
static int uart_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int uart_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg);

static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static uint16_t conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_handle;

void mcpwm_init_mostek_h()
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_MOST_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_MOST_IN2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_MOST_IN3);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, PWM_MOST_IN4);

    mcpwm_config_t pwm_config = {
        .frequency = PWM_FREQ,
        .cmpr_a = 0.0f,     // PWM_MOST_IN1 dla kanału A (0%)
        .cmpr_b = 0.0f,     // PWM_MOST_IN1 dla kanału B (0%)
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_1,
    };
    // inicjalizacja timerów MCPWM
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    // konfiguracja synchronizacji timerów
    mcpwm_sync_config_t sync_cfg = {
        .sync_sig = MCPWM_SELECT_GPIO_SYNC0,  // synchronizacja z GPIO SYNC0
        .timer_val = 0,                       // licznik po synchronizacji
        .count_direction = MCPWM_UP_COUNTER   // kierunek zliczania po synchronizacji
    };

    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_0, &sync_cfg);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_cfg);
    
    // wartości początkowe dla kierunków silników
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    //mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
}

void sterowanie_silnikami(const char* kierunek, float procent_wypelnienia)
{
    if (strcmp(kierunek, "przod") == 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 60+(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 60+(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 40-(procent_wypelnienia*0.4));

    } else if (strcmp(kierunek, "tyl") == 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 60+(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 60+(procent_wypelnienia*0.4));

    } else if (strcmp(kierunek, "lewo") == 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 60+(procent_wypelnienia*0.4)); // (100-60)/100, początek działania silnika = 60%
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 60+(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 60+(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 60+(procent_wypelnienia*0.4));
        
    } else if (strcmp(kierunek, "prawo") == 0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 40-(procent_wypelnienia*0.4));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 40-(procent_wypelnienia*0.4));
        
    } else if (strcmp(kierunek, "stop") == 0) { // tu nie ma znaczenia jaką wartość podasz, bo wartość jest przypisana
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 50.0); // wartości poniżej lub powyżej tej wartości sprawiają, że silnik działa w jakimś kierunku (teoretycznie xd)
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 50.0);
    }
}

int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle_global = event->connect.conn_handle;
                ESP_LOGI(TAG, "Connected, handle %d", conn_handle_global);
            } else {
                ESP_LOGI(TAG, "Connection failed; status=%d", event->connect.status);
                adv_init();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
            if (conn_handle_global == event->disconnect.conn.conn_handle) {
                conn_handle_global = BLE_HS_CONN_HANDLE_NONE;
            }
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




inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static int uart_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    const struct os_mbuf *om = ctxt->om;
    
    if(strcmp((const char *)om->om_data, "w") == 0)
    {
        if (!motors_first_time)
        {
            motors_first_time = 1;
            mcpwm_init_mostek_h();
        }
		sterowanie_silnikami("przod", 95);
        ESP_LOGI(TAG, "autko jedzie do przodu");
	}
	else if(strcmp((const char *)om->om_data, "s") == 0)
    {
        if (!motors_first_time)
        {
            motors_first_time = 1;
            mcpwm_init_mostek_h();
        }
		sterowanie_silnikami("tyl", 95);
        ESP_LOGI(TAG, "autko jedzie do tylu");
	}
	else if(strcmp((const char *)om->om_data, "a") == 0)
    {
        if (!motors_first_time)
        {
            motors_first_time = 1;
            mcpwm_init_mostek_h();
        }
		sterowanie_silnikami("lewo", 95);
        ESP_LOGI(TAG, "autko kreci sie w lewo");
	}
	else if(strcmp((const char *)om->om_data, "d") == 0)
    {
        if (!motors_first_time)
        {
            motors_first_time = 1;
            mcpwm_init_mostek_h();
        }
		sterowanie_silnikami("prawo", 95);
        ESP_LOGI(TAG, "autko kreci sie w prawo");
	}
	else if(strcmp((const char *)om->om_data, "ws") == 0 || strcmp((const char *)om->om_data, "stop") == 0 || strcmp((const char *)om->om_data, "x") == 0)
    {
        if (!motors_first_time)
        {
            motors_first_time = 1;
            mcpwm_init_mostek_h();
        }
		sterowanie_silnikami("stop", 95);
        ESP_LOGI(TAG, "autko stanelo");
	}
    
    ESP_LOGI(TAG, "Received from phone: %.*s", om->om_len, om->om_data);
    ble_uart_send(om->om_data, om->om_len);    
    return 0;
}

static int uart_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
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


static void on_stack_sync(void) {
    adv_init();
}

static void on_stack_reset(int reason) {
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void nimble_host_config_init(void) {
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    ble_store_config_init();
}

static void nimble_host_task(void *param) {
    ESP_LOGI(TAG, "nimble host task started");

    nimble_port_run();

    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to init NVS flash, code: %d", ret);
        return;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to init NimBLE stack, code: %d", ret);
        return;
    }

    int rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to init GAP service, code: %d", rc);
        return;
    }

    nimble_host_config_init();

    xTaskCreate(nimble_host_task, "nimble_host", 4 * 1024, NULL, 5, NULL);
}
