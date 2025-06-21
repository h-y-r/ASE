#include <stdio.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/gptimer.h"		// timer
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <unistd.h>
#include "esp_adc/adc_oneshot.h"
#include "driver/mcpwm.h"		// biblioteka silników
#include "soc/mcpwm_periph.h"	// -//-
#include <string.h>
#include "driver/pulse_cnt.h"	// licznik impulsów do czujnika szczelinowego
#include <sys/time.h>			// ustawianie czasu rtc
#include <time.h>				// zliczanie czasu z rtc

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdbool.h>

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

#define TRIGGER0 18		// czujnik lewy
#define TRIGGER1 15		// czujnik lewy przedni
#define TRIGGER2 12		// czujnik przedni środkowy
#define TRIGGER3 1		// czujnik prawy przedni
#define TRIGGER4 4		// czujnik prawy
#define ECHO0 33
#define ECHO1 16
#define ECHO2 13
#define ECHO3 9
#define ECHO4 5
#define ODB0_GPIO 34      			// czujnik lewy
#define ODB1_GPIO 17      			// czujnik lewy przedni
#define ODB2_GPIO 14      			// czujnik przedni środkowy
#define ODB3_GPIO 11      			// czujnik prawy przedni
#define ODB4_GPIO 2      			// czujnik prawy
#define ODB_KAT_GPIO 10 			// gpio dla czujnika odbiciowego katowego
#define HALL_GPIO 8    				// gpio dla czujnika halla

#define OUTPUT_PINS (1ULL << TRIGGER0) | (1ULL << TRIGGER1) | (1ULL << TRIGGER2) | (1ULL << TRIGGER3) | (1ULL << TRIGGER4)
#define INPUT_PINS (1ULL << ECHO0) | (1ULL << ECHO1) | (1ULL << ECHO2) | (1ULL << ECHO3) | (1ULL << ECHO4)

#define PWM_MOST_IN1 40
#define PWM_MOST_IN2 39
#define PWM_MOST_IN3 38
#define PWM_MOST_IN4 37
#define PWM_FREQ 1000 // 1 kHz

#define SZCZEL_GPIO 41 				// gpio dla czujnika szczelinowego (na esp devkitv1 nie dziala dla gpio18)
#define PCNT_LOW_LIMIT  -1
#define PCNT_HIGH_LIMIT 10240 // liczy do 5120 cm, czyli 51 m
float travelled_distance;

bool start_driving = 0;
time_t driving_time;
	
#define INA219_ADDR 0x40
#define INA219_CONFIG_REG_ADDR 0
#define INA219_SHUNTVOLTAGE_REG_ADDR 1
#define INA219_BUSVOLTAGE_REG_ADDR 2
#define INA219_POWER_REG_ADDR 3
#define INA219_CURRENT_REG_ADDR 4
#define INA219_CALIBRATION_REG_ADDR 5
#define INA219_READ_INTERVAL_MS 100
#define INA219_TIMEOUT_MS 20
#define INA219_CONFIG_MSB 0b00111111
#define INA219_CONFIG_LSB 0b11111111

// Wartość kalibracji monitora zasilania dla zakresu 32 V i 2 A:
#define INA219_CALIBRATION_MSB 0b0010000
#define INA219_CALIBRATION_LSB 0b00000000

#define DELAY_MS 1000    			//czas miedzy pomiarami

volatile uint64_t Echo_timestamp = 0;
volatile uint64_t Echo_timestamp_rising = 0;

int16_t Bus_voltage_raw = 0;
int16_t Current_draw_raw = 0;
int16_t Average_power_raw = 0;

// Napięcie w miliwoltach:
uint32_t Bus_voltage = 0;

// Natężenie prądu w miliamperach:
int32_t Current_draw = 0;

// Moc w miliwatach:
uint32_t Average_power = 0;

// Zużyta energia w milidżulach:
uint32_t Total_energy = 0.0;

typedef enum {
    SENSOR_0,
    SENSOR_1,
    SENSOR_2,
    SENSOR_3,
    SENSOR_4,
    NUMBER_OF_SENSORS
} SensorNumber;

typedef struct Ultrasonic_data_struct {
    uint32_t Current_sensor;
    uint64_t Distance[5];
    gpio_num_t Echo_pins[5];
    gpio_num_t Trigger_pins[5];
    gptimer_handle_t Trigger_timer;
    gptimer_handle_t Ultrasonic_timer;
} Ultrasonic_data_t;

// Funkcja obsługi przerwania licznika, rozpoczynająca impuls "trigger" czujnika ultradźwiękowego:
static bool Periodic_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *Data_struct) {
    gpio_set_level(((Ultrasonic_data_t*)Data_struct)->Trigger_pins[((Ultrasonic_data_t*)Data_struct)->Current_sensor], 1);
    gptimer_stop(((Ultrasonic_data_t*)Data_struct)->Trigger_timer);
    gptimer_set_raw_count(((Ultrasonic_data_t*)Data_struct)->Trigger_timer, 0ULL);
    gptimer_start(((Ultrasonic_data_t*)Data_struct)->Trigger_timer);
    return 0;
}

// Funkcja obsługi przerwania licznika kończąca impuls "trigger" czujnika ultradźwiękowego:
static bool Oneshot_handler(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *Data_struct) {
    gpio_set_level(((Ultrasonic_data_t*)Data_struct)->Trigger_pins[((Ultrasonic_data_t*)Data_struct)->Current_sensor], 0);
    return 0;
}

// Funkcja obsługi przerwania pinów IO, pozwalająca na pomiar czasu trwania impulsu "echo" i zmianę czujnika:
static void IO_Handler(void *Data_struct) {
    gptimer_get_raw_count(((Ultrasonic_data_t*)Data_struct)->Ultrasonic_timer, &Echo_timestamp);
    if(gpio_get_level(((Ultrasonic_data_t*)Data_struct)->Echo_pins[((Ultrasonic_data_t*)Data_struct)->Current_sensor]))
    {
        Echo_timestamp_rising = Echo_timestamp;
    }
    else
    {
            ((Ultrasonic_data_t*)Data_struct)->Distance[((Ultrasonic_data_t*)Data_struct)->Current_sensor] = Echo_timestamp - Echo_timestamp_rising;
            if(((Ultrasonic_data_t*)Data_struct)->Current_sensor == 4)
            {
                ((Ultrasonic_data_t*)Data_struct)->Current_sensor = 0;
            }
            else
            {
                (((Ultrasonic_data_t*)Data_struct)->Current_sensor)++;
            }
    }
}

void I2C_Task(void *pvParameter)
{
    uint8_t I2C_TX_buffer[3];
    uint8_t I2C_RX_buffer[2];

    // Inicjalizacja kontrolera I2C:
    i2c_master_bus_handle_t I2C_Bus_handle = NULL;
    i2c_master_bus_config_t I2C_Master_config = {
        .i2c_port = -1,
        .sda_io_num = 6,
        .scl_io_num = 7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .intr_priority = 0,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&I2C_Master_config, &I2C_Bus_handle);

    i2c_master_dev_handle_t I2C_Device_handle = NULL;
    i2c_device_config_t I2C_Device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(I2C_Bus_handle, &I2C_Device_config, &I2C_Device_handle);

    // Konfiguracja układu INA219:
    I2C_TX_buffer[0] = INA219_CONFIG_REG_ADDR;
    I2C_TX_buffer[1] = INA219_CONFIG_MSB;
    I2C_TX_buffer[2] = INA219_CONFIG_LSB;
    i2c_master_transmit(I2C_Device_handle, I2C_TX_buffer, 3, INA219_TIMEOUT_MS);

    I2C_TX_buffer[0] = INA219_CALIBRATION_REG_ADDR;
    I2C_TX_buffer[1] = INA219_CALIBRATION_MSB;
    I2C_TX_buffer[2] = INA219_CALIBRATION_LSB;
    i2c_master_transmit(I2C_Device_handle, I2C_TX_buffer, 3, INA219_TIMEOUT_MS);

    vTaskDelay(INA219_READ_INTERVAL_MS / portTICK_PERIOD_MS);

    while(1)
    {
        // Odczyt średniej mocy pobieranej przez system:
        I2C_TX_buffer[0] = INA219_POWER_REG_ADDR;
        i2c_master_transmit(I2C_Device_handle, I2C_TX_buffer, 1, INA219_TIMEOUT_MS);
        i2c_master_receive(I2C_Device_handle, I2C_RX_buffer, 2, INA219_TIMEOUT_MS);
        Average_power_raw = ((I2C_RX_buffer[0]) << 8) | (I2C_RX_buffer[1]);

        // Odczyt napięcia zasilania systemu:
        I2C_TX_buffer[0] = INA219_BUSVOLTAGE_REG_ADDR;
        i2c_master_transmit(I2C_Device_handle, I2C_TX_buffer, 1, INA219_TIMEOUT_MS);
        i2c_master_receive(I2C_Device_handle, I2C_RX_buffer, 2, INA219_TIMEOUT_MS);
        Bus_voltage_raw = (((I2C_RX_buffer[0]) << 8) | (I2C_RX_buffer[1])) >> 3;

        // Odczyt średniego natężenia prądu zasilania:
        I2C_TX_buffer[0] = INA219_CURRENT_REG_ADDR;
        i2c_master_transmit(I2C_Device_handle, I2C_TX_buffer, 1, INA219_TIMEOUT_MS);
        i2c_master_receive(I2C_Device_handle, I2C_RX_buffer, 2, INA219_TIMEOUT_MS);
        Current_draw_raw = ((I2C_RX_buffer[0]) << 8) | (I2C_RX_buffer[1]);

        Average_power = Average_power_raw * 2;
        Bus_voltage = Bus_voltage_raw * 4;
        Current_draw = Current_draw_raw / 10;

        // Aktualizacja wartości zużytej dotąd energii (całkowanie metodą prostokątów z wartościowaniem dla górnej granicy przedziału):
        Total_energy = Total_energy + Average_power * INA219_READ_INTERVAL_MS / 1000;
        vTaskDelay(INA219_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

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
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_1);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
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

static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callback
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
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

bool BTH_manual_control_used = 0;
bool BTH_do_it_one_time = 0;
char BTH_om_prev_state[6] = "";
char BTH_om_data_holder[6] = ""; 
static int uart_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) 
{
    const struct os_mbuf *om = ctxt->om;
    char om_str[om->om_len + 1];
    memcpy(om_str, om->om_data, om->om_len);
    om_str[om->om_len] = '\0';
    if(strcmp(om_str, BTH_om_prev_state) != 0)
    {
		BTH_do_it_one_time = 0;
	}
	else if(BTH_do_it_one_time)
	{
		memcpy(om_str, BTH_om_prev_state, strlen(om_str));
	}
	else 
	{
		ESP_LOGE(TAG, "uart_rx_cb -> warunek strcmp(om_str, BTH_om_prev_state) NIE DZIALA");
	}
    
    if(strcmp(om_str, "start") == 0 && BTH_do_it_one_time != 1)
    {
		start_driving = 1;
		ble_uart_send(&"Rozpoczecie jazdy", strlen("Rozpoczecie jazdy"));
		
		BTH_do_it_one_time = 1;
	}
    if(strcmp(om_str, "mc") == 0 && BTH_do_it_one_time != 1) //manual_control (wymagane włączenie tego do ręczego kierowania pojazdem)
    {
		BTH_manual_control_used = !BTH_manual_control_used;
		if (BTH_manual_control_used)
		{
			ble_uart_send(&"Sterujesz pojazdem", strlen("Sterujesz pojazdem"));
		}
		else 
		{	
			ble_uart_send(&"Nie sterujesz pojazdem", strlen("Nie sterujesz pojazdem"));
		}
		
		BTH_do_it_one_time = 1;
	}
    if(strcmp(om_str, "w") == 0 && BTH_manual_control_used)
    {
		sterowanie_silnikami("przod", 50);
		ble_uart_send(&"Jedzie do przodu", strlen("Jedzie do przodu"));
	}
	else if(strcmp(om_str, "s") == 0 && BTH_manual_control_used)
    {
		sterowanie_silnikami("tyl", 50);
		ble_uart_send(&"Jedzie do tylu", strlen("Jedzie do tylu"));
	}
	else if(strcmp(om_str, "a") == 0 && BTH_manual_control_used)
    {
		sterowanie_silnikami("lewo", 50);
		ble_uart_send(&"Skrecil w lewo", strlen("Skrecil w lewo"));
	}
	else if(strcmp(om_str, "d") == 0 && BTH_manual_control_used)
    {
		sterowanie_silnikami("prawo", 50);
		ble_uart_send(&"Skrecil w prawo", strlen("Skrecil w prawo"));
	}
	else if((strcmp(om_str, "ws") == 0 || strcmp(om_str, "stop") == 0 || strcmp(om_str, "x") == 0) && BTH_manual_control_used)
    {
		sterowanie_silnikami("stop", 50);
		ble_uart_send(&"Zatrzymal sie", strlen("Zatrzymal sie"));
		BTH_manual_control_used = 1;
	}
	else if(strcmp(om_str, "param") == 0 && BTH_do_it_one_time != 1)
    {	
		char driving_time_str[10];
		char travelled_distance_str[10];
	    char Total_energy_str[10];
	    //sprintf(driving_time_str, "%02lld:%02lld", (driving_time/60), (driving_time%60));
	    sprintf(driving_time_str, "%02d:%02d", (int)(driving_time/60), (int)(driving_time%60));
	    sprintf(travelled_distance_str, "%d.%d", (int)(travelled_distance), ((int)((travelled_distance)*10))%10);
	    sprintf(Total_energy_str, "%lu", Total_energy);
	    ble_uart_send(&"Czas przejazdu:", strlen("Czas przejazdu:"));
    	ble_uart_send(&driving_time_str, strlen(driving_time_str));
	    ble_uart_send(&"Przemierzony dystans:", strlen("Przemierzony dystans:"));
    	ble_uart_send(&travelled_distance_str, strlen(travelled_distance_str));
	    ble_uart_send(&"Zuzycie energii:", strlen("Zuzycie energii:")); // nrf nie pokazuje polskich znaków
    	ble_uart_send(&Total_energy_str, strlen(Total_energy_str));
    	
		BTH_do_it_one_time = 1;      // jeśli to usuniesz to będzie się towykonywało z każdym rozpoczęciem pętli app_main
	}
	
	if(!BTH_do_it_one_time)
	{
		memcpy(BTH_om_prev_state, om_str, strlen(om_str));
	}
	
    
    //ESP_LOGI(TAG, "Received from phone: %s", om_str);
    //ble_uart_send(om->om_data, om->om_len);
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

//char mpc_result[38] = ""; // stara wersja łańcucha
char mpc_result1[26] = "";
char mpc_result2[21] = "";
void make_param_chain(time_t time, float distance, uint32_t energy)//przekonwertuj liczby na chary i pododawaj
{
	int start_number = 33; 
	int time_min = ((int)time)/60; //60 - max możliwa liczba
	int time_s = ((int)time)%60; //60
	int distance_int = distance; //5120
	int distance_quan = ((int)((distance)*10))%10; //5 //założyłem 0,5 cm ale to na oko
	int ener = energy; //9999 // nwm ile charów zajmie ta zmienna
	int control_number = time_min + time_s + distance_int + distance_quan + ener;
	int end_number = 55;
	//sprintf(mpc_result, "%d;%02d:%02d;%04d.%d;%04d;%05d;%d", start_number, time_min, time_s, distance_int, distance_quan, ener, control_number, end_number);
	sprintf(mpc_result1, "%d;%02d:%02d;%04d.%d;", start_number, time_min, time_s, distance_int, distance_quan);
	sprintf(mpc_result2, "%08d;%08d;%d", ener, control_number, end_number); // to 8 znaków dla energii jest tak na próbę
	
	printf("%s\n", mpc_result1); // wyświetlanie łacucha, żebyś widział aktualny wygląd łańcucha
	printf("%s\n", mpc_result2); 
	ble_uart_send(&mpc_result1, strlen(mpc_result1)); // ta komeda (ble_uart_send) wysyła tylko 20 znaków przez co musiałem podzielić łańcuch na dwie części
	ble_uart_send(&mpc_result2, strlen(mpc_result2)); 
}

void RTC_reset()
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	settimeofday(&tv, NULL);
}



void app_main()
{
	// musiałem to zakomentować bo jak nie jest nic podłączone pod piny do i2c to cała konsola jest zawalona błędami
    TaskHandle_t I2C_Task_handle = NULL;
    xTaskCreatePinnedToCore(I2C_Task, "I2C Task", 4096, NULL, 1, &I2C_Task_handle, tskNO_AFFINITY);
    
	// Inicjalizacja adc dla czujnika halla	
    adc_oneshot_unit_handle_t hall_handle;
	adc_unit_t hall_adc_unit;
	adc_channel_t hall_channel; 
	adc_oneshot_io_to_channel(HALL_GPIO, &hall_adc_unit, &hall_channel);
    adc_oneshot_unit_init_cfg_t hall_init_cfg = {
	    .unit_id = hall_adc_unit,
	    .ulp_mode = ADC_ULP_MODE_DISABLE
	};
	adc_oneshot_new_unit(&hall_init_cfg, &hall_handle);
	adc_oneshot_chan_cfg_t hall_chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,       // domyślnie 12-bit
        .atten = ADC_ATTEN_DB_12                // pełny zakres (do ~3.3V)
    };
	adc_oneshot_config_channel(hall_handle, hall_channel, &hall_chan_cfg);

    // Deklaracja struktury z danymi dotyczącymi czujników ultradźwiękowych:
    Ultrasonic_data_t Ultrasonic_data = {0, {0, 0, 0, 0, 0}, {ECHO0, ECHO1, ECHO2, ECHO3, ECHO4}, {TRIGGER0, TRIGGER1, TRIGGER2, TRIGGER3, TRIGGER4}, NULL, NULL};

    // Konfiguracja licznika wywołującego pomiar czujnikiem ultradźwiękowym:
    gptimer_handle_t Periodic_timer = NULL;
    gptimer_config_t Periodic_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
    };
    gptimer_new_timer(&Periodic_timer_config, &Periodic_timer);
    gptimer_alarm_config_t Periodic_timer_alarm_config = {
        .alarm_count = 100000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_event_callbacks_t Periodic_timer_callback = {
        .on_alarm = Periodic_handler,
    };
    gptimer_set_alarm_action(Periodic_timer, &Periodic_timer_alarm_config);
    gptimer_register_event_callbacks(Periodic_timer, &Periodic_timer_callback, &Ultrasonic_data);
    gptimer_enable(Periodic_timer);

    // Konfiguracja licznika sterującego pinem "trigger" czujnika ultradźwiękowego:
    gptimer_handle_t Oneshot_timer = NULL;
    gptimer_config_t Oneshot_timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
        .intr_priority = 0,
    };
    gptimer_new_timer(&Oneshot_timer_config, &Oneshot_timer);
    gptimer_alarm_config_t Oneshot_timer_alarm_config = {
        .alarm_count = 1000,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false,
    };
    gptimer_event_callbacks_t Oneshot_timer_callback = {
        .on_alarm = Oneshot_handler,
    };
    gptimer_set_alarm_action(Oneshot_timer, &Oneshot_timer_alarm_config);
    gptimer_register_event_callbacks(Oneshot_timer, &Oneshot_timer_callback, &Ultrasonic_data);
    gptimer_enable(Oneshot_timer);
    
    Ultrasonic_data.Trigger_timer = Oneshot_timer;
    Ultrasonic_data.Ultrasonic_timer = Periodic_timer;

    gpio_config_t IO_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = OUTPUT_PINS,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&IO_config);

    IO_config.intr_type = GPIO_INTR_ANYEDGE;
    IO_config.pin_bit_mask = INPUT_PINS;
    IO_config.mode = GPIO_MODE_INPUT;
    gpio_config(&IO_config);

        // Konfiguracja filtrów wejść "echo":
        gpio_pin_glitch_filter_config_t Filter_config = {
            .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        };
        gpio_glitch_filter_handle_t Filters_array[5] = {NULL, NULL, NULL, NULL, NULL};
        for(uint32_t i = 0; i < 5; i++)
        {
            Filter_config.gpio_num = Ultrasonic_data.Echo_pins[i];
            gpio_new_pin_glitch_filter(&Filter_config, &Filters_array[i]);
            gpio_glitch_filter_enable(Filters_array[i]);
        }
    
        // Dołączanie przerwań do pinów "echo":
        gpio_install_isr_service(0);
        for(uint32_t i = 0; i < 5; i++)
        {
            gpio_isr_handler_add(Ultrasonic_data.Echo_pins[i], IO_Handler, &Ultrasonic_data);
        }
    
        // Uruchomienie licznika wywołującego pomiar czujnikiem ultradźwiękowym:
        gptimer_start(Periodic_timer);
        
        // inicjalizacja silników
        mcpwm_init_mostek_h();
        
        // czujnik szczelinowy pcnt - init
	    pcnt_unit_config_t unit_config = {
	        .high_limit = PCNT_HIGH_LIMIT,
	        .low_limit = PCNT_LOW_LIMIT,
	    };
	    pcnt_unit_handle_t pcnt_unit = NULL;
	    pcnt_new_unit(&unit_config, &pcnt_unit);
	
	    pcnt_glitch_filter_config_t filter_config = {
	        .max_glitch_ns = 1000,
	    };
	    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);
	
	    // init - inicjalizacja pinów
	    pcnt_chan_config_t chan_a_config = {
	        .edge_gpio_num = SZCZEL_GPIO,
	        .level_gpio_num = -1,
	    };
	    pcnt_channel_handle_t pcnt_chan_a = NULL;
	    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);
	
	    // ustawienie rackcji na dane zbocze (aktualnie - zbocze opadające - inkrementacja i zbocze narastające - nic)
	    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
	
	    // tworzenie kolejki i przerwania
	    pcnt_event_callbacks_t cbs = {
	        .on_reach = pcnt_on_reach,
	    };
	    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
	    pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue);
	
	    pcnt_unit_enable(pcnt_unit);
	    pcnt_unit_clear_count(pcnt_unit);
	    pcnt_unit_start(pcnt_unit);
    
    	int szczel_state;
        int szczel_range_event;
        
        //bth-ble init
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
        
        // szukanie linii startu
        sterowanie_silnikami("przod", 50);
		while(start_driving == 0)
		{
			// mój pomysł był taki, że jedzie do przodu i ciągle spradza, czy coś wykrył na czujniku kątowym, a jak znajdzie to zeruje licznik czasu przejazdu
			if(gpio_get_level(ODB_KAT_GPIO))
			{
				start_driving = 1;
			}
		}
		RTC_reset();
			
        while(1)
        {	
			driving_time = time(NULL);
			
            bool odb_state[5] = {gpio_get_level(ODB0_GPIO), gpio_get_level(ODB1_GPIO), gpio_get_level(ODB2_GPIO), gpio_get_level(ODB3_GPIO), gpio_get_level(ODB4_GPIO)};
            bool odb_kat_state = gpio_get_level(ODB_KAT_GPIO);
            int hall_adc_reading = 0;
            adc_oneshot_read(hall_handle, hall_channel, &hall_adc_reading);
            bool hall_state = 0;
            hall_adc_reading -= 2000;
            if(hall_adc_reading < -11 || hall_adc_reading > 11)
            {
				hall_state = 1;
			}
			else
			{
				hall_state = 0;
			}
			
            double distance_cm[5];
            for(uint8_t i = 0; i < NUMBER_OF_SENSORS; ++i) 
            {
        		distance_cm[i] = Ultrasonic_data.Distance[i] / 58.0;
        	}

            printf("========================\n");
            for(uint8_t i = 0; i < NUMBER_OF_SENSORS; ++i) 
            {
        		printf("Czujnik ultra %d: %f cm\n", i, distance_cm[i]);
        	}
            //printf("Czujnik %lu:%llu\n", Ultrasonic_data.Current_sensor, Ultrasonic_data.Distance[Ultrasonic_data.Current_sensor]);
	        for(int i = 0; i<NUMBER_OF_SENSORS; ++i)
	        {
				printf("Czujnik odbiciowy %d: %d\n", i, odb_state[i]);
			}
            printf("Czujnik odbiciowy katowy: %d\n", odb_kat_state);
            //printf("Czujnik halla: %d\n", hall_adc_reading);
            printf("Czujnik halla: %d\n", hall_state);
            
            if (xQueueReceive(queue, &szczel_range_event, 50 / portTICK_PERIOD_MS)) {
            printf("Granica zakresu: %d\n", szczel_range_event);
	        } else {
	            pcnt_unit_get_count(pcnt_unit, &szczel_state);
	            //printf("Czujnik szczelinowy: %d\n", szczel_state);
	            travelled_distance = szczel_state*0.5; // przyjąłem odległość między szczytami ząbków jako 0,5 cm, ale na razie na oko
	            printf("Przemierzony dystans: %f cm\n", travelled_distance);
	        }
            printf("Czas przejazdu: %02lld:%02lld\n", driving_time/60, driving_time%60);
            printf("Napiecie: %lu\n", Bus_voltage);
            printf("Prad: %ld\n", Current_draw);
            printf("Moc: %lu\n", Average_power);
            printf("Energia: %lu\n", Total_energy);
            
            // zabezpieczenie, żeby podczas manualnego sterowania pojazdem pojazd nie wjechał w przeszkodę
            if(BTH_manual_control_used)
            {
				// warunek dla przednich czujników
				for(uint8_t i = 1; i < NUMBER_OF_SENSORS-1; ++i) 
	            {
	        		if(distance_cm[i] < 17.5 || odb_state[i])
	        		{
						sterowanie_silnikami("stop", 2137);
						ble_uart_send(&"Przeszkoda z przodu", strlen("Przeszkoda z przodu"));
						
					}
	        	}
	        	for(uint8_t i = 0; i < 2; ++i) 
	            {
					// warunek dla czujników bocznych
	        		if(distance_cm[i*4] < 10 || odb_state[i*4])
	        		{
						sterowanie_silnikami("stop", 2137);
						ble_uart_send(&"Przeszkoda z boku", strlen("Przeszkoda z boku"));
					}
	        	}
			}
			
			// łączenie parametrów w łańcuch danych i wysyłanie nadawanie go bluetoothem
			make_param_chain(driving_time, travelled_distance, Total_energy);
			
			
			
            
            
            // test silników
//            sterowanie_silnikami("przod", 50);
//            vTaskDelay(200 / portTICK_PERIOD_MS);
//            sterowanie_silnikami("tyl", 50);
//            vTaskDelay(200 / portTICK_PERIOD_MS);
//            sterowanie_silnikami("lewo", 50);
//            vTaskDelay(200 / portTICK_PERIOD_MS);
//            sterowanie_silnikami("prawo", 50);
//            vTaskDelay(200 / portTICK_PERIOD_MS);
//            sterowanie_silnikami("stop", 2137); //tu wypełnienie nie ma znaczenia
//            vTaskDelay(200 / portTICK_PERIOD_MS);
            
            vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
        }
}
