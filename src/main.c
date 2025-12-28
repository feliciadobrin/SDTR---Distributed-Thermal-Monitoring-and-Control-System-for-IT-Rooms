#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()
#include "esp_err.h"

#include "ble_node.h"

/* Tag-uri pentru log */
static const char *TAG_APP  = "APP";
static const char *TAG_RT   = "RT_TASK";
static const char *TAG_COMM = "COMMS_TASK";

/* Pinuri hardware */
#define LED_GPIO      GPIO_NUM_25   // LED-ul deja conectat
#define BUZZER_GPIO   GPIO_NUM_26   // Buzzer activ
#define FAN_GPIO      GPIO_NUM_27   // Releu ventilator (IN)

/* Pin DS18B20 (DATA) */
#define DS18B20_GPIO  GPIO_NUM_4    // senzor


/* Limitele plauzibile pentru camera IT */
#define TEMP_MIN_VALID   0.0f    // nu ne așteptăm la sub 0°C în cameră
#define TEMP_MAX_VALID   60.0f   // peste 60°C considerăm eroare de citire


/* Structura mesajului trimis prin coada de telemetrie */
typedef struct
{
    float    temperature;     // temperatura reala (de la senzor)
    int      fan_state;       // 0 = OFF, 1 = ON
    uint32_t timestamp_ms;    // momentul masurarii (ms de la boot)
    int      alarm;           // 0 = normal, 1 = peste prag critic
} TelemetryMessage;

/* Coada globala */
static QueueHandle_t queue_telemetry = NULL;

/* Prag & histerezis */
static float g_temp_threshold  = 30.0f;  // prag de pornire ventilator (LED + FAN)
static float g_temp_hysteresis = 2.0f;   // histerezis
static float g_alarm_threshold = 35.0f;  // prag de alarma (BUZZER)

/* ========================================================================== */
/*                       UTILITARE DS18B20 / 1-WIRE                           */
/* ========================================================================== */

static void ds18b20_set_output(void)
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
}

static void ds18b20_set_input(void)
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
}

static void ds18b20_write_level(int level)
{
    gpio_set_level(DS18B20_GPIO, level);
}

/* Resetul 1-Wire, intoarce true daca senzorul raspunde (presence pulse) */
static bool ds18b20_reset_pulse(void)
{
    ds18b20_set_output();
    ds18b20_write_level(0);
    esp_rom_delay_us(480);          // tinem linia low ~480us

    ds18b20_set_input();            // eliberam linia
    esp_rom_delay_us(70);           // asteptam ~70us

    int presence = (gpio_get_level(DS18B20_GPIO) == 0);  // senzorul trage linia LOW
    esp_rom_delay_us(410);          // asteptam restul slotului

    if (!presence) {
        ESP_LOGW("DS18B20", "No presence pulse detected!");
    }
    return presence;
}

/* Scrie un bit pe bus 1-Wire */
static void ds18b20_write_bit(int bit)
{
    ds18b20_set_output();
    ds18b20_write_level(0);      // incepem cu LOW

    if (bit) {
        // scriem "1"
        esp_rom_delay_us(10);
        ds18b20_set_input();    // eliberam linia, senzor vede '1'
        esp_rom_delay_us(55);
    } else {
        // scriem "0"
        esp_rom_delay_us(65);   // linia e low aproape tot slotul
        ds18b20_set_input();
        esp_rom_delay_us(5);
    }
}

/* Citeste un bit de la DS18B20 */
static int ds18b20_read_bit(void)
{
    int bit;

    ds18b20_set_output();
    ds18b20_write_level(0);      // incepem cu un puls scurt LOW
    esp_rom_delay_us(3);

    ds18b20_set_input();         // eliberam linia
    esp_rom_delay_us(10);        // asteptam putin

    bit = gpio_get_level(DS18B20_GPIO);  // citim nivelul
    esp_rom_delay_us(50);        // asteptam restul slotului

    return bit & 0x1;
}

/* Scrie un byte (LSB first) */
static void ds18b20_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/* Citeste un byte (LSB first) */
static uint8_t ds18b20_read_byte(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++) {
        int bit = ds18b20_read_bit();
        value |= (bit << i);
    }
    return value;
}

/* Citeste temperatura in grade Celsius. Intoarce true daca reuseste. */
static bool ds18b20_read_temperature(float *out_temp_c)
{
    if (!ds18b20_reset_pulse()) {
        return false;
    }

    // SKIP ROM (0xCC) – presupunem un singur senzor pe bus
    ds18b20_write_byte(0xCC);
    // CONVERT T (0x44)
    ds18b20_write_byte(0x44);

    // Asteptam conversia (max ~750ms pentru rezolutie 12 bit)
    vTaskDelay(pdMS_TO_TICKS(750));

    if (!ds18b20_reset_pulse()) {
        return false;
    }

    ds18b20_write_byte(0xCC);   // SKIP ROM
    ds18b20_write_byte(0xBE);   // READ SCRATCHPAD

    uint8_t temp_lsb = ds18b20_read_byte();
    uint8_t temp_msb = ds18b20_read_byte();

    int16_t raw = (int16_t)((temp_msb << 8) | temp_lsb);

    // Pas de 1/16 °C la rezolutie 12-bit
    float temp_c = (float)raw / 16.0f;

    *out_temp_c = temp_c;
    return true;
}

/* ========================================================================== */
/*                  Task 1: proces de timp real (senzor + control)            */
/* ========================================================================== */

static void sensor_control_task(void *pvParameters)
{
    ESP_LOGI(TAG_RT, "RT_SensorControlTask started");

    float current_temp = 25.0f;   // valoare inițială
    int   fan_state    = 0;       // 0 = OFF, 1 = ON

    const TickType_t period = pdMS_TO_TICKS(1000);  // perioada nominală: 1 s
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        float temp_c;
        bool ok = ds18b20_read_temperature(&temp_c);

        if (!ok) {
            // nu schimbăm current_temp, folosim ultima valoare validă
            ESP_LOGW(TAG_RT,
                     "DS18B20 read failed, folosim ultima valoare: %.2f",
                     current_temp);
        } else {
            // filtrăm valori aberante (ex: 534°C)
            if (temp_c < TEMP_MIN_VALID || temp_c > TEMP_MAX_VALID) {
                ESP_LOGW(TAG_RT,
                         "DS18B20 valoare out-of-range: %.2f, pastrez: %.2f",
                         temp_c, current_temp);
            } else {
                current_temp = temp_c;
            }
        }

        /* Logica de control cu prag + histerezis pentru ventilator */
        if (fan_state == 0 && current_temp >= g_temp_threshold) {
            fan_state = 1;   // pornim ventilator => LED & FAN ON
        } else if (fan_state == 1 &&
                   current_temp <= (g_temp_threshold - g_temp_hysteresis)) {
            fan_state = 0;   // oprim ventilator => LED & FAN OFF
        }

        /* Alarma: când temperatura depășește pragul critic (35°C) */
        int alarm = (current_temp >= g_alarm_threshold) ? 1 : 0;

        /* LED = starea ventilatorului */
        gpio_set_level(LED_GPIO, fan_state);

        /* Ventilator (releu)
           Dacă releul este activ-low, folosește:
           gpio_set_level(FAN_GPIO, fan_state ? 0 : 1);
        */
        gpio_set_level(FAN_GPIO, fan_state);

        /* Buzzer-ul: ON când avem alarmă */
        gpio_set_level(BUZZER_GPIO, alarm);

        /* Actualizăm BLE cu temperatura filtrată */
        ble_node_update_temperature(current_temp);

        /* Trimitem mesajul de telemetrie în coadă */
        TelemetryMessage msg;
        msg.temperature  = current_temp;
        msg.fan_state    = fan_state;
        msg.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        msg.alarm        = alarm;

        if (queue_telemetry != NULL) {
            BaseType_t okq = xQueueSend(queue_telemetry, &msg, pdMS_TO_TICKS(10));
            if (okq != pdPASS) {
                ESP_LOGW(TAG_RT, "Queue full, could not send telemetry");
            }
        }

        ESP_LOGI(TAG_RT,
                 "Temp=%.2f C, fan=%d, alarm=%d",
                 current_temp, fan_state, alarm);

        /* Intârziere de frecvență fixă (RTOS-style) */
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/* ========================================================================== */
/*                 Task 2: proces de comunicatie / raportare                  */
/* ========================================================================== */

static void comms_task(void *pvParameters)
{
    ESP_LOGI(TAG_COMM, "CommsAndReportingTask started");

    TelemetryMessage rx_msg;

    while (1)
    {
        if (queue_telemetry != NULL) {
            BaseType_t ok = xQueueReceive(queue_telemetry, &rx_msg, portMAX_DELAY);
            if (ok == pdPASS) {
                ESP_LOGI(TAG_COMM,
                         "Received: Temp=%.2f C, fan=%d, alarm=%d, t=%u ms",
                         rx_msg.temperature,
                         rx_msg.fan_state,
                         rx_msg.alarm,
                         (unsigned int)rx_msg.timestamp_ms);
            }
        }
    }
}

/* ========================================================================== */
/*                          Punctul de intrare al aplicatiei                  */
/* ========================================================================== */

void app_main(void)
{
    ESP_LOGI(TAG_APP, "app_main started");

    /* 1) Pornim BLE (serviciu + caracteristica temperatura) */
    esp_err_t ret = ble_node_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_APP, "BLE init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG_APP, "BLE init OK");
    }

    /* 2) Configuram pinii GPIO pentru LED / BUZZER / FAN */
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);   // initial LED OFF

    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0); // initial Buzzer OFF

    gpio_reset_pin(FAN_GPIO);
    gpio_set_direction(FAN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FAN_GPIO, 0);   // initial FAN OFF (daca releu activ HIGH)

    /* 3) Configuram pinul pentru DS18B20 (lasam pull-up extern de 4.7k la 3.3V) */
    gpio_reset_pin(DS18B20_GPIO);
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_DISABLE);
    ds18b20_set_input();  // initial eliberam linia

    /* 4) Cream coada de telemetrie (maxim 5 mesaje in coada) */
    queue_telemetry = xQueueCreate(5, sizeof(TelemetryMessage));
    if (queue_telemetry == NULL) {
        ESP_LOGE(TAG_APP, "Failed to create telemetry queue");
        return;
    }

    /* 5) Cream Task 1 (timp real) – prioritate mai mare */
    BaseType_t res1 = xTaskCreate(
        sensor_control_task,
        "RT_SensorControlTask",
        4096,
        NULL,
        5,        // prioritate RT
        NULL
    );

    /* 6) Cream Task 2 (comunicatie) – prioritate mai mica */
    BaseType_t res2 = xTaskCreate(
        comms_task,
        "CommsAndReportingTask",
        4096,
        NULL,
        4,        // cu o treapta sub RT
        NULL
    );

    if (res1 != pdPASS || res2 != pdPASS) {
        ESP_LOGE(TAG_APP, "Failed to create tasks");
    }
}
