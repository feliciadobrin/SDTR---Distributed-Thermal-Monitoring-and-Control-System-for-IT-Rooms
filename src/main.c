#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ble_node.h"

/* Tag-uri pentru log */
static const char *TAG_APP  = "APP";
static const char *TAG_RT   = "RT_TASK";
static const char *TAG_COMM = "COMMS_TASK";

/* Pinuri hardware */
#define LED_GPIO     GPIO_NUM_25   // LED-ul deja conectat
#define BUZZER_GPIO  GPIO_NUM_26   // Buzzer activ
#define FAN_GPIO     GPIO_NUM_27   // Releu ventilator (IN)

/* Structura mesajului trimis prin coada de telemetrie */
typedef struct
{
    float    temperature;     // temperatura (simulata)
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

/* ---------------- Task 1: proces de timp real (simulare temp + control) ---------------- */
static void sensor_control_task(void *pvParameters)
{
    ESP_LOGI(TAG_RT, "RT_SensorControlTask started");

    float current_temp = 25.0f;   // temperatura simulata
    int   fan_state    = 0;       // 0 = OFF, 1 = ON

    const TickType_t period = pdMS_TO_TICKS(1000);  // perioada: 1 secunda
    TickType_t last_wake_time = xTaskGetTickCount();

    int phase   = 0;   // 0 = 25C, 1 = 35C
    int counter = 0;   // numara secundele in fiecare faza (0..29)

    while (1)
    {
        /* Alegem temperatura in functie de faza curenta */
        if (phase == 0) {
            current_temp = 25.0f;   // sub prag, ventilator OFF
        } else {
            current_temp = 35.0f;   // peste prag, ventilator ON + alarma
        }

        /* Contor pentru a schimba faza la fiecare 30 secunde */
        counter++;
        if (counter >= 30) {
            counter = 0;
            phase = 1 - phase;  // comutam intre 0 si 1
            ESP_LOGI(TAG_RT, "Switching phase to %d (0=25C, 1=35C)", phase);
        }

        /* Logica de control cu prag + histerezis pentru fan_state */
        if (fan_state == 0 && current_temp >= g_temp_threshold) {
            fan_state = 1;   // "pornim ventilator" => LED & FAN ON
        } else if (fan_state == 1 && current_temp <= (g_temp_threshold - g_temp_hysteresis)) {
            fan_state = 0;   // "oprim ventilator"  => LED & FAN OFF
        }

        /* Alarma: cand temperatura depaseste pragul critic (35C) */
        int alarm = (current_temp >= g_alarm_threshold) ? 1 : 0;

        /* Actualizam LED-ul in functie de fan_state */
        gpio_set_level(LED_GPIO, fan_state);

        /* Actualizam ventilatorul (releul) in functie de fan_state */
        // DACA RELEUL TAU ESTE ACTIV LOW, schimba linia asta in: gpio_set_level(FAN_GPIO, fan_state ? 0 : 1);
        gpio_set_level(FAN_GPIO, fan_state);

        /* Buzzer-ul: ON cand avem alarma */
        gpio_set_level(BUZZER_GPIO, alarm);

        /* 🔹 AICI LEGAM TEMPERATURA DE BLE
           Trimitem temperatura curenta catre caracteristica BLE (0.1°C resolution) */
        ble_node_update_temperature(current_temp);

        /* Pregatim mesajul de trimis prin coada */
        TelemetryMessage msg;
        msg.temperature  = current_temp;
        msg.fan_state    = fan_state;
        msg.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        msg.alarm        = alarm;

        /* Trimitem mesajul in coada (max 10 ms de asteptare daca e plina) */
        if (queue_telemetry != NULL) {
            BaseType_t ok = xQueueSend(queue_telemetry, &msg, pdMS_TO_TICKS(10));
            if (ok != pdPASS) {
                ESP_LOGW(TAG_RT, "Queue full, could not send telemetry");
            }
        }

        ESP_LOGI(TAG_RT,
                 "Temp=%.2f C, fan=%d, alarm=%d (phase=%d, sec=%d)",
                 current_temp, fan_state, alarm, phase, counter);

        /* Intarziere de frecventa fixa */
        vTaskDelayUntil(&last_wake_time, period);
    }
}

/* ---------------- Task 2: proces de comunicatie / raportare ---------------- */
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

/* ---------------- Punctul de intrare al aplicatiei ---------------- */
void app_main(void)
{
    ESP_LOGI(TAG_APP, "app_main started");

    /* 1) Pornim BLE (Bluedroid + serviciul custom cu temperatura) */
    ble_node_init();

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

    /* 3) Cream coada de telemetrie (maxim 5 mesaje in coada) */
    queue_telemetry = xQueueCreate(5, sizeof(TelemetryMessage));
    if (queue_telemetry == NULL) {
        ESP_LOGE(TAG_APP, "Failed to create telemetry queue");
        return;
    }

    /* 4) Cream Task 1 (timp real) – prioritate mai mare */
    BaseType_t res1 = xTaskCreate(
        sensor_control_task,
        "RT_SensorControlTask",
        4096,
        NULL,
        5,        // prioritate RT
        NULL
    );

    /* 5) Cream Task 2 (comunicatie) – prioritate mai mica */
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
