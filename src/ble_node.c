#include "ble_node.h"

#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

static const char *TAG = "BLE_NODE";

// ID arbitrar pentru aplicația GATTS
#define GATTS_APP_ID   0x55
#define SVC_INST_ID    0

// UUID-uri custom pentru serviciu și caracteristică
#define GATTS_SERVICE_UUID_THERMAL  0xFFF0
#define GATTS_CHAR_UUID_TEMP        0xFFF1

// Numărul de handle-uri în tabela GATT (service + char decl + char value)
enum {
    IDX_SVC,
    IDX_CHAR_TEMP,
    IDX_CHAR_TEMP_VAL,
    HRS_IDX_NB,
};

#define GATTS_NUM_HANDLE  HRS_IDX_NB

// Parametrii de advertising
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Datele de advertising (nume + flag-uri)
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// UUID-uri standard pentru service / characteristic
static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;

// Proprietăți pentru caracteristica de temperatură (READ deocamdată)
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;

// Valoarea temperaturii stocată intern (0.1 °C -> int16_t)
static uint8_t temp_char_value[2] = {0x00, 0x00};

// Tabela de handle-uri creată de GATTS
static uint16_t thermal_handle_table[HRS_IDX_NB];

// Păstrăm gatt_if pentru a putea actualiza mai târziu atributele
static esp_gatt_if_t gatts_if_global = ESP_GATT_IF_NONE;

// Forward declarations
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

// Baza de date GATT (service + characteristic)
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] = {

    // Serviciul nostru custom THERMAL_SERVICE (UUID 0xFFF0)
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16,
            (uint8_t *)&primary_service_uuid,
            ESP_GATT_PERM_READ,
            sizeof(uint16_t),
            sizeof(uint16_t),
            (uint8_t *)&(uint16_t){GATTS_SERVICE_UUID_THERMAL}
        }
    },

    // Declarația caracteristicii TEMPERATURE (READ)
    [IDX_CHAR_TEMP] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16,
            (uint8_t *)&character_declaration_uuid,
            ESP_GATT_PERM_READ,
            sizeof(uint8_t),
            sizeof(uint8_t),
            (uint8_t *)&char_prop_read
        }
    },

    // Valoarea caracteristicii TEMPERATURE (UUID 0xFFF1)
    [IDX_CHAR_TEMP_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {
            ESP_UUID_LEN_16,
            (uint8_t *)&(uint16_t){GATTS_CHAR_UUID_TEMP},
            ESP_GATT_PERM_READ,
            sizeof(temp_char_value),
            sizeof(temp_char_value),
            (uint8_t *)temp_char_value
        }
    },
};

// ---------------- GAP HANDLER ----------------

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ADV data set complete, starting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed: %d", param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "Advertising started");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising stopped");
        break;

    default:
        break;
    }
}

// ---------------- GATTS HANDLER ----------------

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATTS_REG_EVT, app_id=%d", param->reg.app_id);
        gatts_if_global = gatts_if;

        // Creăm tabela de atribute pentru serviciul nostru
        esp_err_t ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if,
                                                      HRS_IDX_NB, SVC_INST_ID);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "create_attr_tab failed: %s", esp_err_to_name(ret));
        }
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Create attribute table failed, status 0x%x",
                     param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
            ESP_LOGE(TAG, "Create attribute table abnormally, num_handle (%d) != (%d)",
                     param->add_attr_tab.num_handle, HRS_IDX_NB);
        } else {
            ESP_LOGI(TAG, "Attribute table created successfully, starting service");
            memcpy(thermal_handle_table, param->add_attr_tab.handles,
                   sizeof(thermal_handle_table));
            esp_ble_gatts_start_service(thermal_handle_table[IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "Client connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "Client disconnected, restarting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

// ---------------- API PUBLIC: INIT + UPDATE TEMP ----------------

void ble_node_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing BLE (Bluedroid) with custom thermal service...");

    // 1. NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Bluetooth controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // 3. Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return;
    }

    // 4. Callback-uri GAP & GATTS
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    // Înregistrăm aplicația GATTS
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));

    // 5. Numele device-ului & advertising data
    const char *dev_name = "IT_NODE_1";
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(dev_name));
    ESP_LOGI(TAG, "Device name set to '%s'", dev_name);

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    ESP_LOGI(TAG, "BLE init complete, waiting for ADV config & GATT table...");
}

// Funcție chemată din task-ul de temperatură
void ble_node_update_temperature(float temp_c)
{
    // convertim în zecimi de grad pentru a folosi un int16_t
    int16_t t = (int16_t)roundf(temp_c * 10.0f);

    temp_char_value[0] = (uint8_t)(t & 0xFF);
    temp_char_value[1] = (uint8_t)((t >> 8) & 0xFF);

    if (thermal_handle_table[IDX_CHAR_TEMP_VAL] != 0) {
        esp_err_t ret = esp_ble_gatts_set_attr_value(
                thermal_handle_table[IDX_CHAR_TEMP_VAL],
                sizeof(temp_char_value),
                temp_char_value);

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "set_attr_value failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGD(TAG, "Temperature updated over BLE: %.1f C", temp_c);
        }
    } else {
        // Încă nu a fost creată tabela GATT (de ex. imediat după boot)
        ESP_LOGD(TAG, "GATT table not ready yet, cannot update temperature");
    }
}
