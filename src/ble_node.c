#include <string.h>
#include "ble_node.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

static const char *TAG_BLE = "BLE_NODE";

/* UUID-uri simple (custom) */
#define THERMAL_SERVICE_UUID       0xFFE0
#define THERMAL_CHAR_TEMP_UUID     0xFFE1

/* Indexuri în tabela de atribute */
enum {
    IDX_SVC,
    IDX_CHAR_TEMP,
    IDX_CHAR_VAL_TEMP,
    IDX_NB,
};

/* Handle-urile GATT */
static uint16_t thermal_handle_table[IDX_NB];

/* GATT interface & connection info */
static esp_gatt_if_t g_gatts_if      = ESP_GATT_IF_NONE;
static uint16_t      g_conn_id       = 0;
static bool          g_is_connected  = false;

/* Valoarea expusă prin BLE: temperatura în 0.1 grade (Q10) */
static int16_t g_temp_q10 = 250;   // 25.0 C inițial

/* Proprietăți caracteristică: READ + NOTIFY */
static const uint8_t char_prop_read_notify =
        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* UUID-uri standard pentru service/char declaration */
static const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid  = ESP_GATT_UUID_CHAR_DECLARE;

/* UUID-urile custom ca variabile (folosite în gatt_db și adv_data) */
static const uint16_t thermal_service_uuid_var    = THERMAL_SERVICE_UUID;
static const uint16_t thermal_char_temp_uuid_var  = THERMAL_CHAR_TEMP_UUID;

/* Tabela de atribute GATT (serviciu + caracteristică) */
static const esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    /* Serviciu primar */
    [IDX_SVC] =
    { {ESP_GATT_AUTO_RSP},
      {ESP_UUID_LEN_16,
       (uint8_t *)&primary_service_uuid,
       ESP_GATT_PERM_READ,
       sizeof(uint16_t),
       sizeof(thermal_service_uuid_var),
       (uint8_t *)&thermal_service_uuid_var} },

    /* Declaratia caracteristicii de temperatură */
    [IDX_CHAR_TEMP] =
    { {ESP_GATT_AUTO_RSP},
      {ESP_UUID_LEN_16,
       (uint8_t *)&character_declaration_uuid,
       ESP_GATT_PERM_READ,
       sizeof(uint8_t),
       sizeof(uint8_t),
       (uint8_t *)&char_prop_read_notify} },

    /* Valoarea caracteristicii de temperatură */
    [IDX_CHAR_VAL_TEMP] =
    { {ESP_GATT_AUTO_RSP},
      {ESP_UUID_LEN_16,
       (uint8_t *)&thermal_char_temp_uuid_var,   // <-- UUID-ul caracteristicii
       ESP_GATT_PERM_READ,
       sizeof(int16_t),
       sizeof(int16_t),
       (uint8_t *)&g_temp_q10} },
};

/* Numele dispozitivului */
static const char *DEVICE_NAME = "THERMAL_NODE";

/* Parametri de advertising */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* Advertising payload (nume + UUID serviciu) */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,   // pune doar numele "THERMAL_NODE" în advertising
    .include_txpower     = false,
    .min_interval        = 0,      // 0 = nu specificăm
    .max_interval        = 0,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,      // NU mai trimitem UUID-uri în advertising
    .p_service_uuid      = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};


/* Forward declarations */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param);

/* ========================================================================== */
/*                        Implementare BLE_NODE API                           */
/* ========================================================================== */

esp_err_t ble_node_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG_BLE, "Initializing BLE node...");

    /* NVS pentru stack-ul BT */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Eliberăm memorie de la Classic BT (nu folosim) */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG_BLE, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG_BLE, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG_BLE, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG_BLE, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Callback-uri GAP + GATTS */
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));

    /* Înregistrăm aplicația GATT */
    const uint16_t app_id = 0x55;
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(app_id));

    ESP_LOGI(TAG_BLE, "BLE node init OK, waiting for events...");
    return ESP_OK;
}

/* Actualizează temperatura (în 0.1°C) și, dacă e conectat, trimite notificare */
void ble_node_update_temperature(float temp_c)
{
    int16_t q10 = (int16_t)(temp_c * 10.0f);
    g_temp_q10 = q10;

    if (thermal_handle_table[IDX_CHAR_VAL_TEMP] != 0) {
        esp_ble_gatts_set_attr_value(thermal_handle_table[IDX_CHAR_VAL_TEMP],
                                     sizeof(int16_t),
                                     (uint8_t *)&g_temp_q10);

        if (g_is_connected && g_gatts_if != ESP_GATT_IF_NONE) {
            esp_ble_gatts_send_indicate(g_gatts_if,
                                        g_conn_id,
                                        thermal_handle_table[IDX_CHAR_VAL_TEMP],
                                        sizeof(int16_t),
                                        (uint8_t *)&g_temp_q10,
                                        false); // notification
        }
    }
}

/* ========================================================================== */
/*                             GAP event handler                              */
/* ========================================================================== */

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    ESP_LOGI(TAG_BLE, "GAP event: %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG_BLE, "ADV data set, starting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG_BLE, "Advertising start failed, status 0x%x",
                     param->adv_start_cmpl.status);
        } else {
            ESP_LOGI(TAG_BLE, "Advertising started");
        }
        break;

    default:
        break;
    }
}

/* ========================================================================== */
/*                            GATTS event handler                             */
/* ========================================================================== */

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(TAG_BLE, "GATTS event: %d, gatts_if=%d", event, gatts_if);

    switch (event) {
    case ESP_GATTS_REG_EVT:
    {
        ESP_LOGI(TAG_BLE, "ESP_GATTS_REG_EVT: set device name & config adv");

        g_gatts_if = gatts_if;

        esp_err_t err;

        err = esp_ble_gap_set_device_name(DEVICE_NAME);
        ESP_LOGI(TAG_BLE, "esp_ble_gap_set_device_name: %s", esp_err_to_name(err));

        err = esp_ble_gap_config_adv_data(&adv_data);
        ESP_LOGI(TAG_BLE, "esp_ble_gap_config_adv_data: %s", esp_err_to_name(err));

        /* Creăm tabela de atribute */
        err = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, 0);
        ESP_LOGI(TAG_BLE, "esp_ble_gatts_create_attr_tab: %s", esp_err_to_name(err));
        break;
    }

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG_BLE, "Create attr table failed, error 0x%x",
                     param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(TAG_BLE, "Create attr table abnormal, num_handle=%d, IDX_NB=%d",
                     param->add_attr_tab.num_handle, IDX_NB);
        } else {
            ESP_LOGI(TAG_BLE, "Attr table created, starting service");
            memcpy(thermal_handle_table, param->add_attr_tab.handles,
                   sizeof(thermal_handle_table));
            esp_ble_gatts_start_service(thermal_handle_table[IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG_BLE, "Client connected");
        g_is_connected = true;
        g_conn_id      = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG_BLE, "Client disconnected, restart advertising");
        g_is_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}
