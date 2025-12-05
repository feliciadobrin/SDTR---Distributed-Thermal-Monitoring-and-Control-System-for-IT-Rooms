#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializeaza stack-ul BLE si porneste advertising-ul
 * Serviciu custom cu o caracteristica de temperatura.
 *
 * Intoarce ESP_OK daca totul este in regula.
 */
esp_err_t ble_node_init(void);

/**
 * Actualizeaza temperatura expusa prin BLE.
 * temp_c = temperatura in grade Celsius.
 */
void ble_node_update_temperature(float temp_c);

#ifdef __cplusplus
}
#endif
