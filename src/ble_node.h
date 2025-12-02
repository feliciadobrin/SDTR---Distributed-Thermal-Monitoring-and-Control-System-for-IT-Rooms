#ifndef BLE_NODE_H
#define BLE_NODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Inițializează BLE (apel din app_main)
void ble_node_init(void);

// Actualizează valoarea temperaturii expusă prin BLE (în grade Celsius)
void ble_node_update_temperature(float temp_c);

#ifdef __cplusplus
}
#endif

#endif // BLE_NODE_H
