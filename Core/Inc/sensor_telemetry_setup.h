//
// Created by curty on 3/27/2021.
//

#ifndef EE5450_MODULE2_HW1_SENSOR_TELEMETRY_SETUP_H
#define EE5450_MODULE2_HW1_SENSOR_TELEMETRY_SETUP_H

#include "main.h"

#define WIFI_AP_SSID "Hello Home"  // TODO: change this to yours
#define WIFI_AP_KEY "TaiwanNumbaOne"  // TODO: change this to yours
#define MQTT_BROKER_IP IP_ADDRESS(192, 168, 11, 123)  // TODO: change this IP address to yours

UINT setup_wifi(bool scan_for_aps);
void cleanup_wifi();
UINT setup_nx_wifi(struct global_data_t* global_data);
UINT cleanup_nx_wifi(struct global_data_t* global_data);
UINT setup_nx_mqtt_and_connect(struct global_data_t* global_data);
void cleanup_nx_mqtt(struct global_data_t* global_data);

#endif //EE5450_MODULE2_HW1_SENSOR_TELEMETRY_SETUP_H
