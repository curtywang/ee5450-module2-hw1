//
// Created by curty on 3/27/2021.
//

#ifndef EE5450_MODULE2_HW1_SENSOR_TELEMETRY_H
#define EE5450_MODULE2_HW1_SENSOR_TELEMETRY_H

#include "main.h"

UINT send_nx_mqtt_message(struct global_data_t* global_data, char* topic, char* message);
void get_temperature_message(struct global_data_t* global_data, char* message, size_t message_size);
void get_accelerometer_message(struct global_data_t* global_data, char* message, size_t message_size);
/* TODO: declare your other sensor data functions here */

#endif //EE5450_MODULE2_HW1_SENSOR_TELEMETRY_H
