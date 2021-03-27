//
// Created by curty on 3/27/2021.
//

#include "sensor_telemetry_setup.h"

/**
 * @brief setup the wifi driver (uses global variables in wifi.h; configures SPI3 Inventek module)
 */
UINT setup_wifi(bool scan_for_aps) {
    UINT status;
    uint8_t max_aps = 10;
    status = WIFI_Init();
    if (status != WIFI_STATUS_OK)
        return status;
    if (scan_for_aps == true) {
        WIFI_APs_t aps;
        while (WIFI_ListAccessPoints(&aps, max_aps) != WIFI_STATUS_OK) {
            tx_thread_sleep(100);
        }
        printf("%d", aps.count);
    }
    while (WIFI_Connect(WIFI_AP_SSID, WIFI_AP_KEY, WIFI_ECN_WPA2_PSK) != WIFI_STATUS_OK) {
        tx_thread_sleep(100);
    }
    uint8_t goog_ip[] = {8, 8, 8, 8};
    uint16_t ping_count = 10;
    int32_t ping_result[ping_count];
    while (WIFI_Ping(goog_ip, ping_count, 10, ping_result) != WIFI_STATUS_OK) {
        tx_thread_sleep(100);
    }
    printf("%ld", ping_result[0]);
    return WIFI_STATUS_OK;
}


/**
 * @brief cleanup the wifi driver
 */
void cleanup_wifi() {
    WIFI_Disconnect();
}


/**
 * @brief sets up NetX Duo to work with the wifi driver
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT setup_nx_wifi(struct global_data_t* global_data) {
    UINT status;
    uint8_t ip_address[4];
    nx_system_initialize();
    status = nx_packet_pool_create(&global_data->nx_pool, "NX Packet Pool", TX_PACKET_SIZE,
                                   global_data->nx_ip_pool, TX_POOL_SIZE);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    WIFI_GetIP_Address(ip_address);
    status = nx_ip_create(&global_data->nx_ip, "NX IP Instance 0",
                          IP_ADDRESS(ip_address[0], ip_address[1],
                                     ip_address[2], ip_address[3]),
                          0xFFFFFF00UL,
                          &global_data->nx_pool, NULL, NULL, 0, 0);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    status = nx_wifi_initialize(&global_data->nx_ip, &global_data->nx_pool);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    return status;
}


/**
 * @brief cleanup NetX Duo wifi data
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT cleanup_nx_wifi(struct global_data_t* global_data) {
    UINT status;
    status = nx_ip_delete(&global_data->nx_ip);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    status = nx_packet_pool_delete(&global_data->nx_pool);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    return status;
}


/**
 * @brief setup the netx duo MQTT client
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT setup_nx_mqtt_and_connect(struct global_data_t* global_data) {
    UINT status;
    status = nxd_mqtt_client_create(&global_data->mqtt_client, "MQTT Client",
                                    "le_board", sizeof("le_board") - 1,
                                    &global_data->nx_ip, &global_data->nx_pool,
                                    (void*)&global_data->mqtt_client_stack,
                                    sizeof(global_data->mqtt_client_stack),
                                    2, NX_NULL, 0);
    if (status != NX_SUCCESS)
        return status;
    tx_event_flags_create(&global_data->mqtt_event_flags, "mqtt events");

    global_data->mqtt_server_ip.nxd_ip_version = 4;
    global_data->mqtt_server_ip.nxd_ip_address.v4 = MQTT_BROKER_IP;
    status = nxd_mqtt_client_connect(&global_data->mqtt_client, &global_data->mqtt_server_ip,
                                     NXD_MQTT_PORT, 300, 0, NX_WAIT_FOREVER);
    return status;
}


/**
 * @brief cleanup nx MQTT client data structures
 * @param global_data: pointer to the global data structure
 */
void cleanup_nx_mqtt(struct global_data_t* global_data) {
    UINT status;
    status = nxd_mqtt_client_disconnect(&global_data->mqtt_client);
    printf("%d", status);
    status = nxd_mqtt_client_delete(&global_data->mqtt_client);
    printf("%d", status);
}