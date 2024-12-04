#pragma once

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#define ESPNOW_CHANNEL 1
#define WIFI_CONNECT_RETRY_MAX 5

typedef struct wifi_status
{
    bool connected;
    bool ip_assigned;
    uint8_t current_channel;
} wifi_status_t;

esp_err_t initialize_wifi(void);
bool is_wifi_connected(void);
esp_err_t connect_to_wifi(void);