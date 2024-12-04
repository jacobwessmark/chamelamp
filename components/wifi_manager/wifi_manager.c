#include "wifi_manager.h"
#include <string.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_now.h"

wifi_status_t wifi_status;

static const char *TAG = "WIFI_MANAGER";

void ip_event_handler(void *event_handler_arg,
                      esp_event_base_t event_base,
                      int32_t event_id,
                      void *event_data)
{
    wifi_status_t *wifi_status = (wifi_status_t *)event_handler_arg;
    char ip_str[16] = {0};
    switch (event_id)
    {
    case IP_EVENT_STA_GOT_IP:
        esp_netif_ip_info_t *ip_info = (esp_netif_ip_info_t *)event_data;
        wifi_status->ip_assigned = true;
        ESP_LOGI(TAG, "IP: %s", esp_ip4addr_ntoa(&ip_info->ip, ip_str, sizeof(ip_str)));
        memset(ip_str, 0, sizeof(ip_str));
        ESP_LOGI(TAG, "Netmask: %s", esp_ip4addr_ntoa(&ip_info->netmask, ip_str, sizeof(ip_str)));
        memset(ip_str, 0, sizeof(ip_str));
        ESP_LOGI(TAG, "Gateway: %s", esp_ip4addr_ntoa(&ip_info->gw, ip_str, sizeof(ip_str)));

        break;
    }
}

void wifi_event_handler(void *event_handler_arg,
                        esp_event_base_t event_base,
                        int32_t event_id,
                        void *event_data)
{
    wifi_status_t *wifi_status = (wifi_status_t *)event_handler_arg;
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        break;
    case WIFI_EVENT_STA_CONNECTED:
        wifi_status->connected = true;
        // Ensure the channel is set correctly after connection
        uint8_t current_channel;
        wifi_second_chan_t second;
        esp_wifi_get_channel(&current_channel, &second);
        if (current_channel != ESPNOW_CHANNEL)
        {
            ESP_LOGW(TAG, "Channel changed to %d, resetting to %d", current_channel, ESPNOW_CHANNEL);
            esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        }
        break;

    case WIFI_EVENT_STA_DISCONNECTED:
        wifi_status->connected = false;
        // Reset channel on disconnect
        ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
        break;

    default:
        break;
    }
}

void initialize_wifi_task(void *param)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, &wifi_status));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_event_handler, &wifi_status));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    // Configure WiFi
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASS);
    
    // Set fixed channel in WiFi config
    wifi_config.sta.channel = ESPNOW_CHANNEL;
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -127;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Start WiFi without connecting
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Force the channel setting
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
    
    // Initialize ESP-NOW
    esp_now_init();

    vTaskDelete(NULL);
}

esp_err_t initialize_wifi(void)
{
    xTaskCreate(initialize_wifi_task, "initialize_wifi_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}

bool is_wifi_connected(void)
{
    return wifi_status.connected;
}

void connect_to_wifi_task(void *param)
{
    ESP_LOGI(TAG, "Connecting to WiFi with credentials: %s, %s", CONFIG_WIFI_SSID, CONFIG_WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_connect());

    // Wait until connected
    while (!wifi_status.connected)
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent busy-waiting
    }

    vTaskDelete(NULL); // Delete the task once connected
}

esp_err_t connect_to_wifi(void)
{
    xTaskCreate(connect_to_wifi_task, "connect_to_wifi_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}
