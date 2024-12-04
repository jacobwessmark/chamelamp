#include "esp_now_handler.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define ESPNOW_CHANNEL 1 // Define a fixed channel for ESP-NOW communication

static const char *TAG = "ESP-NOW_Sender";
static QueueHandle_t rgb_send_queue;

// Global peer info for broadcast
static esp_now_peer_info_t broadcast_peer;

static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    ESP_LOGI(TAG, "Last Packet Sent to: %s", macStr);
    ESP_LOGI(TAG, "Last Packet Send Status: %s",
             status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

static void init_broadcast_peer(void)
{
    // Clear peer info
    memset(&broadcast_peer, 0, sizeof(broadcast_peer));

    // Set broadcast address FF:FF:FF:FF:FF:FF
    for (int i = 0; i < 6; i++)
    {
        broadcast_peer.peer_addr[i] = 0xFF;
    }

    // Use the fixed ESP-NOW channel defined in wifi_manager.h
    broadcast_peer.channel = ESPNOW_CHANNEL;  // This should be 1
    broadcast_peer.encrypt = false;

    ESP_LOGI(TAG, "Setting ESP-NOW broadcast to channel %d", ESPNOW_CHANNEL);
}

static bool manage_peer(void)
{
    // Always delete existing peer to ensure channel is updated
    esp_now_del_peer(broadcast_peer.peer_addr);

    // Attempt to add peer
    esp_err_t result = esp_now_add_peer(&broadcast_peer);
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Broadcast peer added successfully on channel %d", broadcast_peer.channel);
        return true;
    }

    ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(result));
    return false;
}

static void esp_now_send_task(void *param)
{
    rgb_data_t rgb_data;
    
    // Add this task to the WDT
    esp_task_wdt_add(NULL);

    while (1)
    {
        if (xQueueReceive(rgb_send_queue, &rgb_data, portMAX_DELAY) == pdTRUE)
        {
            // Verify channel before sending
            uint8_t current_channel;
            wifi_second_chan_t second;
            esp_wifi_get_channel(&current_channel, &second);
            
            if (current_channel != ESPNOW_CHANNEL) {
                ESP_LOGW(TAG, "Channel mismatch detected. WiFi: %d, Expected: %d. Resetting channel.", 
                        current_channel, ESPNOW_CHANNEL);
                esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
            }

            esp_err_t result = esp_now_send(broadcast_peer.peer_addr,
                                          (uint8_t *)&rgb_data,
                                          sizeof(rgb_data_t));

            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "Sent RGB data - R: %d, G: %d, B: %d on channel %d",
                         rgb_data.red, rgb_data.green, rgb_data.blue, ESPNOW_CHANNEL);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send RGB data: %s", esp_err_to_name(result));
                // Re-add peer with correct channel if send fails
                if (result == ESP_ERR_ESPNOW_CHAN) {
                    manage_peer();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }
}

esp_err_t initialize_esp_now_sender(void)
{

    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW initialization failed");
        return ret;
    }

    // Register callbacks
    esp_now_register_send_cb(esp_now_send_cb);

    // Initialize and add broadcast peer
    init_broadcast_peer();
    if (!manage_peer())
    {
        ESP_LOGE(TAG, "Failed to manage broadcast peer");
        return ESP_FAIL;
    }

    // Create queue for RGB data
    rgb_send_queue = xQueueCreate(QUEUE_SIZE, sizeof(rgb_data_t));
    if (!rgb_send_queue)
    {
        ESP_LOGE(TAG, "Failed to create RGB send queue");
        return ESP_FAIL;
    }

    xTaskCreate(esp_now_send_task, "esp_now_send_task", ESP_NOW_TASK_STACK_SIZE,
                NULL, ESP_NOW_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "ESP-NOW sender initialized successfully");
    return ESP_OK;
}

void add_rgb_to_send_queue(const rgb_data_t *data)
{
    if (!data || !rgb_send_queue)
    {
        ESP_LOGE(TAG, "Invalid data or queue not initialized");
        return;
    }

    if (xQueueSend(rgb_send_queue, data, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to add RGB data to send queue");
    }

    ESP_LOGI(TAG, "Added RGB data to send queue - R: %d, G: %d, B: %d",
             data->red, data->green, data->blue);
}