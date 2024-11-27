#include "esp_now_handler.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

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

    broadcast_peer.channel = ESPNOW_CHANNEL;
    broadcast_peer.encrypt = false;
}

static bool manage_peer(void)
{
    if (DELETEBEFOREPAIR)
    {
        esp_now_del_peer(broadcast_peer.peer_addr);
    }

    // Check if peer exists
    if (esp_now_is_peer_exist(broadcast_peer.peer_addr))
    {
        ESP_LOGI(TAG, "Broadcast peer already paired");
        return true;
    }

    // Attempt to add peer
    esp_err_t result = esp_now_add_peer(&broadcast_peer);
    if (result == ESP_OK)
    {
        ESP_LOGI(TAG, "Broadcast peer added successfully");
        return true;
    }

    ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(result));
    return false;
}

static esp_err_t init_wifi(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    return ESP_OK;
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
            esp_err_t result = esp_now_send(broadcast_peer.peer_addr,
                                            (uint8_t *)&rgb_data,
                                            sizeof(rgb_data_t));

            if (result == ESP_OK)
            {
                ESP_LOGI(TAG, "Sent RGB data - R: %d, G: %d, B: %d",
                         rgb_data.red, rgb_data.green, rgb_data.blue);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send RGB data: %s", esp_err_to_name(result));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        esp_task_wdt_reset();
    }
}

esp_err_t initialize_esp_now_sender(void)
{
    // Initialize WiFi
    esp_err_t ret = init_wifi();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return ret;
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
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