#include "esp_now_handler.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lcd1602.h"
#include <string.h>

static const char *TAG = "ESP-NOW_Receiver";
TaskHandle_t esp_now_task_handle = NULL;

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len == sizeof(rgb_data_t))
    {
        rgb_data_t rgb_data;
        memcpy(&rgb_data, data, sizeof(rgb_data_t));

        // Clear display first
        lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
        vTaskDelay(pdMS_TO_TICKS(2)); // Small delay after clear

        // First line: RGB values
        char line1[16];
        snprintf(line1, sizeof(line1), "RGB:%u,%u,%u",
                 (unsigned)rgb_data.red,
                 (unsigned)rgb_data.green,
                 (unsigned)rgb_data.blue);
        lcd_display_text(line1, 0); // Display on row 0

        // Second line: Color name
        lcd_display_text(rgb_data.color_name, 1); // Display on row 1

        ESP_LOGI(TAG, "Received RGB data - R: %d, G: %d, B: %d - Color: %s",
                 rgb_data.red, rgb_data.green, rgb_data.blue, rgb_data.color_name);
    }
    else
    {
        ESP_LOGW(TAG, "Received data of unexpected size: %d bytes", len);
    }
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

esp_err_t initialize_esp_now_receiver(void)
{
    // Initialize WiFi
    esp_err_t ret = init_wifi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed");
        return ret;
    }

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW initialization failed");
        return ret;
    }

    // Register callback
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW receiver initialized successfully");
    return ESP_OK;
} 