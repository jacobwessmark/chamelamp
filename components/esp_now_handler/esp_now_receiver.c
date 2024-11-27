#include "esp_now_handler.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lcd1602.h"
#include "nvs_handler.h"
#include <string.h>

static const char *TAG = "ESP-NOW_Receiver";
TaskHandle_t esp_now_task_handle = NULL;

// Add static variables to store previous color
static uint8_t prev_red = 0;
static uint8_t prev_green = 0;
static uint8_t prev_blue = 0;
static char prev_color_name[32] = {0};

static void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len == sizeof(rgb_data_t))
    {
        rgb_data_t rgb_data;
        memcpy(&rgb_data, data, sizeof(rgb_data_t));

        // Check if the color is different from the previous one
        bool color_changed = (rgb_data.red != prev_red ||
                              rgb_data.green != prev_green ||
                              rgb_data.blue != prev_blue ||
                              strcmp(rgb_data.color_name, prev_color_name) != 0);

        if (color_changed)
        {
            // Update previous color values
            prev_red = rgb_data.red;
            prev_green = rgb_data.green;
            prev_blue = rgb_data.blue;
            strncpy(prev_color_name, rgb_data.color_name, sizeof(prev_color_name) - 1);
            prev_color_name[sizeof(prev_color_name) - 1] = '\0';

            // Check if the color is black (0,0,0)
            if (rgb_data.red == 0 && rgb_data.green == 0 && rgb_data.blue == 0)
            {

                // clear display first
                lcd_send_command(LCD_CMD_CLEAR_DISPLAY);

                // Write chamelamp to display
                lcd_display_text("   Chamelamp", 0);

                // Start the animation for black color
                lcd_start_animation();
                ESP_LOGI(TAG, "Black color detected, starting animation");
            }
            else
            {
                ESP_LOGI(TAG, "Non-black color detected, stopping animation");
                // Stop any running animation
                lcd_stop_animation();

                vTaskDelay(pdMS_TO_TICKS(100)); // Small delay after clear

                lcd_display_text("Now showing", 0);

                vTaskDelay(pdMS_TO_TICKS(10));

                // Second line: Color name
                lcd_display_text(rgb_data.color_name, 1); // Display on row 1

                // Save non-black colors to NVS
                nvs_handler_save_color(rgb_data.red, rgb_data.green, rgb_data.blue, rgb_data.color_name);
            }

            ESP_LOGI(TAG, "New color received - R: %d, G: %d, B: %d - Color: %s",
                     rgb_data.red, rgb_data.green, rgb_data.blue, rgb_data.color_name);
        }
        else
        {
            ESP_LOGD(TAG, "Received same color as before, skipping LCD update");
        }
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

    // Register callback
    ret = esp_now_register_recv_cb(esp_now_recv_cb);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register receive callback: %s", esp_err_to_name(ret));
        esp_now_deinit();
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW receiver initialized successfully");
    return ESP_OK;
}