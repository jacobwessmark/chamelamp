#include "TCS34725_colorsensor.h"
#include "esp_now_handler.h"
#include "lcd1602.h"
#include "esp_mac.h"
#include "neopixel_ring.h"

static const char *TAG = "Main";

void app_main(void)
{
    // Display MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Check device type configuration
    #if defined(CONFIG_DEVICE_TYPE) && CONFIG_DEVICE_TYPE == "sender"
        ESP_LOGI(TAG, "Initializing device as ESP-NOW sender with color sensor");
        
        // Initialize NeoPixel rings
        init_all_rings();
        vTaskDelay(pdMS_TO_TICKS(1000));
        set_small_ring_color(100, 100, 100);

        // Initialize color sensor
        if (init() != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize color sensor");
            set_small_ring_color(255, 0, 0); // Red to indicate error
        }
        
        // Initialize ESP-NOW sender last
        ESP_ERROR_CHECK(initialize_esp_now_sender());

    #elif defined(CONFIG_DEVICE_TYPE) && CONFIG_DEVICE_TYPE == "receiver"
        ESP_LOGI(TAG, "Initializing device as ESP-NOW receiver with LCD");
        
        // Initialize I2C for LCD only
        ESP_ERROR_CHECK(i2c_master_init());
        ESP_ERROR_CHECK(lcd_init());
        
        // Initialize ESP-NOW receiver
        ESP_ERROR_CHECK(initialize_esp_now_receiver());
        
        // Display initial message on LCD
        lcd_display_text("Waiting for RGB...");

    #else
        #error "Invalid device type configuration"
    #endif

    ESP_LOGI(TAG, "Initialization complete");
}