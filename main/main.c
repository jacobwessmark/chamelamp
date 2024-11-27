#include "TCS34725_colorsensor.h"
#include "TCS34725_helper_functions.h"
#include "esp_now_handler.h"
#include "lcd1602.h"
#include "esp_mac.h"
#include "neopixel_ring.h"
#include "pin_map.h"
#include "esp_sleep.h"
#include "nvs_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Main";

// Define an array of NeopixelRing instances
NeopixelRing rings[] = {
    {NULL, SMALL_RING_LED_COUNT, 0, 0, 0, SMALL_RING_PIN},
    {NULL, BIG_RING_LED_COUNT, 0, 0, 0, BIG_RING_PIN}};

void app_main(void)
{

#ifdef CONFIG_DEVICE_MODE_SENDER

    // Initialize all NeoPixel rings
    init_neopixel_rings(rings, sizeof(rings) / sizeof(rings[0]));

    // Wakeup cause
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
        printf("Wakeup caused by timer\n");

        // Reinitialize I2C bus if necessary
        if (i2c_bus_handle == NULL)
        {
            ESP_ERROR_CHECK(initialize_i2c_bus(TCS_I2C_MASTER_SCL_IO, TCS_I2C_MASTER_SDA_IO));
        }

        // Perform actions needed for timer wakeup
        esp_err_t ret = update_thresholds_from_clear();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to update thresholds");
        }

        // Enable wakeup
        enable_wakeup(TCS_INT_PIN); 

        // Sleep and deinitialize I2C bus
        deep_sleep_with_deinitialized_i2c(i2c_bus_handle);

        break;

    case ESP_SLEEP_WAKEUP_EXT0:
        printf("Wakeup deep sleep caused by external signal using RTC_IO\n");

        // turn on small ring
        fade_to_color(&rings[0], 255, 255, 255);

        // Initialize color sensor with all rings
        if (init_chamelamp(TCS_I2C_MASTER_SCL_IO, TCS_I2C_MASTER_SDA_IO) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize color sensor");
            // Turn on red color on the small ring
            fade_to_color(&rings[0], 255, 0, 0);
        }
        // Enable wakeup
        enable_wakeup(TCS_INT_PIN);

        // Initialize ESP-NOW sender
        ESP_ERROR_CHECK(initialize_esp_now_sender());

        // Start data acquisition task
        start_data_acquisition_task(rings);
        break;

    default:

        ESP_LOGI(TAG, "Initializing device as ESP-NOW sender with color sensor");

        // Turn on small ring
        fade_to_color(&rings[0], 0, 100, 0);

        // Initialize color sensor with all rings
        if (init_chamelamp(TCS_I2C_MASTER_SCL_IO, TCS_I2C_MASTER_SDA_IO) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize color sensor");
            // Turn on red color on the small ring
            fade_to_color(&rings[0], 255, 0, 0);
        }

        // Turn off small ring
        fade_to_color(&rings[0], 0, 0, 0);

        // Enable wakeup
        enable_wakeup(TCS_INT_PIN);

        // Sleep and deinitialize I2C bus
        deep_sleep_with_deinitialized_i2c(i2c_bus_handle);

        break;
    }


#else // CONFIG_DEVICE_MODE_RECEIVER
    ESP_LOGI(TAG, "Initializing device as ESP-NOW receiver with LCD");

    // Initialize NVS handler first
    ESP_ERROR_CHECK(nvs_handler_init());

    // Initialize LCD
    lcd_init(LCD_I2C_MASTER_SCL_IO, LCD_I2C_MASTER_SDA_IO);

    // Load last saved color
    uint8_t last_red, last_green, last_blue;
    char last_color_name[32];
    esp_err_t ret = nvs_handler_load_color(&last_red, &last_green, &last_blue, last_color_name, sizeof(last_color_name));
    if (ret == ESP_OK)
    {
        // Check if the last color was black
        if (last_red == 0 && last_green == 0 && last_blue == 0)
        {
            lcd_start_animation();
        }
        else
        {
            // Display the last saved color
            lcd_display_text("Chamelamp", 0);
            char display_text[40];
            snprintf(display_text, sizeof(display_text), "Last: %s", last_color_name);
            lcd_display_text(display_text, 1);
        }
    }
    else
    {
        // Display initial message if no color is saved
        lcd_display_text("Waiting for RGB...", 0);
    }

    // Initialize ESP-NOW receiver with callback for animation control
    ESP_ERROR_CHECK(initialize_esp_now_receiver());

#endif

    ESP_LOGI(TAG, "Initialization complete");
}