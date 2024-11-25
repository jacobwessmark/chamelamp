#include "TCS34725_colorsensor.h"
#include "TCS34725_helper_functions.h"
#include "esp_now_handler.h"
#include "lcd1602.h"
#include "esp_mac.h"
#include "neopixel_ring.h"
#include "pin_map.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

#define THRESHOLD_UPDATE_INTERVAL_US 5 * 1000000

static const char *TAG = "Main";

// Define an array of NeopixelRing instances
NeopixelRing rings[] = {
    {NULL, SMALL_RING_LED_COUNT, 0, 0, 0, SMALL_RING_PIN},
    {NULL, BIG_RING_LED_COUNT, 0, 0, 0, BIG_RING_PIN}};

void enable_wakeup(gpio_num_t int_pin)
{
    ESP_LOGI(TAG, "Enabling wake and sleep");
    // Initialize the RTC GPIO
    rtc_gpio_init(int_pin);

    // Set the direction of the RTC GPIO
    rtc_gpio_set_direction(int_pin, RTC_GPIO_MODE_INPUT_ONLY);

    // Enable the wakeup source
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(int_pin, 0));
    //ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(THRESHOLD_UPDATE_INTERVAL_US));
}

void app_main(void)
{

#ifdef CONFIG_DEVICE_MODE_SENDER

    // Initialize all NeoPixel rings
    init_neopixel_rings(rings, sizeof(rings) / sizeof(rings[0]));
    // Turn off all rings
    turn_off_all_rings(rings, sizeof(rings) / sizeof(rings[0]));

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

        // Turn off all rings
        esp_deep_sleep_start();

        break;

    case ESP_SLEEP_WAKEUP_EXT0:
        printf("Wakeup deep sleep caused by external signal using RTC_IO\n");

        // turn on small ring
        fade_to_color(&rings[0], 100, 100, 100);

        // init the i2c bus
        if (i2c_bus_handle == NULL)
        {
            ESP_ERROR_CHECK(initialize_i2c_bus(TCS_I2C_MASTER_SCL_IO, TCS_I2C_MASTER_SDA_IO));
        }

        ESP_ERROR_CHECK(initialize_esp_now_sender());
        start_data_acquisition_task(rings);
        break;

    default:

        ESP_LOGI(TAG, "Initializing device as ESP-NOW sender with color sensor");

        // Turn on small ring
        fade_to_color(&rings[0], 100, 100, 100);

        // Initialize color sensor with all rings
        if (init_chamelamp(TCS_I2C_MASTER_SCL_IO, TCS_I2C_MASTER_SDA_IO) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize color sensor");
            // Turn on red color on the small ring
            fade_to_color(&rings[0], 255, 0, 0); // Assuming small_ring is at index 0
        }

        ESP_ERROR_CHECK(initialize_esp_now_sender());
        start_data_acquisition_task(rings);

        break;
    }

    // Enable wakeup
    enable_wakeup(TCS_INT_PIN);
    

#else // CONFIG_DEVICE_MODE_RECEIVER
    ESP_LOGI(TAG, "Initializing device as ESP-NOW receiver with LCD");

    // Initialize LCD
    lcd_init(LCD_I2C_MASTER_SCL_IO, LCD_I2C_MASTER_SDA_IO);

    // Initialize ESP-NOW receiver
    ESP_ERROR_CHECK(initialize_esp_now_receiver());

    // Display initial message on LCD
    lcd_display_text("Waiting for RGB...", 0);
#endif

    ESP_LOGI(TAG, "Initialization complete");
}