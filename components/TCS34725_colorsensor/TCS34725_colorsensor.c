#include "TCS34725_colorsensor.h"
#include "TCS34725_colorsensor_private.h"
#include "TCS34725_helper_functions.h"
#include "esp_log.h"
#include "esp_now_handler.h"

static const char *TAG = "TCS34725";

i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t i2c_dev_handle;

// I2C semaphore
SemaphoreHandle_t i2c_semaphore;

// Define black threshold
uint8_t BLACK_THRESHOLD = 100;

RGBColor rgb_colors[] = {
    // Pure Red and variants
    {"Red", 255, 0, 0},
    {"Bright Red", 255, 20, 20},
    {"Dark Red", 200, 0, 0},
    {"Deep Red", 180, 0, 0},
    {"Ruby Red", 155, 0, 20},

    // Red-Orange spectrum
    {"Reddish-Orange", 255, 64, 0},
    {"Orange", 255, 128, 0},
    {"Yellow-Orange", 255, 191, 0},

    // Yellows
    {"Yellow", 255, 255, 0},
    {"Dark Yellow", 204, 204, 0},
    {"Yellow-Green", 191, 255, 0},

    // Greens
    {"Green", 0, 255, 0},
    {"Spring Green", 0, 255, 127},
    {"Lime Green", 50, 205, 50},

    // Blues and Cyans
    {"Cyan", 0, 255, 255},
    {"Light Blue", 0, 127, 255},
    {"Blue", 0, 0, 255},

    // Purples and Pinks
    {"Purple", 128, 0, 128},
    {"Pink", 255, 0, 128},
    {"Crimson", 220, 20, 60},

};

// Map 16-bit value to 8-bit range
uint8_t map_value(uint16_t x, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
    return (uint8_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Function to find the closest predefined color
const RGBColor *get_closest_color(uint8_t avg_r, uint8_t avg_g, uint8_t avg_b)
{
    int closest_index = 0;
    int min_diff = 255 * 3;

    for (int i = 0; i < NUM_RGB_COLORS; ++i)
    {
        int diff_r = abs(avg_r - rgb_colors[i].r);
        int diff_g = abs(avg_g - rgb_colors[i].g);
        int diff_b = abs(avg_b - rgb_colors[i].b);
        int total_diff = diff_r + diff_g + diff_b;

        if (total_diff < min_diff)
        {
            min_diff = total_diff;
            closest_index = i;
        }
    }

    return &rgb_colors[closest_index];
}

esp_err_t update_thresholds_from_clear(void)
{
    uint16_t r, g, b, clear;
    esp_err_t ret;

    while (1) // Loop until thresholds are set successfully
    {
        ret = get_raw_data(&r, &g, &b, &clear);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor data");
            return ret;
        }

        ESP_LOGI(TAG, "Current clear value: %u", clear);

        // Calculate thresholds as percentages of current value
        uint32_t low_threshold = (clear * THRESHOLD_PERCENTAGE_BELOW) / 100;  // 70% of current value
        uint32_t high_threshold = (clear * THRESHOLD_PERCENTAGE_ABOVE) / 100; // 130% of current value

        // Ensure minimum difference between thresholds
        if (high_threshold - low_threshold < 1000)
        {
            low_threshold = (clear > 500) ? clear - 500 : 0;
            high_threshold = clear + 500;
        }

        // Apply safety limits
        if (low_threshold < 100)
            low_threshold = 100;
        if (high_threshold > 65000)
            high_threshold = 65000;

        // Ensure high_threshold is greater than low_threshold
        if (high_threshold <= low_threshold)
        {
            high_threshold = low_threshold + 1000; // Ensure a minimum gap
        }

        ESP_LOGI(TAG, "Setting new thresholds - Low: %" PRIu32 ", High: %" PRIu32,
                 low_threshold, high_threshold);

        ret = set_interrupt_thresholds((uint16_t)low_threshold, (uint16_t)high_threshold);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set interrupt thresholds");
            return ret;
        }

        // Re-read the clear channel to verify it's within the new thresholds
        ret = get_raw_data(&r, &g, &b, &clear);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor data after setting thresholds");
            return ret;
        }

        if (clear >= low_threshold && clear <= high_threshold)
        {
            break; // Exit loop if successful
        }
        else
        {
            ESP_LOGW(TAG, "Clear value %" PRIu16 " is out of the new threshold range [%" PRIu32 ", %" PRIu32 "], retrying...",
                     clear, low_threshold, high_threshold);
            vTaskDelay(pdMS_TO_TICKS(100)); // Optional delay before retrying
        }
    }

    return ESP_OK;
}

void data_acquisition_task(void *param)
{
    NeopixelRing *rings = (NeopixelRing *)param; // Cast param to a pointer to NeopixelRing pointers
    NeopixelRing *big_ring = &rings[1];          // Access the first element
    NeopixelRing *small_ring = &rings[0];        // Access the second element
    uint16_t r, g, b, c;
    uint8_t r_8bit, g_8bit, b_8bit;
    rgb_data_t rgb_data;

    // Arrays to store the current colors for each LED
    uint8_t stored_r[big_ring->led_count];
    uint8_t stored_g[big_ring->led_count];
    uint8_t stored_b[big_ring->led_count];

    // Initialize arrays to zero
    for (int i = 0; i < big_ring->led_count; i++)
    {
        stored_r[i] = 0;
        stored_g[i] = 0;
        stored_b[i] = 0;
    }

    int filled_leds = 0;      // Track how many LEDs have been filled
    int current_position = 0; // Current position to update

    // Variables for tracking stable color
    uint8_t last_avg_r = 0, last_avg_g = 0, last_avg_b = 0;
    int stable_color_count = 0;

    while (1)
    {
        if (get_raw_data(&r, &g, &b, &c) == ESP_OK)
        {
            // Map raw values to 8-bit RGB
            r_8bit = map_value(r, 0, 65535, 0, 255);
            g_8bit = map_value(g, 0, 65535, 0, 255);
            b_8bit = map_value(b, 0, 65535, 0, 255);

            ESP_LOGI(TAG, "Current RGB: (%u, %u, %u)", r_8bit, g_8bit, b_8bit);

            // Check if black detected
            uint8_t dynamic_black_threshold = (uint8_t)(c * BLACK_THRESHOLD_PERCENTAGE);
            if (r_8bit < dynamic_black_threshold && g_8bit < dynamic_black_threshold && b_8bit < dynamic_black_threshold)
            {
                // Fade all LEDs to black
                ESP_LOGI(TAG, "Black detected - fading to black");
                fade_to_color(big_ring, 0, 0, 0);
                fade_to_color(small_ring, 0, 0, 0);
                //  Reset arrays and counters
                memset(stored_r, 0, sizeof(stored_r));
                memset(stored_g, 0, sizeof(stored_g));
                memset(stored_b, 0, sizeof(stored_b));
                filled_leds = 0;
                current_position = 0;
                stable_color_count = 0;

                vTaskDelay(pdMS_TO_TICKS(500));

                // New thresholds with led turned off
                update_thresholds_from_clear();

                vTaskDelay(pdMS_TO_TICKS(500));

                // Clear any pending interrupts
                ESP_ERROR_CHECK(clear_pending_interrupts());

                //ESP_ERROR_CHECK(enable_interrupt(false));

                vTaskDelay(pdMS_TO_TICKS(50));

                // Deep sleep
                esp_deep_sleep_start();
            }

            // Find closest predefined color
            const RGBColor *closest = get_closest_color(r_8bit, g_8bit, b_8bit);

            // Store the new color in current position
            stored_r[current_position] = closest->r;
            stored_g[current_position] = closest->g;
            stored_b[current_position] = closest->b;

            // Update filled LED count if we haven't filled all LEDs yet
            if (filled_leds < big_ring->led_count)
            {
                filled_leds++;
            }

            // Calculate average of all filled LEDs
            uint32_t sum_r = 0, sum_g = 0, sum_b = 0;
            for (int i = 0; i < filled_leds; i++)
            {
                sum_r += stored_r[i];
                sum_g += stored_g[i];
                sum_b += stored_b[i];
            }
            uint8_t avg_r = sum_r / filled_leds;
            uint8_t avg_g = sum_g / filled_leds;
            uint8_t avg_b = sum_b / filled_leds;

            // Add a 5ms delay after calculating averages
            vTaskDelay(pdMS_TO_TICKS(5));

            // Update all filled LEDs with the average color
            for (int i = 0; i < filled_leds; i++)
            {
                set_single_led(big_ring, i, avg_r, avg_g, avg_b);
            }

            // If we've filled all LEDs, check for stable color
            if (filled_leds == big_ring->led_count)
            {
                // Check if color is the same as last time (with tolerance)
                if (abs(avg_r - last_avg_r) <= TOLERANCE &&
                    abs(avg_g - last_avg_g) <= TOLERANCE &&
                    abs(avg_b - last_avg_b) <= TOLERANCE)
                {
                    stable_color_count++;
                    // ESP_LOGI(TAG, "Same color for %d rounds", stable_color_count);

                    // If color has been stable for enough rounds, send it
                    if (stable_color_count >= STABLE_COLOR_THRESHOLD)
                    {
                        rgb_data.red = avg_r;
                        rgb_data.green = avg_g;
                        rgb_data.blue = avg_b;

                        // Get the closest color name for the average color
                        const RGBColor *final_color = get_closest_color(avg_r, avg_g, avg_b);
                        strncpy(rgb_data.color_name, final_color->name, sizeof(rgb_data.color_name) - 1);
                        rgb_data.color_name[sizeof(rgb_data.color_name) - 1] = '\0'; // Ensure null termination

                        add_rgb_to_send_queue(&rgb_data);
                        // ESP_LOGI(TAG, "Color stable - sent %s RGB: (%u, %u, %u)",
                        //          rgb_data.color_name,
                        //          (unsigned int)avg_r, (unsigned int)avg_g, (unsigned int)avg_b);
                        stable_color_count = 0; // Reset counter after sending
                    }
                }
                else
                {
                    // Color changed, reset counter
                    stable_color_count = 0;
                    ESP_LOGI(TAG, "Color changed - reset stability counter");
                }

                // Update last color
                last_avg_r = avg_r;
                last_avg_g = avg_g;
                last_avg_b = avg_b;
            }

            // Move to next position
            current_position = (current_position + 1) % big_ring->led_count;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

esp_err_t initialize_i2c_bus(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    i2c_master_bus_config_t i2c_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = PULL_UP_RESISTOR_MODE,
    };
    esp_err_t ret = i2c_new_master_bus(&i2c_config, &i2c_bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure I2C bus");
    }

    ESP_LOGI(TAG, "Probing ACTIVE_SENSOR_ADDRESS 0x%02X...", TCS3472_ADDRESS);
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus_handle, TCS3472_ADDRESS, pdMS_TO_TICKS(100)));

    // Configure the device handle for the detected address
    i2c_device_config_t dev_cfg = {
        .device_address = TCS3472_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle));

    return ESP_OK;
}

esp_err_t configure_and_initialize_sensor(gpio_num_t scl_pin, gpio_num_t sda_pin)
{

    i2c_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_semaphore);

    ESP_ERROR_CHECK(initialize_i2c_bus(scl_pin, sda_pin));

    // Initialize the sensor
    ESP_ERROR_CHECK(check_device_id());

    // Power ON
    ESP_ERROR_CHECK(write8(TCS3472_ENABLE, TCS3472_ENABLE_PON));
    vTaskDelay(pdMS_TO_TICKS(3)); // Wait 2.4ms for power up

    // Set integration time and gain
    ESP_ERROR_CHECK(write8(TCS3472_ATIME, TCS3472_INTEGRATION_TIME_700MS));
    ESP_ERROR_CHECK(write8(TCS3472_CONTROL, TCS3472_GAIN_60X));

    // Enable ADC
    ESP_ERROR_CHECK(write8(TCS3472_ENABLE, TCS3472_ENABLE_PON | TCS3472_ENABLE_AEN));
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for ADC to stabilize

    // Set persistence filter
    ESP_ERROR_CHECK(write8(TCS3472_PERS, TCS3472_PERS_1_CYCLE));

    // Enable interrupt
    ESP_ERROR_CHECK(write8(TCS3472_ENABLE, TCS3472_ENABLE_PON | TCS3472_ENABLE_AEN | TCS3472_ENABLE_AIEN));

    // Set interrupt thresholds
    ESP_ERROR_CHECK(update_thresholds_from_clear());


    // Clear any pending interrupts
    ESP_ERROR_CHECK(clear_pending_interrupts());


    return ESP_OK;
}

// start task
void start_data_acquisition_task(NeopixelRing *rings)
{
    xTaskCreate(data_acquisition_task, "data_acquisition_task", 4096, rings, 5, NULL);
}

// Main initialization function
esp_err_t init_chamelamp(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    ESP_LOGI(TAG, "Initializing TCS3472 sensor");

    // Configure the I2C master bus
    ESP_ERROR_CHECK(configure_and_initialize_sensor(scl_pin, sda_pin));

    vTaskDelay(pdMS_TO_TICKS(50));

    // Update thresholds
    update_thresholds_from_clear();

    ESP_LOGI(TAG, "TCS34727 initialized successfully.");

    return ESP_OK;
}
