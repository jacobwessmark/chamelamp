// System and driver dependencies
#include "TCS34725_colorsensor.h"
#include "TCS34725_colorsensor_private.h"
#include "TCS34725_helper_functions.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/rtc_io.h"

static const char *TAG = "TCS34725";

// Low-level I2C Communication Functions
esp_err_t read8(uint8_t reg, uint8_t *value)
{
    uint8_t cmd_buf[1] = {TCS3472_COMMAND_BIT | reg};
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev_handle, cmd_buf, sizeof(cmd_buf), value, 1, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay to ensure stable communication
    return ret;
}

esp_err_t write8(uint8_t reg, uint8_t value)
{
    uint8_t cmd_buf[2] = {TCS3472_COMMAND_BIT | reg, value};
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, cmd_buf, sizeof(cmd_buf), pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay to ensure stable communication
    return ret;
}


esp_err_t check_device_id(void)
{
    uint8_t device_id;
    esp_err_t ret = read8(TCS3472_ID, &device_id);

    if ((ret == ESP_OK) && ((device_id == TCS34727_ID_VALUE) || (device_id == TCS34725_ID_VALUE)))
    {
        ESP_LOGI(TAG, "Device ID confirmed: 0x%02X", device_id);
        return ESP_OK;
    }
    else
    {
        ESP_LOGW(TAG, "Unexpected device ID: 0x%02X", device_id);
    }

    ESP_LOGE(TAG, "Device ID is invalid: 0x%02X", device_id);
    return ESP_FAIL;
}

// Color Data Acquisition Functions
esp_err_t get_raw_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    // Read all color channels (clear, red, green, blue)
    uint8_t data[8];
    ESP_ERROR_CHECK(read8(TCS3472_CDATAL, &data[0]));
    ESP_ERROR_CHECK(read8(TCS3472_CDATAH, &data[1]));
    ESP_ERROR_CHECK(read8(TCS3472_RDATAL, &data[2]));
    ESP_ERROR_CHECK(read8(TCS3472_RDATAH, &data[3]));
    ESP_ERROR_CHECK(read8(TCS3472_GDATAL, &data[4]));
    ESP_ERROR_CHECK(read8(TCS3472_GDATAH, &data[5]));
    ESP_ERROR_CHECK(read8(TCS3472_BDATAL, &data[6]));
    ESP_ERROR_CHECK(read8(TCS3472_BDATAH, &data[7]));

    // Combine high and low bytes for each channel
    *c = (data[1] << 8) | data[0];
    *r = (data[3] << 8) | data[2];
    *g = (data[5] << 8) | data[4];
    *b = (data[7] << 8) | data[6];

    return ESP_OK;
}

// Interrupt Configuration Functions
esp_err_t set_interrupt_thresholds(uint16_t low_threshold, uint16_t high_threshold)
{
    ESP_LOGI(TAG, "Setting interrupt thresholds - Low: %u, High: %u", low_threshold, high_threshold);

    // Configure low threshold registers
    ESP_ERROR_CHECK(write8(TCS3472_AILTL, low_threshold & 0xFF));
    ESP_ERROR_CHECK(write8(TCS3472_AILTH, (low_threshold >> 8) & 0xFF));

    // Configure high threshold registers
    ESP_ERROR_CHECK(write8(TCS3472_AIHTL, high_threshold & 0xFF));
    ESP_ERROR_CHECK(write8(TCS3472_AIHTH, (high_threshold >> 8) & 0xFF));

    return ESP_OK;
}

// Power Management Functions
esp_err_t power_on_sensor(bool power_on)
{
    uint8_t value = power_on ? TCS3472_ENABLE_PON : 0x00;
    write8(TCS3472_ENABLE, value);
    vTaskDelay(pdMS_TO_TICKS(3)); // Power-on stabilization delay
    return ESP_OK;
}

// Sensor Configuration Functions
esp_err_t set_integration_time(uint8_t integration_time)
{
    return write8(TCS3472_ATIME, integration_time);
}

esp_err_t set_gain(uint8_t gain)
{
    return write8(TCS3472_CONTROL, gain);
}

esp_err_t set_persistence_filter(uint8_t persistence_value)
{
    return write8(TCS3472_PERS, persistence_value);
}

esp_err_t enable_adc(bool enable)
{
    uint8_t reg_value = enable ? TCS3472_ENABLE_AEN : 0x00;
    write8(TCS3472_ENABLE, reg_value);
    vTaskDelay(pdMS_TO_TICKS(50)); // ADC stabilization delay
    return ESP_OK;
}

// Interrupt Management Functions
esp_err_t clear_pending_interrupts()
{
    uint8_t status;
    esp_err_t ret = read8(TCS3472_STATUS, &status);
    if (ret != ESP_OK)
    {
        return ret;
    }
    ret = write8(TCS3472_CMD_CLEAR_INT, 0x00);
    if (ret != ESP_OK)
    {
        return ret;
    }
    return ESP_OK;
}

esp_err_t enable_interrupt(bool enable)
{
    uint8_t value = enable ? TCS3472_ENABLE_AIEN : 0x00;
    return write8(TCS3472_ENABLE, value);
}

// I2C Driver Management Functions
esp_err_t deinitialize_i2c_driver(i2c_master_bus_handle_t i2c_master_bus_handle)
{
    esp_err_t ret = i2c_del_master_bus(i2c_master_bus_handle);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "I2C bus uninstalled successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to uninstall I2C bus, error: %s", esp_err_to_name(ret));
    }

    return ret;
}

// Power Management Functions
void enable_wakeup(gpio_num_t int_pin)
{
    ESP_LOGI(TAG, "Configuring wake and sleep functionality");

    // Configure RTC GPIO for interrupt-based wakeup
    rtc_gpio_init(int_pin);
    rtc_gpio_set_direction(int_pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(int_pin);
    rtc_gpio_pullup_en(int_pin);

    // Configure wakeup sources
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(int_pin, 0));
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(THRESHOLD_UPDATE_INTERVAL_US));
}

void deep_sleep_with_deinitialized_i2c(i2c_master_bus_handle_t i2c_master_bus_handle)
{
    deinitialize_i2c_driver(i2c_master_bus_handle);
    vTaskDelay(pdMS_TO_TICKS(50)); // Allow time for cleanup

    ESP_LOGI(TAG, "Entering deep sleep mode");
    esp_deep_sleep_start();
}
