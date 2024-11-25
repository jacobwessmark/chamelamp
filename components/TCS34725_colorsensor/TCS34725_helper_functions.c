#include "TCS34725_colorsensor.h"
#include "TCS34725_colorsensor_private.h"
#include "TCS34725_helper_functions.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "TCS34725";

// Function to read an 8-bit register
esp_err_t read8(uint8_t reg, uint8_t *value)
{
    uint8_t cmd_buf[1] = {TCS3472_COMMAND_BIT | reg};
    esp_err_t ret = i2c_master_transmit_receive(i2c_dev_handle, cmd_buf, sizeof(cmd_buf), value, 1, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay after write
    return ret;
}

// Function to write an 8-bit register
esp_err_t write8(uint8_t reg, uint8_t value)
{
    uint8_t cmd_buf[2] = {TCS3472_COMMAND_BIT | reg, value};
    esp_err_t ret = i2c_master_transmit(i2c_dev_handle, cmd_buf, sizeof(cmd_buf), pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(5)); // Small delay after write
    return ret;
}

// Function to configure the sensor for wait state
esp_err_t configure_wait_state(bool enable_long_wait, uint8_t wait_time)
{
    esp_err_t ret;

    // Enable wait state by setting the WEN bit in the Enable register
    uint8_t enable_reg;
    ret = read8(TCS3472_ENABLE, &enable_reg);
    if (ret != ESP_OK)
        return ret;

    enable_reg |= TCS3472_ENABLE_WEN;
    ret = write8(TCS3472_ENABLE, enable_reg);
    if (ret != ESP_OK)
        return ret;

    // Set the wait time in the WTIME register using the provided define
    ret = write8(TCS3472_WTIME, wait_time);
    if (ret != ESP_OK)
        return ret;

    // Optionally set the WLONG bit in the Configuration register
    if (enable_long_wait)
    {
        uint8_t config_reg;
        ret = read8(TCS3472_CONFIG, &config_reg);
        if (ret != ESP_OK)
            return ret;

        config_reg |= TCS3472_ENABLE_WEN; // Ensure this is the correct bit for WLONG
        ret = write8(TCS3472_CONFIG, config_reg);
        if (ret != ESP_OK)
            return ret;
    }

    return ESP_OK;
}

// Check device ID to confirm sensor presence
esp_err_t check_device_id()
{
    uint8_t device_id = 0;
    ESP_LOGI(TAG, "Reading device ID from register 0x12...");

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

// Read raw color data
esp_err_t get_raw_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
    uint8_t data[8];
    ESP_ERROR_CHECK(read8(TCS3472_CDATAL, &data[0]));
    ESP_ERROR_CHECK(read8(TCS3472_CDATAH, &data[1]));
    ESP_ERROR_CHECK(read8(TCS3472_RDATAL, &data[2]));
    ESP_ERROR_CHECK(read8(TCS3472_RDATAH, &data[3]));
    ESP_ERROR_CHECK(read8(TCS3472_GDATAL, &data[4]));
    ESP_ERROR_CHECK(read8(TCS3472_GDATAH, &data[5]));
    ESP_ERROR_CHECK(read8(TCS3472_BDATAL, &data[6]));
    ESP_ERROR_CHECK(read8(TCS3472_BDATAH, &data[7]));

    *c = (data[1] << 8) | data[0];
    *r = (data[3] << 8) | data[2];
    *g = (data[5] << 8) | data[4];
    *b = (data[7] << 8) | data[6];

    return ESP_OK;
}

// Helper function to set interrupt thresholds
esp_err_t set_interrupt_thresholds(uint16_t low_threshold, uint16_t high_threshold)
{
    ESP_LOGI(TAG, "Setting interrupt thresholds - Low: %u, High: %u", low_threshold, high_threshold);

    // Set low threshold (split into low and high bytes)
    ESP_ERROR_CHECK(write8(TCS3472_AILTL, low_threshold & 0xFF));        // Low byte
    ESP_ERROR_CHECK(write8(TCS3472_AILTH, (low_threshold >> 8) & 0xFF)); // High byte

    // Set high threshold (split into low and high bytes)
    ESP_ERROR_CHECK(write8(TCS3472_AIHTL, high_threshold & 0xFF));        // Low byte
    ESP_ERROR_CHECK(write8(TCS3472_AIHTH, (high_threshold >> 8) & 0xFF)); // High byte

    return ESP_OK;
}
// Helper function to power on/off the sensor
esp_err_t power_on_sensor(bool power_on)
{
    uint8_t value = power_on ? TCS3472_ENABLE_PON : 0x00;
    write8(TCS3472_ENABLE, value);
    vTaskDelay(pdMS_TO_TICKS(3)); // Wait 2.4ms for power up
    return ESP_OK;
}

// Helper function to set integration time
esp_err_t set_integration_time(uint8_t integration_time)
{
    return write8(TCS3472_ATIME, integration_time);
}

// Helper function to set gain
esp_err_t set_gain(uint8_t gain)
{
    return write8(TCS3472_CONTROL, gain);
}

// Helper function to set persistence filter
esp_err_t set_persistence_filter(uint8_t persistence_value)
{
    return write8(TCS3472_PERS, persistence_value);
}

// Enable ADC functionality
esp_err_t enable_adc(bool enable)
{
    uint8_t reg_value = enable ? TCS3472_ENABLE_AEN : 0x00;
    write8(TCS3472_ENABLE, reg_value);
    vTaskDelay(pdMS_TO_TICKS(50)); // Wait for ADC to stabilize
    return ESP_OK;
}

// Clear any pending interrupts
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

