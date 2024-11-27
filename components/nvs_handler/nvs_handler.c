#include <string.h>
#include "nvs_handler.h"
#include "esp_log.h"

static const char *TAG = "nvs_handler";
static nvs_handle_t s_nvs_handle;

esp_err_t nvs_handler_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Open NVS handle
    ret = nvs_open("color_data", NVS_READWRITE, &s_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "NVS handler initialized successfully");
    return ESP_OK;
}

esp_err_t nvs_handler_save_color(uint8_t red, uint8_t green, uint8_t blue, const char* color_name)
{
    esp_err_t ret;

    // Save RGB values
    ret = nvs_set_u8(s_nvs_handle, "last_red", red);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(s_nvs_handle, "last_green", green);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(s_nvs_handle, "last_blue", blue);
    if (ret != ESP_OK) return ret;

    // Save color name
    ret = nvs_set_str(s_nvs_handle, "last_color_name", color_name);
    if (ret != ESP_OK) return ret;

    // Commit changes
    ret = nvs_commit(s_nvs_handle);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Color saved to NVS: RGB(%u,%u,%u)-%s", red, green, blue, color_name);
    return ESP_OK;
}

esp_err_t nvs_handler_load_color(uint8_t* red, uint8_t* green, uint8_t* blue, char* color_name, size_t name_size)
{
    esp_err_t ret;

    // Load RGB values
    ret = nvs_get_u8(s_nvs_handle, "last_red", red);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) return ret;

    ret = nvs_get_u8(s_nvs_handle, "last_green", green);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) return ret;

    ret = nvs_get_u8(s_nvs_handle, "last_blue", blue);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) return ret;

    // Load color name
    size_t required_size;
    ret = nvs_get_str(s_nvs_handle, "last_color_name", NULL, &required_size);
    if (ret == ESP_OK) {
        if (required_size <= name_size) {
            ret = nvs_get_str(s_nvs_handle, "last_color_name", color_name, &name_size);
            if (ret != ESP_OK) return ret;
        } else {
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        return ret;
    }

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // Set default values if no color is saved
        *red = 0;
        *green = 0;
        *blue = 0;
        strncpy(color_name, "None", name_size);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Color loaded from NVS: RGB(%u,%u,%u)-%s", *red, *green, *blue, color_name);
    return ESP_OK;
}