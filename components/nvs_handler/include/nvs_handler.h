#pragma once

#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

// NVS Storage Management Functions

// Initialize NVS flash storage system
// Returns ESP_OK if successful, error code otherwise
esp_err_t nvs_handler_init(void);

// Store RGB color data in NVS
// Parameters:
//   red, green, blue: Color component values (0-255)
//   color_name: String identifier for the color
// Returns ESP_OK if successful, error code otherwise
esp_err_t nvs_handler_save_color(uint8_t red, uint8_t green, uint8_t blue, const char* color_name);

// Retrieve RGB color data from NVS
// Parameters:
//   red, green, blue: Pointers to store color components
//   color_name: Buffer to store color identifier
//   name_size: Size of color_name buffer
// Returns ESP_OK if successful, error code otherwise
esp_err_t nvs_handler_load_color(uint8_t* red, uint8_t* green, uint8_t* blue, 
                               char* color_name, size_t name_size);