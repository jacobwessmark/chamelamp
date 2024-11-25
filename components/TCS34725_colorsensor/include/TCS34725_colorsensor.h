#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "rgb_types.h"
#include "neopixel_ring.h"
#include "driver/i2c_master.h"

extern i2c_master_bus_handle_t i2c_bus_handle;
extern i2c_master_dev_handle_t i2c_dev_handle;

// Public function prototypes (API)
esp_err_t get_raw_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
const RGBColor *get_closest_color(uint8_t avg_r, uint8_t avg_g, uint8_t avg_b);
esp_err_t update_thresholds_from_clear(void);
esp_err_t initialize_i2c_bus(gpio_num_t scl_pin, gpio_num_t sda_pin);
esp_err_t init_chamelamp(gpio_num_t scl_pin, gpio_num_t sda_pin);
void start_data_acquisition_task(NeopixelRing *rings);
