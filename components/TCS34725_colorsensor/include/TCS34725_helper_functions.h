#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Low-level I2C Communication Functions
esp_err_t read8(uint8_t reg, uint8_t *value);
esp_err_t write8(uint8_t reg, uint8_t value);

// Sensor Configuration Functions
esp_err_t configure_wait_state(bool enable_long_wait, uint8_t wait_time);
esp_err_t check_device_id(void);
esp_err_t set_integration_time(uint8_t integration_time);
esp_err_t set_gain(uint8_t gain);
esp_err_t set_persistence_filter(uint8_t persistence_value);
esp_err_t enable_adc(bool enable);

// Color Data Acquisition Functions
esp_err_t get_raw_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

// Interrupt Configuration and Management Functions
esp_err_t set_interrupt_thresholds(uint16_t low_threshold, uint16_t high_threshold);
esp_err_t enable_interrupt(bool enable);
esp_err_t clear_pending_interrupts(void);

// Power Management Functions
esp_err_t power_on_sensor(bool power_on);
void enable_wakeup(gpio_num_t int_pin);
void deep_sleep_with_deinitialized_i2c(i2c_master_bus_handle_t i2c_master_bus_handle);

// I2C Driver Management Functions
esp_err_t deinitialize_i2c_driver(i2c_master_bus_handle_t i2c_master_bus_handle);