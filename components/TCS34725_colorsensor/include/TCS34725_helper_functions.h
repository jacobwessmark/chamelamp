#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Function prototypes for TCS34725 helper functions

// Function to read an 8-bit register
esp_err_t read8(uint8_t reg, uint8_t *value);

// Function to write an 8-bit register
esp_err_t write8(uint8_t reg, uint8_t value);

// Function to configure the sensor for wait state
esp_err_t configure_wait_state(bool enable_long_wait, uint8_t wait_time);

// Check device ID to confirm sensor presence
esp_err_t check_device_id(void);

// Read raw color data
esp_err_t get_raw_data(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

// Helper function to set interrupt thresholds
esp_err_t set_interrupt_thresholds(uint16_t low_threshold, uint16_t high_threshold);

// Helper function to power on/off the sensor
esp_err_t power_on_sensor(bool power_on);

// Helper function to set integration time
esp_err_t set_integration_time(uint8_t integration_time);

// Helper function to set gain
esp_err_t set_gain(uint8_t gain);

// Helper function to set persistence filter
esp_err_t set_persistence_filter(uint8_t persistence_value);

// Enable ADC functionality
esp_err_t enable_adc(bool enable);

// Clear any pending interrupts
esp_err_t clear_pending_interrupts(void);

// Enable or disable interrupt
esp_err_t enable_interrupt(bool enable);