#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

// LCD Command Set
// Basic Display Control Commands
#define LCD_CMD_CLEAR_DISPLAY 0x01    // Clear all display data
#define LCD_CMD_RETURN_HOME 0x02      // Return cursor to home position
#define LCD_CMD_ENTRY_MODE_SET 0x04   // Set cursor move direction and display shift
#define LCD_CMD_DISPLAY_CONTROL 0x08  // Control display, cursor, and blink
#define LCD_CMD_CURSOR_SHIFT 0x10     // Move cursor and shift display
#define LCD_CMD_FUNCTION_SET 0x20     // Set interface data length, lines, and font
#define LCD_CMD_SET_CGRAM_ADDR 0x40   // Set CGRAM address for custom characters
#define LCD_CMD_SET_DDRAM_ADDR 0x80   // Set DDRAM address for display data

// Display Control Functions
// Animation Control
void lcd_start_animation(void);        // Start display animation sequence
void lcd_stop_animation(void);         // Stop current animation sequence

// Core Display Functions
void lcd_init(gpio_num_t scl_pin, gpio_num_t sda_pin);     // Initialize LCD with specified I2C pins
void lcd_display_text(const char *text, uint8_t row);       // Display text on specified row (0 or 1)
void lcd_send_command(uint8_t command);                     // Send direct command to LCD

