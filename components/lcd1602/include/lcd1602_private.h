#pragma once

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// I2C Communication Configuration
#define I2C_MASTER_NUM I2C_NUM_0      // I2C port number for LCD communication
#define I2C_LCD_FREQ_HZ 400000        // I2C clock frequency (400 kHz)
#define LCD_ADDR 0x27                 // LCD I2C address (PCF8574 I/O expander)
#define LCD_BACKLIGHT 0x08            // Backlight control bit

// LCD Entry Mode Configuration
#define LCD_ENTRY_RIGHT 0x00          // Text direction: right to left
#define LCD_ENTRY_LEFT 0x02           // Text direction: left to right
#define LCD_ENTRY_SHIFT_INCREMENT 0x01 // Auto-shift display right
#define LCD_ENTRY_SHIFT_DECREMENT 0x00 // Auto-shift display left

// Display Control Configuration
#define LCD_DISPLAY_ON 0x04           // Display enabled
#define LCD_DISPLAY_OFF 0x00          // Display disabled
#define LCD_CURSOR_ON 0x02            // Cursor visible
#define LCD_CURSOR_OFF 0x00           // Cursor hidden
#define LCD_BLINK_ON 0x01             // Cursor blink enabled
#define LCD_BLINK_OFF 0x00            // Cursor blink disabled

// Display/Cursor Movement Configuration
#define LCD_DISPLAY_MOVE 0x08         // Move entire display
#define LCD_CURSOR_MOVE 0x00          // Move cursor only
#define LCD_MOVE_RIGHT 0x04           // Move/shift right
#define LCD_MOVE_LEFT 0x00            // Move/shift left

// LCD Interface Configuration
#define LCD_8BIT_MODE 0x10            // 8-bit data interface
#define LCD_4BIT_MODE 0x00            // 4-bit data interface
#define LCD_2LINE 0x08                // Two-line display
#define LCD_1LINE 0x00                // Single-line display
#define LCD_5x10_DOTS 0x04            // 5x10 character font
#define LCD_5x8_DOTS 0x00             // 5x8 character font

// Internal LCD Control Functions
void lcd_send_command(uint8_t command);    // Send control command to LCD
void lcd_send_data(uint8_t nibble);        // Send 4-bit data to LCD
void lcd_write_char(char ch);              // Write single character to LCD
void i2c_master_init(gpio_num_t scl_pin, gpio_num_t sda_pin);  // Initialize I2C master interface