#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

// LCD command definitions, used to control various functions and settings of the LCD
#define LCD_CMD_CLEAR_DISPLAY 0x01
#define LCD_CMD_RETURN_HOME 0x02
#define LCD_CMD_ENTRY_MODE_SET 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_CURSOR_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM_ADDR 0x40
#define LCD_CMD_SET_DDRAM_ADDR 0x80

// Public functions
void lcd_init(gpio_num_t scl_pin, gpio_num_t sda_pin);
void lcd_display_text(const char *text, uint8_t row);
void lcd_send_command(uint8_t command);

