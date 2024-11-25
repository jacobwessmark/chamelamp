#pragma once

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM I2C_NUM_0 // Specifies the I2C port number (0 in this case)
#define I2C_LCD_FREQ_HZ 400000   // Sets the I2C clock speed to 400 kHz (for faster communication)
#define LCD_ADDR 0x27            // I2C address of the PCF8574 I/O expander connected to the LCD       // Defines the UART port number (0 for default serial communication)
#define LCD_BACKLIGHT 0x08       // Bit that controls the LCD backlight (when set, the backlight is on)


// Flags for controlling entry mode on the LCD
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// Flags for controlling the display, cursor, and blinking settings
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// Flags for controlling display or cursor shift settings
#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

// Flags for LCD function set mode (bit mode, line number, and font size)
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10_DOTS 0x04
#define LCD_5x8_DOTS 0x00

// Private functions
void lcd_send_command(uint8_t command);
void lcd_send_data(uint8_t nibble);
void lcd_write_char(char ch);
void i2c_master_init(gpio_num_t scl_pin, gpio_num_t sda_pin);