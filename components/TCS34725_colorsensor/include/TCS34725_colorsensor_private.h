#pragma once

// Required includes for private implementation
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "TCS34725_colorsensor.h"
#include <string.h>
#include "esp_sleep.h"
#include <inttypes.h>

// I2C definitions
#define I2C_MASTER_FREQ_HZ 200000
#define I2C_MASTER_PORT I2C_NUM_0
#define TRANS_QUEUE_DEPTH 10
#define PULL_UP_RESISTOR_MODE true

// TCS34725 register definitions
#define TCS3472_ADDRESS 0x29
#define TCS34727_ID_VALUE 0x4D
#define TCS34725_ID_VALUE 0x44
#define TCS3472_ID 0x12
#define TCS3472_COMMAND_BIT 0x80

// Register addresses
#define TCS3472_ENABLE 0x00
#define TCS3472_ATIME 0x01
#define TCS3472_WTIME 0x03
#define TCS3472_CONFIG 0x0D
#define TCS3472_CONTROL 0x0F
#define TCS3472_STATUS 0x13
#define TCS3472_PERS 0x0C

// Color Data Registers
#define TCS3472_CDATAL 0x14
#define TCS3472_CDATAH 0x15
#define TCS3472_RDATAL 0x16
#define TCS3472_RDATAH 0x17
#define TCS3472_GDATAL 0x18
#define TCS3472_GDATAH 0x19
#define TCS3472_BDATAL 0x1A
#define TCS3472_BDATAH 0x1B

// Interrupt Threshold Registers
#define TCS3472_AILTL 0x04
#define TCS3472_AILTH 0x05
#define TCS3472_AIHTL 0x06
#define TCS3472_AIHTH 0x07

// Persistence register values
#define TCS3472_PERS_NONE 0b0000
#define TCS3472_PERS_1_CYCLE 0b0001
#define TCS3472_PERS_2_CYCLE 0b0010
#define TCS3472_PERS_3_CYCLE 0b0011
#define TCS3472_PERS_5_CYCLE 0b0100
#define TCS3472_PERS_10_CYCLE 0b0101

// Enable Register Bits
#define TCS3472_ENABLE_PON 0x01
#define TCS3472_ENABLE_AEN 0x02
#define TCS3472_ENABLE_WEN 0x03
#define TCS3472_ENABLE_AIEN 0x10

// Gain settings
#define TCS3472_GAIN_1X 0x00
#define TCS3472_GAIN_4X 0x01
#define TCS3472_GAIN_16X 0x02
#define TCS3472_GAIN_60X 0x03

// Integration times
#define TCS3472_INTEGRATION_TIME_2_4MS 0xFF
#define TCS3472_INTEGRATION_TIME_5_4MS 0xF6
#define TCS3472_INTEGRATION_TIME_13_7MS 0xEB
#define TCS3472_INTEGRATION_TIME_101MS 0xD5
#define TCS3472_INTEGRATION_TIME_154MS 0xC0
#define TCS3472_INTEGRATION_TIME_700MS 0x00

// Interrupt thresholds
#define CLEAR_THRESHOLD_DARK 120
#define CLEAR_THRESHOLD_BRIGHT 3000
#define TCS3472_CMD_CLEAR_INT 0xE7
#define TCS3472_STATUS_AINT 0x10

// Threshold calculation defines
#define THRESHOLD_PERCENTAGE_BELOW 80
#define THRESHOLD_PERCENTAGE_ABOVE 120
#define THRESHOLD_MIN_DIFFERENCE 1000
#define THRESHOLD_MIN_VALUE 100
#define THRESHOLD_MAX_VALUE 65000
#define THRESHOLD_SAFETY_MARGIN 500

// Event group bits (defined only once)
#define SENSOR_INT_BIT (1 << 0)

// Retry settings
#define MAX_RETRIES 3
#define RETRY_DELAY_US 50000

// Sampling configuration
#define SAMPLE_NUMBER 5

// Number of colors in the rgb_colors array
#define NUM_RGB_COLORS (sizeof(rgb_colors) / sizeof(rgb_colors[0]))

// Stable color threshold and tolerance
#define STABLE_COLOR_THRESHOLD 100 // Required matches before sending
#define TOLERANCE 5                // Allow small variations in color

// Define the percentage threshold for detecting black color
// This value is used to determine if the RGB values are low enough to be considered black
// How to use: 0.01 = 1% of the max value (65535), so if lower than 655.35, it's black
#define BLACK_THRESHOLD_PERCENTAGE 0.05

// Color array declaration
extern RGBColor rgb_colors[];

// Add near the top with other static variables
extern gpio_num_t tcs_scl_pin;
extern gpio_num_t tcs_sda_pin;
extern gpio_num_t tcs_int_pin;

// I2C semaphore
extern SemaphoreHandle_t i2c_semaphore;

// Private function prototypes
esp_err_t check_device_id(void);
esp_err_t write8(uint8_t reg, uint8_t value);
esp_err_t read8(uint8_t reg, uint8_t *value);
esp_err_t set_interrupt_thresholds(uint16_t low_threshold, uint16_t high_threshold);
esp_err_t update_thresholds_from_clear(void);
uint8_t map_value(uint16_t x, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max);
esp_err_t configure_and_initialize_sensor(gpio_num_t scl_pin, gpio_num_t sda_pin);
