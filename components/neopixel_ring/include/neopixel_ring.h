# pragma once

#include "neopixel.h"
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define FADE_STEPS 50 // Number of steps in fade animation
#define FADE_DELAY 10 // Delay between steps in milliseconds

// NeopixelRing structure
typedef struct
{
    tNeopixelContext *context;
    uint32_t led_count;
    uint8_t current_red;
    uint8_t current_green;
    uint8_t current_blue;
    int gpio_pin;
} NeopixelRing;

void init_neopixel_rings(NeopixelRing *rings, size_t ring_count);
void init_neopixel_ring(NeopixelRing *ring);
void set_single_led(NeopixelRing *ring, int led_index, uint8_t r, uint8_t g, uint8_t b);
void turn_off_all_rings(NeopixelRing *rings, size_t ring_count);
void fade_to_color(NeopixelRing *ring, uint8_t target_r, uint8_t target_g, uint8_t target_b);
