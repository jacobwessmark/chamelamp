#pragma once

#include "neopixel.h"
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Animation Configuration
#define FADE_STEPS 50     // Number of interpolation steps for smooth color transitions
#define FADE_DELAY 10     // Delay between fade steps (milliseconds)

// NeoPixel Ring Control Structure
typedef struct
{
    tNeopixelContext *context;    // Low-level NeoPixel driver context
    uint32_t led_count;           // Number of LEDs in the ring
    uint8_t current_red;          // Current red intensity (0-255)
    uint8_t current_green;        // Current green intensity (0-255)
    uint8_t current_blue;         // Current blue intensity (0-255)
    int gpio_pin;                 // GPIO pin number for data output
} NeopixelRing;

// Ring Control Functions
void init_neopixel_rings(NeopixelRing *rings, size_t ring_count);    // Initialize multiple NeoPixel rings
void init_neopixel_ring(NeopixelRing *ring);                         // Initialize single NeoPixel ring
void set_single_led(NeopixelRing *ring, int led_index,               // Set color of individual LED
                   uint8_t r, uint8_t g, uint8_t b);
void turn_off_all_rings(NeopixelRing *rings, size_t ring_count);     // Turn off all LEDs in all rings
void fade_to_color(NeopixelRing *ring,                               // Smoothly transition to new color
                  uint8_t target_r, uint8_t target_g, uint8_t target_b);
