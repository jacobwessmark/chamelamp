#include "neopixel_ring.h"

static const char *TAG = "NEOPIXEL_RING";

// Define the NeopixelRing structure with stored color values

void fade_to_color(NeopixelRing *ring, uint8_t target_r, uint8_t target_g, uint8_t target_b)
{
    tNeopixel pixels[ring->led_count];

    // Initialize all pixels with their current values
    for (int i = 0; i < ring->led_count; i++)
    {
        pixels[i].index = i;
        pixels[i].rgb = NP_RGB(ring->current_red, ring->current_green, ring->current_blue);
    }

    // Fade each LED to target color
    for (int step = 0; step < FADE_STEPS; step++)
    {
        for (int i = 0; i < ring->led_count; i++)
        {
            uint8_t current_r = NP_RGB2RED(pixels[i].rgb);
            uint8_t current_g = NP_RGB2GREEN(pixels[i].rgb);
            uint8_t current_b = NP_RGB2BLUE(pixels[i].rgb);

            // Calculate intermediate color
            uint8_t new_r = current_r + ((target_r - current_r) * step) / FADE_STEPS;
            uint8_t new_g = current_g + ((target_g - current_g) * step) / FADE_STEPS;
            uint8_t new_b = current_b + ((target_b - current_b) * step) / FADE_STEPS;

            pixels[i].rgb = NP_RGB(new_r, new_g, new_b);
        }

        // Update all LEDs at once
        neopixel_SetPixel(ring->context, pixels, ring->led_count);

        vTaskDelay(pdMS_TO_TICKS(FADE_DELAY));
    }

    // Update the ring's stored RGB values
    ring->current_red = target_r;
    ring->current_green = target_g;
    ring->current_blue = target_b;
}

void set_single_led(NeopixelRing *ring, int led_index, uint8_t r, uint8_t g, uint8_t b)
{
    if (led_index < 0 || led_index >= ring->led_count)
        return;

    // Create pixel structure
    tNeopixel pixel = {
        .index = led_index,
        .rgb = NP_RGB(r, g, b)};

    // Update single LED using neopixel_SetPixel
    neopixel_SetPixel(ring->context, &pixel, 1);

    // Store the current color values
    ring->current_red = r;
    ring->current_green = g;
    ring->current_blue = b;
}

void init_neopixel_rings(NeopixelRing *rings, size_t ring_count)
{
    for (size_t i = 0; i < ring_count; i++)
    {
        init_neopixel_ring(&rings[i]);
    }
}

// Initialize the NeoPixel ring
void init_neopixel_ring(NeopixelRing *ring)
{
    ring->context = neopixel_Init(ring->led_count, ring->gpio_pin);
    ring->current_red = 0;
    ring->current_green = 0;
    ring->current_blue = 0;

    if (!ring->context)
    {
        ESP_LOGE(TAG, "Failed to initialize NeoPixel ring on GPIO %d", ring->gpio_pin);
    }
    else
    {
        ESP_LOGI(TAG, "NeoPixel ring initialized");
    }
}

// Fading function to transition from the current to the target color
void fade_in_ring_color(NeopixelRing *ring, uint8_t target_red, uint8_t target_green, uint8_t target_blue)
{
    if (!ring->context)
        return;

    tNeopixel pixels[ring->led_count];
    uint8_t start_red = ring->current_red;
    uint8_t start_green = ring->current_green;
    uint8_t start_blue = ring->current_blue;

    uint8_t step_count = 20; // Number of steps for fading

    // Transition gradually from the current color to the target color
    for (uint8_t step = 1; step <= step_count; step++)
    {
        // Calculate intermediate color based on step
        uint8_t red = start_red + ((target_red - start_red) * step) / step_count;
        uint8_t green = start_green + ((target_green - start_green) * step) / step_count;
        uint8_t blue = start_blue + ((target_blue - start_blue) * step) / step_count;

        // Set each LED in the ring to the intermediate color
        for (uint32_t i = 0; i < ring->led_count; i++)
        {
            pixels[i].index = i;
            pixels[i].rgb = NP_RGB(red, green, blue);
        }
        neopixel_SetPixel(ring->context, pixels, ring->led_count);

        // Small delay to create fade effect
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed for faster or slower fade
    }

    // Update the current color to the new target color
    ring->current_red = target_red;
    ring->current_green = target_green;
    ring->current_blue = target_blue;
}

void turn_off_all_rings(NeopixelRing *rings, size_t ring_count)
{
    for (size_t i = 0; i < ring_count; i++)
    {
        fade_to_color(&rings[i], 0, 0, 0);
    }
}
