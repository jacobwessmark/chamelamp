# NeoPixel Ring Control Component Configuration
# Provides high-level control interface for NeoPixel LED rings
# Dependencies: Zorxx NeoPixel driver for low-level LED control
idf_component_register(SRCS "neopixel_ring.c"
                    INCLUDE_DIRS "include"
                    REQUIRES zorxx__neopixel)
