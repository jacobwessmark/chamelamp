# TCS34725 Color Sensor Component Configuration
# Source files: Main sensor driver and helper functions
# Dependencies: ESP-IDF driver, NeoPixel ring control, and ESP-NOW communication
idf_component_register(SRCS "TCS34725_colorsensor.c" "TCS34725_helper_functions.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver neopixel_ring esp_now_handler)