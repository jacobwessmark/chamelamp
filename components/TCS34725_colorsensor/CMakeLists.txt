idf_component_register(SRCS "TCS34725_colorsensor.c" "TCS34725_helper_functions.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver neopixel_ring esp_now_handler)