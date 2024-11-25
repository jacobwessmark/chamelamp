# First, determine the required components based on device type
if(CONFIG_DEVICE_MODE_SENDER)
    set(EXTRA_REQUIRES TCS34725_colorsensor neopixel_ring)
elseif(CONFIG_DEVICE_MODE_RECEIVER)
    set(EXTRA_REQUIRES lcd1602)
endif()

# Register the component with all required dependencies
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_now_handler ${EXTRA_REQUIRES}
)