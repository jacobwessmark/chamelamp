# Main Application Component Configuration
# Determines required components based on device operating mode

# Component Selection Logic
if(CONFIG_DEVICE_MODE_SENDER)
    # Sender mode requires color sensor and LED ring functionality
    set(EXTRA_REQUIRES TCS34725_colorsensor neopixel_ring)
elseif(CONFIG_DEVICE_MODE_RECEIVER)
    # Receiver mode requires LCD display functionality
    set(EXTRA_REQUIRES lcd1602)
endif()

# Register Main Component
# Core dependencies: ESP-NOW communication and NVS storage
# Additional dependencies are determined by device operating mode
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_now_handler ${EXTRA_REQUIRES} nvs_handler
)