# ESP-NOW Communication Handler Component Configuration
# Implements sender and receiver functionality for wireless data transfer
# Dependencies: ESP WiFi, NVS Flash, LCD display, NVS handler, and Color sensor components
idf_component_register(SRCS "esp_now_sender.c" "esp_now_receiver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi nvs_flash lcd1602 nvs_handler TCS34725_colorsensor wifi_manager)
