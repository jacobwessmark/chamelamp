idf_component_register(SRCS "esp_now_sender.c" "esp_now_receiver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES TCS34725_colorsensor esp_wifi nvs_flash lcd1602)
