# LCD1602 Display Component Configuration
# Implements LCD display control and animation functionality
# Dependencies: ESP-IDF driver for hardware interface
idf_component_register(SRCS "lcd1602.c" "lcd_animation.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
