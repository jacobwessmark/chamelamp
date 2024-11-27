# Non-Volatile Storage (NVS) Handler Component Configuration
# Provides persistent storage functionality for device configuration and data
idf_component_register(SRCS "nvs_handler.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "nvs_flash") 