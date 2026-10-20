idf_component_register(SRCS "main.cpp" "max30102.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_i2c esp_driver_gpio)