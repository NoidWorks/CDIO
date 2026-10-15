idf_component_register(
    SRCS "lidar.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart
        esp_driver_gpio
)