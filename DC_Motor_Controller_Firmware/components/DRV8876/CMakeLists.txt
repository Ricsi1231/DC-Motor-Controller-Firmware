idf_component_register(
    SRCS "src/DRV8876.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_ledc esp_driver_gpio
)