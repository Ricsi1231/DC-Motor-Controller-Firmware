idf_component_register(
    SRCS "src/Encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_pcnt esp_timer
)