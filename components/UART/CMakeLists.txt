idf_component_register(
    SRCS "src/UartBus.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio Interfaces
)
