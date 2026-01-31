idf_component_register(
    SRCS "src/MotorCommHandler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES Interfaces USB
    PRIV_REQUIRES esp_timer
)
