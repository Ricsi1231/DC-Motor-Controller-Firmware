idf_component_register(
    SRCS "src/motorControl.cpp"
    INCLUDE_DIRS "include"
    REQUIRES PID DRV8876 Encoder
)