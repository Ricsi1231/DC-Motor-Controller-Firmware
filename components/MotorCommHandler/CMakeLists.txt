idf_component_register(
    SRCS "src/MotorCommHandler.cpp"
    INCLUDE_DIRS "include"
    REQUIRES USB
    PRIV_REQUIRES motorControl Encoder PID DRV8876
)
