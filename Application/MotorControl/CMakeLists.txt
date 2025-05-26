idf_component_register(
    SRCS "src/MotorControl.cpp"
    INCLUDE_DIRS "include"
    REQUIRES L298N DRV8876 Encoder PID
)