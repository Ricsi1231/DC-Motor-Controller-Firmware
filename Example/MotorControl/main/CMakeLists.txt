idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES MotorControl DRV8876 Encoder PID)
