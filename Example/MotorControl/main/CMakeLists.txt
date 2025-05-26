idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES MotorControl L298N DRV8876 Encoder PID)
