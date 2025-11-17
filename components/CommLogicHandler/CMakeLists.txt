idf_component_register(SRCS "src/CommLogicHandler.cpp"
                       INCLUDE_DIRS "include"
                       REQUIRES DRV8876 Encoder PID MotorCommHandler MotorControl)
