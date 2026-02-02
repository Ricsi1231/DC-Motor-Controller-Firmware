idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES motorControl DRV8876 Encoder PID)
