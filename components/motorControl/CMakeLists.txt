idf_component_register(
    SRCS "src/motorControl.cpp"
         "src/MotionProfiler.cpp"
         "src/SettleDetector.cpp"
         "src/StallDetector.cpp"
         "src/SoftLimiter.cpp"
         "src/MotionGuard.cpp"
    INCLUDE_DIRS "include"
    REQUIRES Interfaces DRV8876 Encoder PID
    PRIV_REQUIRES esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
