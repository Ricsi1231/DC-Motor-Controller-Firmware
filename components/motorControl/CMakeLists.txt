idf_component_register(
    SRCS
        "src/MotorControl.cpp"
        "src/ControlLaw.cpp"
        "src/MotionMonitor.cpp"
        "src/Actuator.cpp"
        "src/ControlTask.cpp"
        "src/MotionProfile.cpp"
        "src/MotionDetector.cpp"
        "src/SoftLimits.cpp"
    INCLUDE_DIRS "include"
    REQUIRES PID DRV8876 Encoder FuzzyPIDController
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
