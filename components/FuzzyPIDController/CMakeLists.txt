idf_component_register(
    SRCS "src/FuzzyPIDController.cpp"
    INCLUDE_DIRS "include"
    REQUIRES PID efll
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
