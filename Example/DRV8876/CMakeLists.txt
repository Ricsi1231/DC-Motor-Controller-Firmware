cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "../../components")
set(EXCLUDE_COMPONENTS "USB" "MotorCommHandler")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drv8876_example)