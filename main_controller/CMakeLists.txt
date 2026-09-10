cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME main_controller)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    # Motor driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Motors/DRV8833.c

    # Encoder odometry
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Encoders/encoders.c

    # Low level control
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/PID.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/EKF.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/straightline_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/turn_controller.c

    # Sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/ICM-42688-P/ICM42688.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/TCA9548A/TCA9548A.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X_Driver/tof_sensors.c

    # VL53L0X - stock ST API (STSW-IMG005 v1.0.4), do not edit.
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Core/vl53l0x_api.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Core/vl53l0x_api_calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Core/vl53l0x_api_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Core/vl53l0x_api_ranging.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Core/vl53l0x_api_strings.c

    # VL53L0X platform layer - rewritten for STM32 HAL I2C.
    # NOTE: the other three files ST ships in Platform/ (vl53l0x_i2c_platform.c,
    # vl53l0x_i2c_win_serial_comms.c, vl53l0x_platform_log.c) are Win32-only
    # reference code and are deliberately NOT built. Do not add them.
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Sensors/VL53L0X/Platform/vl53l0x_platform.c

    # Utilities
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Utils/dwt_timer.c

    # On-target test harness
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Tests/test_harness.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Motors
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Encoders
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Control/LowLevel
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Sensors/ICM-42688-P
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Sensors/TCA9548A
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Sensors/VL53L0X_Driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Sensors/VL53L0X/Core
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Sensors/VL53L0X/Platform
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Utils
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Tests
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    m   # math library, for fabsf() in the controllers
)
