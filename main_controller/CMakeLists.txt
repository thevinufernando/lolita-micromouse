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
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/straightline_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Control/LowLevel/turn_controller.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Motors
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Encoders
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/Control/LowLevel
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
