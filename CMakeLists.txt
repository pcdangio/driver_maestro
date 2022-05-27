cmake_minimum_required(VERSION 3.2)

# Set up project.
project(driver_maestro
        VERSION 1.0
        DESCRIPTION "A driver for the Pololu Maestro servo controller.")

# Create library.
add_library(driver_maestro STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/driver.cpp)

# Set include directories for internal/external use.
target_include_directories(driver_maestro PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS})