cmake_minimum_required(VERSION 3.2)

# Set up project.
project(driver_maestro
        VERSION 1.0
        DESCRIPTION "A driver for the Pololu Maestro servo controller.")

# Find package dependencies.
find_package(Boost REQUIRED COMPONENTS
    system)
find_package(Threads REQUIRED)

# Add sources.
file(GLOB_RECURSE maestro_source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Create library.
add_library(driver_maestro STATIC ${maestro_source})

# Link to dependencies.
target_link_libraries(driver_maestro
    Boost::system
    Threads::Threads)

# Set include directories for internal/external use.
target_include_directories(driver_maestro PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)