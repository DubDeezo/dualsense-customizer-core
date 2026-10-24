cmake_minimum_required(VERSION 3.16)
project(DualSenseHapticCore CXX)

# Force modern language features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimize for low-latency processing and strict hardware compliance
if(MSVC)
    add_compile_options(/W4 /WX /O2 /Oi)
else()
    add_compile_options(-Wall -Wextra -Werror -O3 -ffast-math)
endif()

find_package(Threads REQUIRED)

# Interface/Library targets
add_library(dualsense_haptic_core 
    dualsense_manager.cpp
    dualsense_manager.hpp
    adaptive_triggers.hpp
    dualsense_telemetry.hpp
    haptic_engine.hpp
)

# Set up the executable using main.cpp
add_executable(dualsense_test main.cpp)

# Detect and automatically link hidapi based on current environment
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(HIDAPI QUIET hidapi-hidraw)
    if(NOT HIDAPI_FOUND)
        pkg_check_modules(HIDAPI QUIET hidapi-libusb)
    endif()
endif()

if(HIDAPI_FOUND)
    target_link_libraries(dualsense_haptic_core PRIVATE ${HIDAPI_LIBRARIES})
    target_include_directories(dualsense_haptic_core PRIVATE ${HIDAPI_INCLUDE_DIRS})
else()
    # Fallback for standard system paths / package managers
    target_link_libraries(dualsense_haptic_core PRIVATE hidapi)
endif()

# Link the core library and threading to your executable app
target_link_libraries(dualsense_test PRIVATE dualsense_haptic_core Threads::Threads)
