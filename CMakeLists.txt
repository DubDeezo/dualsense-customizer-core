cmake_minimum_required(VERSION 3.16)
project(DualSenseHapticCore CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /WX /O2 /Oi)
else()
    add_compile_options(-Wall -Wextra -Werror -O3 -ffast-math)
endif()

find_package(Threads REQUIRED)

# Incorporate Dear ImGui Source Files from Local External Repositories Automatically
include(FetchContent)
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com
    GIT_TAG        v1.90.4
)
FetchContent_MakeAvailable(imgui)

# Create an array of source files to manage the implementation structure mapping boundaries
set(IMGUI_SOURCES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
)

# Modern compiled Core Library targets
add_library(dualsense_haptic_core 
    dualsense_manager.cpp
    dualsense_manager.hpp
    adaptive_triggers.hpp
    dualsense_telemetry.hpp
    haptic_engine.hpp
    ${IMGUI_SOURCES}
)
target_include_directories(dualsense_haptic_core PUBLIC ${imgui_SOURCE_DIR})

# Set up testing executable app
add_executable(dualsense_dashboard main.cpp)

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
    target_link_libraries(dualsense_haptic_core PRIVATE hidapi)
endif()

target_link_libraries(dualsense_dashboard PRIVATE dualsense_haptic_core Threads::Threads)
