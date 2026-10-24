cmake_minimum_required(VERSION 3.16)
project(DualSenseHapticCore CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /O2 /Oi)
else()
    add_compile_options(-Wall -Wextra -O3 -ffast-math)
endif()

find_package(Threads REQUIRED)

set(IMGUI_SOURCES
    imgui/imgui.cpp
    imgui/imgui_draw.cpp
    imgui/imgui_tables.cpp
    imgui/imgui_widgets.cpp
)

add_library(dualsense_haptic_core STATIC
    dualsense_manager.cpp
    dualsense_manager.hpp
    adaptive_triggers.hpp
    dualsense_telemetry.hpp
    haptic_engine.hpp
    ${IMGUI_SOURCES}
)

target_include_directories(dualsense_haptic_core PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/hidapi
)

add_executable(dualsense_dashboard main.cpp)
target_include_directories(dualsense_dashboard PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(dualsense_dashboard PRIVATE dualsense_haptic_core Threads::Threads)
