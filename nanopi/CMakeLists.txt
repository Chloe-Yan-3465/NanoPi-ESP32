cmake_minimum_required(VERSION 3.16)
project(nanopi_mode2_capture_20260808 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(realsense2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ROCKCHIP_MPP REQUIRED IMPORTED_TARGET rockchip_mpp)

add_executable(mode2_capture camera_cap/mode2_capture.cpp)
target_compile_options(mode2_capture PRIVATE
    -O2 -Wall -Wextra -Wpedantic
)
target_include_directories(mode2_capture PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/camera_cap
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)

if(TARGET realsense2::realsense2)
    set(REALSENSE_TARGET realsense2::realsense2)
elseif(TARGET realsense2)
    set(REALSENSE_TARGET realsense2)
else()
    set(REALSENSE_TARGET ${realsense2_LIBRARY})
    target_include_directories(mode2_capture PRIVATE ${realsense2_INCLUDE_DIR})
endif()

target_link_libraries(mode2_capture PRIVATE
    ${REALSENSE_TARGET}
    yaml-cpp
    PkgConfig::ROCKCHIP_MPP
    Threads::Threads
    rt
)

add_executable(uart_camera_receiver camera_cap/uart_camera_receiver.cpp)
target_compile_options(uart_camera_receiver PRIVATE
    -O2 -Wall -Wextra -Wpedantic
)
target_include_directories(uart_camera_receiver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/common
)
target_link_libraries(uart_camera_receiver PRIVATE
    Threads::Threads
    rt
)
