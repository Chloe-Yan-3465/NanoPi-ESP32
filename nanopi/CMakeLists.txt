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
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

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

# librealsense2 happens to re-export bundled ZSTD_* symbols, which lets the
# linker drop libzstd under --as-needed and leaves the depth codec silently
# bound to realsense's private copy. Pin the system libzstd with
# --no-as-needed so its DT_NEEDED entry is always emitted. The raw
# pkg-config path is used instead of PkgConfig::ZSTD because CMake hoists
# imported-target libraries past the flat linker flags, which would separate
# the flag from the library it must apply to.
target_link_libraries(mode2_capture PRIVATE
    ${REALSENSE_TARGET}
    yaml-cpp
    PkgConfig::ROCKCHIP_MPP
    -Wl,--no-as-needed
    ${ZSTD_LINK_LIBRARIES}
    -Wl,--as-needed
    Threads::Threads
    rt
)
target_include_directories(mode2_capture PRIVATE ${ZSTD_INCLUDE_DIRS})

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
