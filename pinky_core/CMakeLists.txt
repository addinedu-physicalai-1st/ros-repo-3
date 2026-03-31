cmake_minimum_required(VERSION 3.20)
project(pinky_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(BUILD_HAL   "Build hardware abstraction layer (requires ARM64)" ON)
option(BUILD_TESTS "Build unit tests" ON)

if(NOT CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(BUILD_HAL OFF)
  message(STATUS "Non-ARM64 platform: HAL disabled, building core + tests only")
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(Threads REQUIRED)

# spdlog
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
  message(STATUS "spdlog not found via find_package, trying pkg-config")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SPDLOG REQUIRED spdlog)
endif()

# OpenCV (optional, needed for camera frame encoding)
find_package(OpenCV QUIET COMPONENTS core imgcodecs)

# ONNX Runtime (optional, for inference module)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /opt/onnxruntime/include
        /usr/local/include/onnxruntime
        $ENV{ONNXRUNTIME_ROOT}/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS /opt/onnxruntime/lib
        /usr/local/lib
        $ENV{ONNXRUNTIME_ROOT}/lib)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  set(ONNXRUNTIME_FOUND TRUE)
  message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIBRARY}")
else()
  set(ONNXRUNTIME_FOUND FALSE)
  message(STATUS "ONNX Runtime not found, inference module will be skipped")
endif()

# ---------------------------------------------------------------------------
# Core library (platform-independent)
# ---------------------------------------------------------------------------
set(CORE_SOURCES
  src/common/logger.cpp
  src/protocol/checksum.cpp
  src/protocol/serializer.cpp
  src/net/tcp_server.cpp
  src/net/udp_server.cpp
  src/net/connection_manager.cpp
  src/net/frame_sender.cpp
  src/core/diff_drive.cpp
  src/core/odometry.cpp
  src/core/lidar_processor.cpp
  src/core/battery_monitor.cpp
  src/core/led_controller.cpp
  src/inference/onnx_actor.cpp
  src/inference/observation_builder.cpp
  src/inference/rl_controller.cpp
)

add_library(pinky_core STATIC ${CORE_SOURCES})

target_include_directories(pinky_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pinky_core PUBLIC
  Threads::Threads
)

if(spdlog_FOUND)
  target_link_libraries(pinky_core PUBLIC spdlog::spdlog)
else()
  target_include_directories(pinky_core PUBLIC ${SPDLOG_INCLUDE_DIRS})
  target_link_libraries(pinky_core PUBLIC ${SPDLOG_LIBRARIES})
endif()

if(OpenCV_FOUND)
  target_link_libraries(pinky_core PUBLIC ${OpenCV_LIBS})
  target_compile_definitions(pinky_core PUBLIC PINKY_HAS_OPENCV)
endif()

if(ONNXRUNTIME_FOUND)
  target_include_directories(pinky_core PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
  target_link_libraries(pinky_core PUBLIC ${ONNXRUNTIME_LIBRARY})
  target_compile_definitions(pinky_core PUBLIC PINKY_HAS_ONNXRUNTIME)
endif()

# ---------------------------------------------------------------------------
# HAL library (ARM64 only, Phase 5)
# ---------------------------------------------------------------------------
if(BUILD_HAL)
  message(STATUS "Building HAL for ARM64")
  add_library(pinky_hal STATIC 
    src/hal/dynamixel_motor.cpp
    src/hal/sllidar_driver.cpp
    src/hal/bno055_imu.cpp
    src/hal/adc_sensor.cpp
    src/hal/ws2811_led.cpp
    src/hal/ili9341_lcd.cpp
  )
  
  # Search for RPLIDAR SDK locally in pinky_devices
  set(RPLIDAR_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pinky_devices/rplidar_sdk")
  if(EXISTS "${RPLIDAR_SDK_DIR}")
    target_include_directories(pinky_hal PUBLIC 
      ${RPLIDAR_SDK_DIR}/sdk/include
      ${RPLIDAR_SDK_DIR}/sdk/src
    )
    # The static library name and release path for aarch64 gcc
    file(GLOB RPLIDAR_LIB_PATH "${RPLIDAR_SDK_DIR}/sdk/output/Linux/Release/libsl_lidar_sdk.a")
    target_link_libraries(pinky_hal PUBLIC pinky_core dynamixel_sdk wiringPi ws2811 ${RPLIDAR_LIB_PATH} pthread)
  else()
    message(WARNING "RPLIDAR SDK not found. Lidar node will fail to compile. Please install in pinky_devices!")
    target_link_libraries(pinky_hal PUBLIC pinky_core dynamixel_sdk wiringPi ws2811)
  endif()

  target_compile_definitions(pinky_hal PUBLIC BUILD_HAL)
endif()

# ---------------------------------------------------------------------------
# Main executable (Phase 6)
# ---------------------------------------------------------------------------
add_executable(pinky_robot src/app/main.cpp src/app/robot_app.cpp)
target_link_libraries(pinky_robot PRIVATE pinky_core)

if(BUILD_HAL)
  target_link_libraries(pinky_robot PRIVATE pinky_hal)
  target_compile_definitions(pinky_robot PUBLIC BUILD_HAL)
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTS)
  enable_testing()

  # Protocol roundtrip test
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_protocol.cpp)
    add_executable(test_protocol tests/test_protocol.cpp)
    target_link_libraries(test_protocol PRIVATE pinky_core)
    add_test(NAME protocol_roundtrip COMMAND test_protocol)
  endif()

  # Core logic tests
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_core.cpp)
    add_executable(test_core tests/test_core.cpp)
    target_link_libraries(test_core PRIVATE pinky_core)
    add_test(NAME core_logic COMMAND test_core)
  endif()
endif()
