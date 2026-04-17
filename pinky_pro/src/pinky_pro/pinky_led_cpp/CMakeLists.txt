cmake_minimum_required(VERSION 3.8)
project(pinky_led_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinky_interfaces REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(WS281X_DIR "/home/hajun/ros2_ws/pinky/pinky_devices/rpi_ws281x")

  add_executable(led_service_node src/main_node.cpp)
  target_include_directories(led_service_node PRIVATE ${WS281X_DIR})
  target_link_libraries(led_service_node
    ${WS281X_DIR}/build/libws2811.a
  )
  ament_target_dependencies(led_service_node
    rclcpp
    pinky_interfaces
  )

  install(TARGETS
    led_service_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "${PROJECT_NAME} build only for ARM64. Skip building.")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
