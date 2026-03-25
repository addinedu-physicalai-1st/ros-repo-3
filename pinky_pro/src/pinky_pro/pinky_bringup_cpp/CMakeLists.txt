cmake_minimum_required(VERSION 3.8)
project(pinky_bringup_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(realtime_tools REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  add_executable(pinky_bringup_node src/main_node.cpp)
  target_include_directories(pinky_bringup_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(pinky_bringup_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    dynamixel_sdk
    realtime_tools
  )

  install(TARGETS
    pinky_bringup_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "${PROJECT_NAME} build only for ARM64. Skip building.")
endif()

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
