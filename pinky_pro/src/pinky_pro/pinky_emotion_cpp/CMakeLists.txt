cmake_minimum_required(VERSION 3.8)
project(pinky_emotion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinky_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  add_executable(emotion_server_node src/main_node.cpp)
  target_include_directories(emotion_server_node PRIVATE ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(emotion_server_node
    rclcpp
    pinky_interfaces
    OpenCV
    cv_bridge
    ament_index_cpp
  )
  target_link_libraries(emotion_server_node
    wiringPi
  )

  install(TARGETS
    emotion_server_node
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
