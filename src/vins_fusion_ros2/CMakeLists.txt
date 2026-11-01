cmake_minimum_required(VERSION 3.16...3.22)
project(vins_fusion_ros2)

include(cmake/foxy_compat.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wextra -Wpedantic)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_subdirectory(vins)
add_executable(vins_fusion_ros2_node src/vins_estimator.cpp src/vins_estimator_node.cpp src/visualization.cpp)
ament_target_dependencies(vins_fusion_ros2_node rclcpp rcpputils message_filters std_msgs builtin_interfaces sensor_msgs visualization_msgs geometry_msgs nav_msgs tf2 tf2_ros cv_bridge image_transport)
target_link_libraries(vins_fusion_ros2_node vins_lib)

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(
  TARGETS vins_fusion_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_visualization test/test_visualization.cpp src/visualization.cpp)
  ament_target_dependencies(test_visualization rclcpp builtin_interfaces sensor_msgs
    std_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros cv_bridge)
  target_link_libraries(test_visualization vins_lib)
  add_test(NAME sensor_smoke
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/smoke_test.py
      $<TARGET_FILE:vins_fusion_ros2_node> ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(sensor_smoke PROPERTIES TIMEOUT 90)
endif()
ament_package()
