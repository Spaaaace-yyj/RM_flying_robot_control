cmake_minimum_required(VERSION 3.8)
project(drone_visualization LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(odometry_drone_marker src/odometry_drone_marker.cpp)
target_compile_features(odometry_drone_marker PRIVATE cxx_std_17)
target_compile_options(odometry_drone_marker PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(odometry_drone_marker
  rclcpp rcl_interfaces builtin_interfaces nav_msgs geometry_msgs std_msgs visualization_msgs)

install(TARGETS odometry_drone_marker DESTINATION lib/drone_visualization)
install(DIRECTORY launch config DESTINATION share/drone_visualization)
ament_package()
