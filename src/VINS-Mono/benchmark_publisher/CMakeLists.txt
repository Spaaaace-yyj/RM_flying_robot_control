cmake_minimum_required(VERSION 3.8)
project(benchmark_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(benchmark_publisher src/benchmark_publisher_node.cpp)

target_include_directories(benchmark_publisher PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(benchmark_publisher rclcpp nav_msgs geometry_msgs)

install(TARGETS benchmark_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
