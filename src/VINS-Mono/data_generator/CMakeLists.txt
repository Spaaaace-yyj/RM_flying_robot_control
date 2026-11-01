cmake_minimum_required(VERSION 3.8)
project(data_generator)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(data_generator
  src/data_generator_node.cpp
  src/data_generator.cpp)

target_include_directories(data_generator PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(data_generator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge)

target_link_libraries(data_generator ${OpenCV_LIBS})

install(TARGETS data_generator DESTINATION lib/${PROJECT_NAME})

ament_package()
