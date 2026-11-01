cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(feature_tracker
  src/feature_tracker_node.cpp
  src/parameters.cpp
  src/feature_tracker.cpp)

target_include_directories(feature_tracker PRIVATE
  ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(feature_tracker
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  camera_model)

target_link_libraries(feature_tracker
  ${OpenCV_LIBS})

install(TARGETS feature_tracker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
