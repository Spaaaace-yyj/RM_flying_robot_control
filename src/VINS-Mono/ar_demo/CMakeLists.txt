cmake_minimum_required(VERSION 3.8)
project(ar_demo)

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
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(ar_demo_node src/ar_demo_node.cpp)

target_include_directories(ar_demo_node PRIVATE ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(ar_demo_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  camera_model)

target_link_libraries(ar_demo_node ${OpenCV_LIBS})

install(TARGETS ar_demo_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
