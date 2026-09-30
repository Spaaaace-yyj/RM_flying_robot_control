cmake_minimum_required(VERSION 3.8)
project(driver_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(driver_interface_node
  src/driver_interface_node.cpp)

target_include_directories(driver_interface_node PRIVATE
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(driver_interface_node
  rclcpp
  sensor_msgs
  cv_bridge
  px4_msgs)

target_link_libraries(driver_interface_node
  ${OpenCV_LIBRARIES})

install(TARGETS driver_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME})

ament_package()
