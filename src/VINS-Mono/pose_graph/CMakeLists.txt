cmake_minimum_required(VERSION 3.8)
project(pose_graph)

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
find_package(ament_index_cpp REQUIRED)
find_package(camera_model REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pose_graph
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp)

target_include_directories(pose_graph PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS})

ament_target_dependencies(pose_graph
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  ament_index_cpp
  camera_model)

target_link_libraries(pose_graph
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})

install(TARGETS pose_graph
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../support_files/
  DESTINATION share/${PROJECT_NAME}/support_files)

ament_package()
