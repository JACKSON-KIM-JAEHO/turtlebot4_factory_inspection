cmake_minimum_required(VERSION 3.8)
project(turtlebot4_factory_inspection)

# C++17, warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ament and ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(angles REQUIRED)
find_package(yaml-cpp REQUIRED)

# Gather all source files
include("${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake")

# Executable
add_executable(bt_runner
  ${SOURCE_FILES}
)

# Include paths for this project and external headers
target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${angles_INCLUDE_DIRS}
)

# Link libraries
ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  angles
  yaml-cpp
)

# Some platforms require linking stdc++fs for filesystem
if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
  target_link_libraries(bt_runner stdc++fs)
endif()

# On systems where yaml-cpp isn't linked automatically
target_link_libraries(bt_runner yaml-cpp)

# Install rules
install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# export dependencies and finish
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  cv_bridge
  OpenCV
  angles
  yaml-cpp
)
ament_package()
