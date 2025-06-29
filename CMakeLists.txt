# 이 프로젝트에서 사용할 최소 CMake 버전
cmake_minimum_required(VERSION 3.8)

# 프로젝트 이름
project(turtlebot4_factory_inspection)

# C++ 표준 설정: C++17을 사용하고, 필수로 적용하도록 지정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 경고 옵션 추가 (GCC 또는 Clang을 사용할 경우만 적용)
# -Wall: 모든 일반 경고
# -Wextra: 추가적인 경고
# -Wpedantic: C++ 표준을 엄격히 준수
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- 의존성 찾기 (필수 패키지) ----------
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

# ---------- 실행 파일 빌드 설정 ----------
include("${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake")

add_executable(bt_runner
  ${SOURCE_FILES}
)

# ---------- 헤더 파일 포함 경로 설정 ----------
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

# ---------- 의존성 연결 ----------
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

# yaml-cpp은 일반 CMake 라이브러리이므로 직접 링크
target_link_libraries(bt_runner yaml-cpp)

# ---------- 설치 설정 ----------
# 실행 파일
install(TARGETS bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더
install(DIRECTORY include/
  DESTINATION include/
)

# Behavior Tree XML 설치
install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# ---------- 빌드 시스템에 패키지로 등록 ----------
ament_package()
