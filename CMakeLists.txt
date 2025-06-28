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

# ROS 2 핵심 라이브러리
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Nav2 액션 메시지
find_package(nav2_msgs REQUIRED)

# Behavior Tree CPP
find_package(behaviortree_cpp_v3 REQUIRED)

# 실행 중 XML 파일 설치 위치를 가져오기 위한 패키지 (BT 실행 시 추천)
find_package(ament_index_cpp REQUIRED)

# ---------- 실행 파일 빌드 설정 ----------
include("${CMAKE_CURRENT_SOURCE_DIR}/sources.cmake")

add_executable(bt_runner
  ${SOURCE_FILES}
)

# ---------- 헤더 파일 포함 경로 설정 ----------
target_include_directories(bt_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------- 의존성 연결 ----------
ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  nav2_msgs
  behaviortree_cpp_v3
  ament_index_cpp
  tf2_ros
  tf2_geometry_msgs
)

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

# ---------- 빌드 시스템에 패키지로 등록 ----------
ament_package()
