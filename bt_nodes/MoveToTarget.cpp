// 핵심 이동 노드: 주어진 목표 위치로 이동 (Nav2 사용)

#include "turtlebot4_factory_inspection/bt_nodes/MoveToTarget.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"

// 좌표계 변환 및 거리 계산 관련
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // AMCL 초기 보정 용도
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <angles/angles.h>  // shortest_angular_distance 계산

// Action 클라이언트 및 비동기 처리를 위한 라이브러리
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <thread>
#include <chrono>
#include <vector>
#include <future>
#include <cmath>

using namespace turtlebot4_factory_inspection;
using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

// 생성자: 노드 초기화 및 Action 클라이언트, TF 리스너 설정
MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::CoroActionNode(name, config)  // Coroutine 기반 실행 구조
{
  node_ = rclcpp::Node::make_shared("move_to_target_bt_node");

  // Nav2 목적지 이동 action 클라이언트 생성
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  // TF 리스너 설정
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 별도 스레드로 ROS 노드 spin 실행 (executor)
  spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
}

// 소멸자: spin 스레드 종료 처리
MoveToTarget::~MoveToTarget()
{
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

// 포트 정의: BT 트리에서 입출력할 값 명시
BT::PortsList MoveToTarget::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),  // 전체 작업 리스트
    BT::InputPort<int>("current_index"),  // 현재 타겟 인덱스
    BT::InputPort<std::vector<int>>("faillist"),  // 실패한 인덱스 리스트
    BT::OutputPort<int>("current_index"),  // 인덱스 그대로 출력
    BT::OutputPort<bool>("is_arrived"),  // 도착 여부
    BT::OutputPort<std::vector<int>>("faillist"),  // 실패 시 업데이트된 리스트
    BT::OutputPort<double>("angle")  // 도착 실패 시 다음 바라볼 방향
  };
}

// BT 실행 시 동작 정의
BT::NodeStatus MoveToTarget::tick()
{
  // Blackboard에서 값 가져오기
  std::vector<geometry_msgs::msg::PoseStamped> task_list;
  int current_index;
  std::vector<int> fail_list;

  if (!getInput("task_list", task_list)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Missing input [task_list]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("current_index", current_index)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Missing input [current_index]");
    return BT::NodeStatus::SUCCESS;  // (오류지만 SUCCESS 반환으로 트리 흐름 유지)
  }
  getInput("faillist", fail_list);

  // 유효한 인덱스인지 확인
  if (current_index < 0 || current_index >= static_cast<int>(task_list.size())) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Invalid current_index: %d", current_index);
    return BT::NodeStatus::FAILURE;
  }

  // 현재 목표 위치 가져오기
  auto goal = task_list[current_index];
  goal.header.frame_id = "map";
  goal.header.stamp = node_->get_clock()->now();

  // 목표까지 거리 계산하여 timeout 기준 설정
  double max_duration_sec = 10.0;  // 기본값
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    double dx = goal.pose.position.x - tf.transform.translation.x;
    double dy = goal.pose.position.y - tf.transform.translation.y;
    double distance = std::hypot(dx, dy);
    max_duration_sec = distance * 3.0;
    RCLCPP_INFO(node_->get_logger(), "[MoveToTarget] Distance = %.2f m → Timeout = %.1f s", distance, max_duration_sec);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] TF error during distance calc: %s", ex.what());
    max_duration_sec = 30.0;  // TF 실패 시 예비 타임아웃
  }

  // 이동 재시도 최대 3번까지
  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Action server unavailable");
      continue;
    }

    // 목적지 설정 및 goal 전송
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal = client_->async_send_goal(goal_msg);
    if (send_goal.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Failed to send goal (attempt %d)", attempt);
      continue;
    }

    auto handle = send_goal.get();
    if (!handle) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Goal rejected");
      continue;
    }
    goal_handle_ = handle;

    // 결과 수신 대기
    auto result_future = client_->async_get_result(handle);
    auto start = node_->now();

    while (rclcpp::ok()) {
      if (result_future.wait_for(100ms) == std::future_status::ready) break;

      // 타임아웃 시 goal 취소
      if ((node_->now() - start).seconds() > max_duration_sec) {
        client_->async_cancel_goal(handle);
        RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Timeout cancel (attempt %d)", attempt);
        break;
      }

      // 트리에 Running 상태 알림
      setStatusRunningAndYield();
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      setOutput("is_arrived", true);  // 성공 상태 저장
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Attempt %d failed", attempt);
    }
  }

  // 3회 실패 시: 실패 리스트에 추가 + 방향 저장
  setOutput("is_arrived", false);
  fail_list.push_back(current_index);
  setOutput("faillist", fail_list);

  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    double yaw = std::atan2(
      goal.pose.position.y - tf.transform.translation.y,
      goal.pose.position.x - tf.transform.translation.x
    );
    setOutput("angle", yaw);  // 다음 노드에서 회전용
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] TF error computing angle: %s", ex.what());
  }

  setOutput("current_index", current_index);
  return BT::NodeStatus::SUCCESS;  // 실패지만 트리 흐름을 위해 SUCCESS 반환
}

// 트리 강제 중단 시: goal 취소
void MoveToTarget::halt()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Halt: goal canceled");
  }
  CoroActionNode::halt();  // 부모 클래스 halt 처리
}
