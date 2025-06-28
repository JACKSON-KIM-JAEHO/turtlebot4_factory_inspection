#include "turtlebot4_factory_inspection/MoveToTarget.hpp" //헤더 파일 (클래스 정의)
#include <chrono> //시간 관련 유틸리티 (초, 밀리초 단위)
#include <memory> // 스마트 포인터 사용을 위한 헤더
#include <thread> // std::thread 등 멀티 스레딩

using namespace std::chrono_literals; // 5s, 100ms 등처럼 시간 단위를 문자처럼 자연스럽게 쓰기 위한 선언

MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_target_node");
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose"); //액션 서버 클라이언트 생성

  // 별도 ROS2 스레드로 spin()
  std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  }).detach();
}

BT::NodeStatus MoveToTarget::tick()
{
  if (!client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = 0.0;  // 목표 위치
  goal.pose.pose.position.y = 1.5;
  goal.pose.pose.orientation.w = 1.0;  // 회전 없음 -> 이것만 보면 본인 기준 무조건 몇도 돌아라 인 것 같다. 몇도로 맞춰라가 아니라

  auto future_goal = client_->async_send_goal(goal);
  if (future_goal.wait_for(3s) != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to send goal.");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = future_goal.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected.");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = client_->async_get_result(goal_handle);
  while (result_future.wait_for(100ms) != std::future_status::ready) {
    std::this_thread::sleep_for(100ms);
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "Navigation succeeded.");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Navigation failed.");
    return BT::NodeStatus::FAILURE;
  }
}