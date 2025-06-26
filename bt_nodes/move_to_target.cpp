#include "turtlebot4_factory_inspection/move_to_target.hpp"
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_target_node");
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  // 별도 ROS2 스레드로 spin()
  std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  }).detach();
}

BT::NodeStatus MoveToTarget::tick()
{
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = node_->get_clock()->now();
  goal.pose.pose.position.x = 0.0;  // ✅ 목표 위치
  goal.pose.pose.position.y = 1.5;
  goal.pose.pose.orientation.w = 1.0;  // 회전 없음

  auto future_goal = client_->async_send_goal(goal);
  if (future_goal.wait_for(5s) != std::future_status::ready) {
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