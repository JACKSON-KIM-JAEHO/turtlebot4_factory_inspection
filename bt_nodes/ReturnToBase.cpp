// src/bt_nodes/ReturnToBase.cpp

#include "turtlebot4_factory_inspection/bt_nodes/ReturnToBase.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <chrono>

namespace turtlebot4_factory_inspection::bt_nodes
{

ReturnToBase::ReturnToBase(const std::string& name, const BT::NodeConfiguration& config)
: BT::CoroActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("return_to_base_bt_node");
  client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&ReturnToBase::odom_callback, this, std::placeholders::_1));
}

void ReturnToBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  has_pose_ = true;
}

geometry_msgs::msg::PoseStamped ReturnToBase::load_base_pose()
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");
  std::string yaml_file = pkg_share + "/data/base_location.yaml";

  YAML::Node yaml = YAML::LoadFile(yaml_file);
  auto base = yaml["base_pose"];

  double x = base["x"].as<double>();
  double y = base["y"].as<double>();
  double yaw = base["yaw"].as<double>();  // radian

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = tf2::toMsg(q);

  base_pose_ = pose.pose;
  return pose;
}

BT::NodeStatus ReturnToBase::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[ReturnToBase] 🏁 복귀 시작");

  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[ReturnToBase] ❌ Nav2 action 서버 연결 실패");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = load_base_pose();

  if (has_pose_) {
    double dx = base_pose_.position.x - current_pose_.position.x;
    double dy = base_pose_.position.y - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance < 0.15) {
      RCLCPP_INFO(node_->get_logger(), "[ReturnToBase] 🤖 이미 base에 도착함 (%.2f m)", distance);
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "[ReturnToBase] 현재 위치 정보 없음 → 복귀 강제 실행");
  }

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  auto goal_handle_future = client_ptr_->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[ReturnToBase] ❌ Goal 전송 실패");
    return BT::NodeStatus::FAILURE;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[ReturnToBase] ❌ Goal handle 생성 실패");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = client_ptr_->async_get_result(goal_handle);
  RCLCPP_INFO(node_->get_logger(), "[ReturnToBase] ⏳ base로 복귀 중...");

  while (rclcpp::ok()) {
    if (result_future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready)
      break;
    setStatusRunningAndYield();
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[ReturnToBase] ✅ 복귀 성공");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "[ReturnToBase] ❌ 복귀 실패");
    return BT::NodeStatus::FAILURE;
  }
}

void ReturnToBase::halt()
{
  RCLCPP_WARN(node_->get_logger(), "[ReturnToBase] ⛔ HALT 호출됨");
}

}  // namespace turtlebot4_factory_inspection::bt_nodes
