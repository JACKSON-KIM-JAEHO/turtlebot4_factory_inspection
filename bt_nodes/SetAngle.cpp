#include "turtlebot4_factory_inspection/bt_nodes/SetAngle.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>                       // [ADDED] TF buffer header for fallback yaw handling
#include <tf2_ros/transform_listener.h>           // [ADDED] TF listener header
#include <tf2/utils.h>
#include <cmath>
#include <vector>                                  // [ADDED] for task_yaw_list

using namespace std::chrono_literals;

// SetAngle Action Node: 지정된 yaw 방향(절대 또는 fallback)으로 회전
SetAngle::SetAngle(const std::string& name, const BT::NodeConfiguration& config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("set_angle_bt_node");
  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock()); // [ADDED]
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // [ADDED]
}

BT::PortsList SetAngle::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("task_yaw_list"),   // 원래 포트: 작업 리스트 yaw
    BT::InputPort<int>("current_index"),                  // 원래 포트: 현재 인덱스
    BT::InputPort<double>("angle")                        // [ADDED] fallback yaw from MoveToTarget
  };
}

// 회전 시작 시 로그
BT::NodeStatus SetAngle::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[SetAngle] ▶️ Starting rotation...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetAngle::onRunning()
{
  // 1) task_yaw_list, current_index 읽기
  std::vector<double> yaw_list;
  int index;
  if (!getInput("task_yaw_list", yaw_list)) {
    RCLCPP_ERROR(node_->get_logger(), "[SetAngle] ❌ task_yaw_list missing");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("current_index", index)) {
    RCLCPP_ERROR(node_->get_logger(), "[SetAngle] ❌ current_index missing");
    return BT::NodeStatus::FAILURE;
  }
  // 디버깅: 받은 값 확인
  RCLCPP_INFO(node_->get_logger(), "[SetAngle] 🔎 Received current_index = %d, yaw = %.1f", index, yaw_list[index]);

  // 2) fallback angle 읽기
  auto angle_res = getInput<double>("angle");
  bool has_fallback = angle_res.has_value();
  double fallback_yaw = has_fallback ? angle_res.value() : 0.0;
  if (has_fallback) {
    RCLCPP_INFO(node_->get_logger(), "[SetAngle] Using fallback yaw: %.2f rad", fallback_yaw); // [ADDED]
  }

  if (index < 0 || index >= static_cast<int>(yaw_list.size())) {
    RCLCPP_ERROR(node_->get_logger(), "[SetAngle] ❌ Invalid index: %d", index);
    return BT::NodeStatus::FAILURE;
  }

  // 3) 목표 yaw 결정
  double target_yaw_rad;
  if (has_fallback) {
    target_yaw_rad = fallback_yaw;  // [ADDED]
  } else {
    double target_yaw_deg = yaw_list[index];
    RCLCPP_INFO(node_->get_logger(), "[SetAngle] Using task_yaw_list[%d]: %.2f deg", index, target_yaw_deg);
    target_yaw_rad = target_yaw_deg * M_PI / 180.0;
  }

  // 4) 현재 yaw 조회 via TF
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[SetAngle] ⚠️ TF lookup failed: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  double current_yaw = tf2::getYaw(q);

  // 5) yaw 오차 계산 및 정규화
  double yaw_error = angles::shortest_angular_distance(current_yaw, target_yaw_rad);

  // 6) 목표 도달 여부 확인 (~5도)
  if (std::abs(yaw_error) < 0.087) {
    RCLCPP_INFO(node_->get_logger(), "[SetAngle] ✅ Target yaw reached (%.1f deg)", has_fallback ? (fallback_yaw*180.0/M_PI) : (yaw_list[index]));
    stop();
    return BT::NodeStatus::SUCCESS;
  }

  // 7) 회전 명령 발행
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = (yaw_error > 0) ? 0.4 : -0.4;
  pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

void SetAngle::onHalted()
{
  stop();
  RCLCPP_WARN(node_->get_logger(), "[SetAngle] ⏹ Halted and stopped rotation.");
}

// 모터 정지 함수
void SetAngle::stop()
{
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = 0.0;
  pub_->publish(cmd);
}
