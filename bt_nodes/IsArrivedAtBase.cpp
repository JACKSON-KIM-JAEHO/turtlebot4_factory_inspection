// src/bt_nodes/IsArrivedAtBase.cpp

#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cmath>

namespace turtlebot4_factory_inspection::bt_nodes
{

IsArrivedAtBase::IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("is_arrived_at_base_bt_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList IsArrivedAtBase::providedPorts()
{
  return {};
}

geometry_msgs::msg::Pose IsArrivedAtBase::load_base_pose()
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");
  std::string file_path = pkg_share + "/data/base_location.yaml";
  YAML::Node yaml = YAML::LoadFile(file_path);

  auto base = yaml["base_pose"];
  double x = base["x"].as<double>();
  double y = base["y"].as<double>();
  double yaw = base["yaw"].as<double>();

  // yaw → quaternion 변환
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);  // roll, pitch, yaw

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = tf2::toMsg(q);

  return pose;
}

BT::NodeStatus IsArrivedAtBase::tick()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[IsArrivedAtBase] TF Error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose base_pose = load_base_pose();

  double dx = base_pose.position.x - transform.transform.translation.x;
  double dy = base_pose.position.y - transform.transform.translation.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 현재 방향 계산
  tf2::Quaternion q_curr(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(q_curr).getRPY(roll, pitch, current_yaw);

  // 목표 방향 계산
  tf2::Quaternion q_target;
  tf2::fromMsg(base_pose.orientation, q_target);
  double _, __, target_yaw;
  tf2::Matrix3x3(q_target).getRPY(_, __, target_yaw);

  double yaw_diff = std::fabs(current_yaw - target_yaw);
  if (yaw_diff > M_PI) {
    yaw_diff = 2 * M_PI - yaw_diff;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[IsArrivedAtBase] 📍 거리: %.3f m, 각도차: %.3f rad",
              distance, yaw_diff);

  if (distance < 0.3 && yaw_diff < 0.2) {
    RCLCPP_INFO(node_->get_logger(), "[IsArrivedAtBase] ✅ 위치 및 방향 도착 확인");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace turtlebot4_factory_inspection::bt_nodes
