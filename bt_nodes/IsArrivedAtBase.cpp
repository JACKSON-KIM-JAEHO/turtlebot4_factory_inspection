#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

namespace turtlebot4_factory_inspection
{

IsArrivedAtBase::IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
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
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");
  std::string file_path = package_share_dir + "/data/base_location.yaml";
  YAML::Node yaml = YAML::LoadFile(file_path);

  auto base = yaml["base_pose"];
  geometry_msgs::msg::Pose pose;
  pose.position.x = base["x"].as<double>();
  pose.position.y = base["y"].as<double>();
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;  // 회전은 무시
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

  RCLCPP_INFO(node_->get_logger(), "[IsArrivedAtBase] 📍 Base까지 거리: %.3f m", distance);

  if (distance < 0.3) {
    RCLCPP_INFO(node_->get_logger(), "[IsArrivedAtBase] ✅ Base에 도착");
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace turtlebot4_factory_inspection
