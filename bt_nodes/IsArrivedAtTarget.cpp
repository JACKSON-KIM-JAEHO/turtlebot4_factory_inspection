#include "turtlebot4_factory_inspection/IsArrivedAtTarget.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

namespace turtlebot4_factory_inspection {

IsArrivedAtTarget::IsArrivedAtTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("is_arrived_at_target_bt_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList IsArrivedAtTarget::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),
    BT::InputPort<int>("current_index"),
    BT::OutputPort<int>("current_index")
  };
}

BT::NodeStatus IsArrivedAtTarget::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> task_list;
  if (!getInput("task_list", task_list)) {
    RCLCPP_ERROR(node_->get_logger(), "[IsArrivedAtTarget] Missing input [task_list]");
    return BT::NodeStatus::FAILURE;
  }

  int current_index;
  if (!getInput("current_index", current_index)) {
    RCLCPP_ERROR(node_->get_logger(), "[IsArrivedAtTarget] Missing input [current_index]");
    return BT::NodeStatus::FAILURE;
  }

  if (current_index < 0 || static_cast<size_t>(current_index) >= task_list.size()) {
    RCLCPP_ERROR(node_->get_logger(), "[IsArrivedAtTarget] current_index (%d) out of bounds", current_index);
    return BT::NodeStatus::FAILURE;
  }

  const auto& target_pose = task_list[current_index].pose;
  double target_x = target_pose.position.x;
  double target_y = target_pose.position.y;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[IsArrivedAtTarget] TF Error: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  double dx = target_x - transform.transform.translation.x;
  double dy = target_y - transform.transform.translation.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  RCLCPP_INFO(node_->get_logger(), "[IsArrivedAtTarget] Distance to target[%d]: %.3f m", current_index, distance);
  
  setOutput("current_index", current_index);
  
  if (distance < 0.2) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace turtlebot4_factory_inspection
