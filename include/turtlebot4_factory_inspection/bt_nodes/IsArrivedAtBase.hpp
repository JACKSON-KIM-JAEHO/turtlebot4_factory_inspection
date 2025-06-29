// include/turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp

#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

namespace turtlebot4_factory_inspection::bt_nodes
{

class IsArrivedAtBase : public BT::ConditionNode
{
public:
  IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::Pose load_base_pose();
};

}  // namespace turtlebot4_factory_inspection::bt_nodes

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_
