#ifndef TURTLEBOT4_FACTORY_INSPECTION__IS_ARRIVED_AT_TARGET_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__IS_ARRIVED_AT_TARGET_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace turtlebot4_factory_inspection::bt_nodes {

class IsArrivedAtTarget : public BT::ConditionNode
{
public:
  IsArrivedAtTarget(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__IS_ARRIVED_AT_TARGET_HPP_
