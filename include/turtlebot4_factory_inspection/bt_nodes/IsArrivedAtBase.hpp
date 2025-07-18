// include/turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp

#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace turtlebot4_factory_inspection
{

class IsArrivedAtBase : public BT::SyncActionNode
{
public:
  IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  geometry_msgs::msg::Pose load_base_pose();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  geometry_msgs::msg::Pose current_pose_;
  bool has_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__IS_ARRIVED_AT_BASE_HPP_

