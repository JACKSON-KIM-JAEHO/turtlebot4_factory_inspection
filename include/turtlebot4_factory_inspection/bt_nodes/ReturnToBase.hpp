// include/turtlebot4_factory_inspection/bt_nodes/ReturnToBase.hpp

#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__RETURN_TO_BASE_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__RETURN_TO_BASE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace turtlebot4_factory_inspection
{

class ReturnToBase : public BT::CoroActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ReturnToBase(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  geometry_msgs::msg::PoseStamped load_base_pose();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose base_pose_;
  bool has_pose_ = false;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__RETURN_TO_BASE_HPP_
