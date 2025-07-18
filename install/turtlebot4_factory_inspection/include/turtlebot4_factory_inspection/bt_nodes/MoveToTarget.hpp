#ifndef TURTLEBOT4_FACTORY_INSPECTION__MOVE_TO_TARGET_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__MOVE_TO_TARGET_HPP_

#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace turtlebot4_factory_inspection::bt_nodes
{

class MoveToTarget : public BT::CoroActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MoveToTarget(const std::string& name, const BT::NodeConfiguration& config);
  ~MoveToTarget();

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  typename GoalHandle::SharedPtr goal_handle_;
  std::mutex mutex_;

  std::thread spin_thread_;
};

}  // namespace turtlebot4_factory_inspection::bt_nodes

#endif  // TURTLEBOT4_FACTORY_INSPECTION__MOVE_TO_TARGET_HPP_
