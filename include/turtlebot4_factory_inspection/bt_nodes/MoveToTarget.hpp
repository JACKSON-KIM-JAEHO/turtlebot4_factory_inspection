#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__MOVE_TO_TARGET_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__MOVE_TO_TARGET_HPP_

#include <vector>                                      // for std::vector
#include <memory>                                      // for shared_ptr
#include <thread>
#include <mutex>
#include <future>                                      // for std::future_status
#include <cmath>                                       // for std::atan2

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // AMCL 초기 보정 메시지
#include <tf2_ros/buffer.h>                            // TF buffer
#include <tf2_ros/transform_listener.h>                // TF listener
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace turtlebot4_factory_inspection
{

class MoveToTarget : public BT::CoroActionNode
{
public:
  MoveToTarget(const std::string& name, const BT::NodeConfiguration& config);
  ~MoveToTarget();

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::thread spin_thread_;
  std::mutex mutex_;
};

}
#endif  // namespace turtlebot4_factory_inspection::bt_nodes