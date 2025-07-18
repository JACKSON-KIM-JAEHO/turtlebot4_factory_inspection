#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__INITIAL_SETUP_HPP_
#define STURTLEBOT4_FACTORY_INSPECTION__BT_NODES__INITIAL_SETUP_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace turtlebot4_factory_inspection
{

class InitialSetup : public BT::SyncActionNode
{
public:
  InitialSetup(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__INITIAL_SETUP_HPP_
