#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ADVANCE_INDEX_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ADVANCE_INDEX_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace turtlebot4_factory_inspection
{

class AdvanceIndex : public BT::SyncActionNode
{
public:
  AdvanceIndex(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ADVANCE_INDEX_HPP_
