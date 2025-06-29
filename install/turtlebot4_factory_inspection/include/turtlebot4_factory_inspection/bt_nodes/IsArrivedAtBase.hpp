#ifndef TURTLEBOT4_FACTORY_INSPECTION_IS_ARRIVED_AT_BASE_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION_IS_ARRIVED_AT_BASE_HPP_

#include <behaviortree_cpp_v3/action_node.h>

namespace turtlebot4_factory_inspection::bt_nodes {

class IsArrivedAtBase : public BT::SyncActionNode
{
public:
  IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION_IS_ARRIVED_AT_BASE_HPP_
