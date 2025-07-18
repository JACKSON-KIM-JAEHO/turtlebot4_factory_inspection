#ifndef TURTLEBOT4_FACTORY_INSPECTION_BT_NODES_CHECKIFSETUPDONE_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION_BT_NODES_CHECKIFSETUPDONE_HPP_

#include <behaviortree_cpp_v3/condition_node.h>

namespace turtlebot4_factory_inspection {

class CheckIfSetupDone : public BT::ConditionNode
{
public:
  CheckIfSetupDone(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION_BT_NODES_CHECKIFSETUPDONE_HPP_
