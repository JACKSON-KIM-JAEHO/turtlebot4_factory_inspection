#include "turtlebot4_factory_inspection/bt_nodes/ReturnToBase.hpp"

namespace turtlebot4_factory_inspection::bt_nodes {

ReturnToBase::ReturnToBase(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList ReturnToBase::providedPorts()
{
  return {};
}

BT::NodeStatus ReturnToBase::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection::bt_nodes
