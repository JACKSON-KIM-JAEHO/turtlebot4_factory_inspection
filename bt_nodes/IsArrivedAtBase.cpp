#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp"

namespace turtlebot4_factory_inspection::bt_nodes {

IsArrivedAtBase::IsArrivedAtBase(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList IsArrivedAtBase::providedPorts()
{
  return {};
}

BT::NodeStatus IsArrivedAtBase::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
