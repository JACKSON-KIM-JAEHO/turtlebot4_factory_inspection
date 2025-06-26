#include "turtlebot4_factory_inspection/IsArrivedAtTarget.hpp"

namespace turtlebot4_factory_inspection {

IsArrivedAtTarget::IsArrivedAtTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList IsArrivedAtTarget::providedPorts()
{
  return {};
}

BT::NodeStatus IsArrivedAtTarget::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
