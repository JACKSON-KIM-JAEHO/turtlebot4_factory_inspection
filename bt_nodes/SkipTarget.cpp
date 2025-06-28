#include "turtlebot4_factory_inspection/SkipTarget.hpp"

namespace turtlebot4_factory_inspection {

SkipTarget::SkipTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList SkipTarget::providedPorts()
{
  return {};
}

BT::NodeStatus SkipTarget::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
