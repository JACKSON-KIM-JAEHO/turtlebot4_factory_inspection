#include "turtlebot4_factory_inspection/CheckIfSetupDone.hpp"

namespace turtlebot4_factory_inspection {

CheckIfSetupDone::CheckIfSetupDone(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList CheckIfSetupDone::providedPorts()
{
  return {};
}

BT::NodeStatus CheckIfSetupDone::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
