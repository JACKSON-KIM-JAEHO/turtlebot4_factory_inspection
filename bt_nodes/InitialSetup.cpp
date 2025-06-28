#include "turtlebot4_factory_inspection/InitialSetup.hpp"

namespace turtlebot4_factory_inspection {

InitialSetup::InitialSetup(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList InitialSetup::providedPorts()
{
  return {};
}

BT::NodeStatus InitialSetup::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
