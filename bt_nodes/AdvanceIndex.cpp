#include "turtlebot4_factory_inspection/AdvanceIndex.hpp"

namespace turtlebot4_factory_inspection {

AdvanceIndex::AdvanceIndex(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList AdvanceIndex::providedPorts()
{
  return {};
}

BT::NodeStatus AdvanceIndex::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
