#include "turtlebot4_factory_inspection/ReturnToBase.hpp"

namespace turtlebot4_factory_inspection {

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

}  // namespace turtlebot4_factory_inspection
