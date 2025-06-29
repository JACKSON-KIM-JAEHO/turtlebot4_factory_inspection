#include "turtlebot4_factory_inspection/bt_nodes/SetAngle.hpp"

namespace turtlebot4_factory_inspection::bt_nodes {

SetAngle::SetAngle(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList SetAngle::providedPorts()
{
  return {};
}

BT::NodeStatus SetAngle::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
