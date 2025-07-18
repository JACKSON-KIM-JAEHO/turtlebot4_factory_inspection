#include "turtlebot4_factory_inspection/bt_nodes/CheckIfSetupDone.hpp"

namespace turtlebot4_factory_inspection {

CheckIfSetupDone::CheckIfSetupDone(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config) {}

BT::PortsList CheckIfSetupDone::providedPorts()
{
  return {
    BT::InputPort<bool>("flag")
  };
}

BT::NodeStatus CheckIfSetupDone::tick()
{
  bool flag = false;
  if (getInput("flag", flag) && flag) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace turtlebot4_factory_inspection
