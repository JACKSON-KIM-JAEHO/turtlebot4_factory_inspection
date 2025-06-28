#include "turtlebot4_factory_inspection/IsNomoreTask.hpp"

namespace turtlebot4_factory_inspection {

IsNomoreTask::IsNomoreTask(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList IsNomoreTask::providedPorts()
{
  return {};
}

BT::NodeStatus IsNomoreTask::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
