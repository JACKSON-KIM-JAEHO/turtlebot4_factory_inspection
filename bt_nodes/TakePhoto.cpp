#include "turtlebot4_factory_inspection/TakePhoto.hpp"

namespace turtlebot4_factory_inspection {

TakePhoto::TakePhoto(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config) {}

BT::PortsList TakePhoto::providedPorts()
{
  return {};
}

BT::NodeStatus TakePhoto::tick()
{
  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
