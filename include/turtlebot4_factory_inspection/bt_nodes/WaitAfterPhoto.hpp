#pragma once

#include <behaviortree_cpp_v3/action_node.h>  // ✅ 정확한 v3 헤더만 사용
#include <string>

namespace turtlebot4_factory_inspection
{

class WaitAfterPhoto : public BT::SyncActionNode
{
public:
  WaitAfterPhoto(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace turtlebot4_factory_inspection::bt_nodes
