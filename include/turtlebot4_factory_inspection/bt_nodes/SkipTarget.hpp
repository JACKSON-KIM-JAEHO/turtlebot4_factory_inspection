#ifndef SkipTarget_HPP_
#define SkipTarget_HPP_

#include <behaviortree_cpp_v3/action_node.h>

class SkipTarget : public BT::SyncActionNode
{
public:
  SkipTarget(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

#endif  // SkipTarget_HPP_
