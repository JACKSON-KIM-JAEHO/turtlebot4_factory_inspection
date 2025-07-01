#ifndef TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ISNOMORETASK_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ISNOMORETASK_HPP_

#include <behaviortree_cpp_v3/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

namespace turtlebot4_factory_inspection {

class IsNoMoreTask : public BT::ConditionNode
{
public:
  IsNoMoreTask(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace turtlebot4_factory_inspection

#endif  // TURTLEBOT4_FACTORY_INSPECTION__BT_NODES__ISNOMORETASK_HPP_
