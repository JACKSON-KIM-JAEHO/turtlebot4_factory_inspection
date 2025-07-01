#include "turtlebot4_factory_inspection/bt_nodes/AdvanceIndex.hpp"

namespace turtlebot4_factory_inspection
{

AdvanceIndex::AdvanceIndex(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("advance_index_bt_node");
}

BT::PortsList AdvanceIndex::providedPorts()
{
  return {
    BT::InputPort<int>("current_index"),
    BT::OutputPort<int>("updated_index")  // ✅ 포트 이름 분리!
  };
}

BT::NodeStatus AdvanceIndex::tick()
{
  int index;
  if (!getInput("current_index", index))
  {
    RCLCPP_ERROR(node_->get_logger(), "[AdvanceIndex] ❌ Missing input [current_index]");
    return BT::NodeStatus::FAILURE;
  }

  index += 1;

  if (!setOutput("updated_index", index))
  {
    RCLCPP_ERROR(node_->get_logger(), "[AdvanceIndex] ❌ Failed to set output [updated_index]");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[AdvanceIndex] ✅ Advanced index to: %d", index);

  return BT::NodeStatus::FAILURE;
}

}  // namespace turtlebot4_factory_inspection
