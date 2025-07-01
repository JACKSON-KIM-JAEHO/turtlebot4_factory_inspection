#include "turtlebot4_factory_inspection/bt_nodes/IsNoMoreTask.hpp"
#include <rclcpp/rclcpp.hpp>

namespace turtlebot4_factory_inspection {

IsNoMoreTask::IsNoMoreTask(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config) {}

BT::PortsList IsNoMoreTask::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),
    BT::InputPort<int>("current_index"),
    BT::OutputPort<int>("current_index")
  };
}

BT::NodeStatus IsNoMoreTask::tick()
{
  auto task_list_result = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("task_list");
  auto index_result = getInput<int>("current_index");

  if (!task_list_result || !index_result)
  {
    RCLCPP_ERROR(rclcpp::get_logger("IsNoMoreTask"), "❌ Missing required inputs: task_list or current_index");
    return BT::NodeStatus::FAILURE;
  }

  const auto& task_list = task_list_result.value();
  int index = index_result.value();

  RCLCPP_INFO(rclcpp::get_logger("IsNoMoreTask"),
              "📦 Checking task status... total: %lu, current_index: %d",
              task_list.size(), index);

  // ✅ blackboard 상태 유지를 위해 다시 저장
  setOutput("current_index", index);

  if (index >= static_cast<int>(task_list.size()))
  {
    RCLCPP_INFO(rclcpp::get_logger("IsNoMoreTask"), "✅ No more tasks remaining.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("IsNoMoreTask"), "🕒 Tasks remaining.");
    setOutput("current_index", index);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace turtlebot4_factory_inspection
