#include "turtlebot4_factory_inspection/bt_nodes/InitialSetup.hpp"
#include "turtlebot4_factory_inspection/utils/load_waypoints.hpp"  // YAML 로딩 함수 포함

#include <rclcpp/rclcpp.hpp>

namespace turtlebot4_factory_inspection
{

InitialSetup::InitialSetup(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("initial_setup_bt_node");
}

BT::PortsList InitialSetup::providedPorts()
{
  return {
    BT::InputPort<std::string>("file_path"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),
    BT::OutputPort<std::vector<double>>("task_yaw_list"),
    BT::OutputPort<int>("current_index"),
    BT::OutputPort<bool>("flag")  // 최초 실행 완료 표시
  };
}

BT::NodeStatus InitialSetup::tick()
{
  std::string path;
  if (!getInput("file_path", path)) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialSetup] ❌ Missing required input [file_path]");
    return BT::NodeStatus::FAILURE;
  }

  auto [waypoints, yaw_list]  = load_waypoints_from_yaml(path);
  if (waypoints.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialSetup] ❌ Loaded waypoint list is empty");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("task_list", waypoints);
  setOutput("task_yaw_list", yaw_list);
  setOutput("current_index", 0);  // ✅ current_index도 반드시 설정해야 함!
  setOutput("flag", true);        // ✅ setup 완료 플래그 설정

  RCLCPP_INFO(node_->get_logger(),
    "[InitialSetup] ✅ Loaded %lu waypoints from %s",
    waypoints.size(), path.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection
