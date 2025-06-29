#include "turtlebot4_factory_inspection/bt_nodes/InitialSetup.hpp"
#include "turtlebot4_factory_inspection/utils/load_waypoints.hpp"

#include <rclcpp/rclcpp.hpp>

namespace turtlebot4_factory_inspection::bt_nodes
{

InitialSetup::InitialSetup(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("initial_setup_bt_node"); //initial_setup_bt_node라는 이름으로 ROS 2 노드를 생성
}

BT::PortsList InitialSetup::providedPorts() //BT 노드가 어떤 데이터를 Blackboard로부터 입력받고 출력할 것인지를 정의
{
  return {
    BT::InputPort<std::string>("file_path"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),
    BT::OutputPort<std::vector<double>>("task_yaw_list"),
    BT::OutputPort<int>("current_index"),
    BT::OutputPort<bool>("flag")
  };
}

BT::NodeStatus InitialSetup::tick() //노드가 실제 실행될 때 수행할 동작을 정의하는 핵심 함수 | YAML 파일로부터 데이터를 로딩하는 부분
{
  std::string path;

  if (!getInput("file_path", path)) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialSetup] Missing required input [file_path]");
    return BT::NodeStatus::FAILURE;
  }

  auto [waypoints, yaw_list] = load_waypoints_from_yaml(path);

  if (waypoints.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[InitialSetup] Loaded waypoint list is empty");
    return BT::NodeStatus::FAILURE;
  }

  // Blackboard에 로드된 데이터를 저장하는 부분
  setOutput("task_list", waypoints);
  setOutput("task_yaw_list", yaw_list);
  setOutput("current_index", 0);
  setOutput("flag", true);

//정보성 로그 출력 | 추후 디버깅을 위해서 구현
  RCLCPP_INFO(node_->get_logger(),
    "[InitialSetup] Loaded %lu waypoints from %s",
    waypoints.size(), path.c_str());

  return BT::NodeStatus::SUCCESS;
}

}
