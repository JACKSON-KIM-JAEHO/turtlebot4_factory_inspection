#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>

#include "turtlebot4_factory_inspection/utils/load_waypoints.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/InitialSetup.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/MoveToTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/ReturnToBase.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/SetAngle.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/SkipTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/TakePhoto.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsNoMoreTask.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/AdvanceIndex.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/CheckIfSetupDone.hpp"

using namespace turtlebot4_factory_inspection;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<InitialSetup>("InitialSetup");
  factory.registerNodeType<MoveToTarget>("MoveToTarget");
  factory.registerNodeType<ReturnToBase>("ReturnToBase");
  factory.registerNodeType<SetAngle>("SetAngle");
  factory.registerNodeType<SkipTarget>("SkipTarget");
  factory.registerNodeType<TakePhoto>("TakePhoto");
  factory.registerNodeType<IsArrivedAtBase>("IsArrivedAtBase");
  factory.registerNodeType<IsArrivedAtTarget>("IsArrivedAtTarget");
  factory.registerNodeType<IsNoMoreTask>("IsNoMoreTask");
  factory.registerNodeType<AdvanceIndex>("AdvanceIndex");
  factory.registerNodeType<CheckIfSetupDone>("CheckIfSetupDone");
  // ✅ Blackboard 생성 및 경로 설정
  auto blackboard = BT::Blackboard::create();
  std::string package_path = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");

  blackboard->set<std::string>("waypoints_file_path", package_path + "/data/waypoints.yaml");
  std::string yaml_path = package_path + "/data/waypoints.yaml";
  blackboard->set<std::string>("waypoints_file_path", yaml_path);
  
  auto [task_list, task_yaw_list] = turtlebot4_factory_inspection::load_waypoints_from_yaml(yaml_path);
  blackboard->set("task_list", task_list);
  blackboard->set("task_yaw_list", task_yaw_list);

  std::string tree_file = package_path + "/trees/main_tree.xml";
  auto tree = factory.createTreeFromFile(tree_file, blackboard);

  BT::StdCoutLogger logger(tree);

  // 트리 실행 루프
  while (rclcpp::ok())
  {
    BT::NodeStatus status = tree.tickRoot();

    if (status == BT::NodeStatus::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("bt_runner"), "✅ 트리 성공적으로 종료됨: 모든 작업 완료 및 복귀 완료");
      break;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(rclcpp::get_logger("bt_runner"), "🛑 시스템 종료 중...");
  rclcpp::shutdown();
  return 0;
}