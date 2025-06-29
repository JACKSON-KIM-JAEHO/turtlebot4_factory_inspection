#include "behaviortree_cpp_v3/bt_factory.h"
#include "turtlebot4_factory_inspection/bt_nodes/AdvanceIndex.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/CheckIfSetupDone.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/InitialSetup.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtBase.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsArrivedAtTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/IsNomoreTask.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/MoveToTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/ReturnToBase.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/SetAngle.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/SkipTarget.hpp"
#include "turtlebot4_factory_inspection/bt_nodes/TakePhoto.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "turtlebot4_factory_inspection/utils/load_waypoints.hpp"

#include "rclcpp/rclcpp.hpp"  // ✅ ROS 2 init, shutdown 필요
#include <chrono>
#include <thread>

using namespace turtlebot4_factory_inspection::bt_nodes;

int main(int argc, char **argv)
{
  // ✅ ROS 2 초기화 (필수)
  rclcpp::init(argc, argv);

  // Behavior Tree 설정
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<AdvanceIndex>("AdvanceIndex");
  factory.registerNodeType<CheckIfSetupDone>("CheckIfSetupDone");
  factory.registerNodeType<InitialSetup>("InitialSetup");
  factory.registerNodeType<IsArrivedAtBase>("IsArrivedAtBase");
  factory.registerNodeType<IsArrivedAtTarget>("IsArrivedAtTarget");
  factory.registerNodeType<MoveToTarget>("MoveToTarget");
  factory.registerNodeType<IsNomoreTask>("IsNomoreTask");
  factory.registerNodeType<ReturnToBase>("ReturnToBase");
  factory.registerNodeType<SetAngle>("SetAngle");
  factory.registerNodeType<SkipTarget>("SkipTarget");
  factory.registerNodeType<TakePhoto>("TakePhoto");

  // XML 트리 파일 경로 얻기
  auto blackboard = BT::Blackboard::create();

  //Blackboard 생성 및 경로 설정
  std::string pkg_path = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");
  std::string tree_path = pkg_path + "/trees/main_tree.xml";
 
  std::string yaml_path = pkg_path + "/data/waypoints.yaml";
  blackboard->set<std::string>("waypoints_file_path", yaml_path);

  // 트리 생성
  auto tree = factory.createTreeFromFile(tree_path, blackboard);

  // 루트 노드가 RUNNING일 동안 실행 루프 유지
  while (tree.tickRoot() == BT::NodeStatus::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ✅ ROS 2 종료 (자원 정리)
  rclcpp::shutdown();

  return 0;
}
