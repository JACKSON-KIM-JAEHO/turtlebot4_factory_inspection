#include "behaviortree_cpp_v3/bt_factory.h"
#include "turtlebot4_factory_inspection/move_to_target.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"  // ✅ ROS 2 init, shutdown 필요
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
  // ✅ ROS 2 초기화 (필수)
  rclcpp::init(argc, argv);

  // Behavior Tree 설정
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<MoveToTarget>("MoveToTarget");

  // XML 트리 파일 경로 얻기
  std::string pkg_path = ament_index_cpp::get_package_share_directory("turtlebot4_factory_inspection");
  std::string tree_path = pkg_path + "/trees/main_tree.xml";

  // 트리 생성
  auto tree = factory.createTreeFromFile(tree_path);

  // 루트 노드가 RUNNING일 동안 실행 루프 유지
  while (tree.tickRoot() == BT::NodeStatus::RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // ✅ ROS 2 종료 (자원 정리)
  rclcpp::shutdown();

  return 0;
}
