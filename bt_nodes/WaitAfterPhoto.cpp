#include "turtlebot4_factory_inspection/bt_nodes/WaitAfterPhoto.hpp"
#include <chrono>
#include <thread>
#include <iostream>

namespace turtlebot4_factory_inspection
{

WaitAfterPhoto::WaitAfterPhoto(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{}

BT::PortsList WaitAfterPhoto::providedPorts()
{
  return {
    BT::InputPort<int>("wait_time", 5, "Time to wait in seconds")
  };
}

BT::NodeStatus WaitAfterPhoto::tick()
{
  int wait_time = 5;
  getInput("wait_time", wait_time);

  std::cout << "[WaitAfterPhoto] ⏳ Waiting for " << wait_time << " seconds..." << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(wait_time));
  std::cout << "[WaitAfterPhoto] ✅ Done waiting." << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace turtlebot4_factory_inspection::bt_nodes
