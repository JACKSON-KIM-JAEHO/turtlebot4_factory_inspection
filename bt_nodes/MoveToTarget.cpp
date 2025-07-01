#include "turtlebot4_factory_inspection/bt_nodes/MoveToTarget.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>  // AMCL 초기 보정
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <angles/angles.h>  // for shortest_angular_distance
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <thread>
#include <chrono>
#include <vector>    // for std::vector
#include <future>    // for std::future_status
#include <cmath>     // for std::atan2


using namespace turtlebot4_factory_inspection;
using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& config)
: BT::CoroActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_to_target_bt_node");
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  spin_thread_ = std::thread([this]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
}

MoveToTarget::~MoveToTarget()
{
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

BT::PortsList MoveToTarget::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("task_list"),
    BT::InputPort<int>("current_index"),
    BT::InputPort<std::vector<int>>("faillist"),
    BT::OutputPort<int>("current_index"),
    BT::OutputPort<bool>("is_arrived"),
    BT::OutputPort<std::vector<int>>("faillist"),
    BT::OutputPort<double>("angle")
  };
}

BT::NodeStatus MoveToTarget::tick()
{
  std::vector<geometry_msgs::msg::PoseStamped> task_list;
  int current_index;
  std::vector<int> fail_list;

  if (!getInput("task_list", task_list)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Missing input [task_list]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("current_index", current_index)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Missing input [current_index]");
    return BT::NodeStatus::SUCCESS;
  }
  getInput("faillist", fail_list);

  if (current_index < 0 || current_index >= static_cast<int>(task_list.size())) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Invalid current_index: %d", current_index);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = task_list[current_index];
  goal.header.frame_id = "map";
  goal.header.stamp = node_->get_clock()->now();

  // 이동 거리 계산 후 1m당 7초 기준으로 timeout 설정
  double max_duration_sec = 10.0;
  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    double dx = goal.pose.position.x - tf.transform.translation.x;
    double dy = goal.pose.position.y - tf.transform.translation.y;
    double distance = std::hypot(dx, dy);
    max_duration_sec = distance * 3.0;
    RCLCPP_INFO(node_->get_logger(), "[MoveToTarget] Distance = %.2f m → Timeout = %.1f s", distance, max_duration_sec);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] TF error during distance calc: %s", ex.what());
    max_duration_sec = 30.0;
  }

  for (int attempt = 1; attempt <= 3; ++attempt) {
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Action server unavailable");
      continue;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal = client_->async_send_goal(goal_msg);
    if (send_goal.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Failed to send goal (attempt %d)", attempt);
      continue;
    }

    auto handle = send_goal.get();
    if (!handle) {
      RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] Goal rejected");
      continue;
    }
    goal_handle_ = handle;

    auto result_future = client_->async_get_result(handle);
    auto start = node_->now();

    while (rclcpp::ok()) {
      if (result_future.wait_for(100ms) == std::future_status::ready) break;
      if ((node_->now() - start).seconds() > max_duration_sec) {
        client_->async_cancel_goal(handle);
        RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Timeout cancel (attempt %d)", attempt);
        break;
      }
      setStatusRunningAndYield();
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      setOutput("is_arrived", true);
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Attempt %d failed", attempt);
    }
  }

  // 최종 실패 처리
  setOutput("is_arrived", false);
  fail_list.push_back(current_index);
  setOutput("faillist", fail_list);

  try {
    auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    double yaw = std::atan2(
      goal.pose.position.y - tf.transform.translation.y,
      goal.pose.position.x - tf.transform.translation.x
    );
    setOutput("angle", yaw);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToTarget] TF error computing angle: %s", ex.what());
  }

  setOutput("current_index", current_index);
  return BT::NodeStatus::SUCCESS;
}

void MoveToTarget::halt()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(node_->get_logger(), "[MoveToTarget] Halt: goal canceled");
  }
  CoroActionNode::halt();
}
