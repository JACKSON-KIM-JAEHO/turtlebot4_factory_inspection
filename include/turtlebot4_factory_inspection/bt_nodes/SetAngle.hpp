#ifndef SET_ANGLE_HPP_
#define SET_ANGLE_HPP_

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>                                // for task_yaw_list
#include <angles/angles.h>                      // for shortest_angular_distance

class SetAngle : public BT::StatefulActionNode
{
public:
  SetAngle(const std::string& name, const BT::NodeConfiguration& config);

  // 입출력 포트 정의: yaw 리스트, 인덱스, fallback 각도
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void stop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // SET_ANGLE_HPP_
