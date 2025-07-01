#ifndef TAKE_PHOTO_HPP
#define TAKE_PHOTO_HPP

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class TakePhoto : public BT::SyncActionNode
{
public:
  TakePhoto(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  void saveMetadata(
    const std::string& base_path,
    int index,
    const geometry_msgs::msg::Pose& pose,
    bool success,
    const std::string& reason = "");

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  sensor_msgs::msg::Image::SharedPtr latest_image_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // TAKE_PHOTO_HPP
