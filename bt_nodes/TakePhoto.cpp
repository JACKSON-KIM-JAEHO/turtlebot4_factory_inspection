#include "turtlebot4_factory_inspection/bt_nodes/TakePhoto.hpp"
#include "cv_bridge/cv_bridge.h"

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

TakePhoto::TakePhoto(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("take_photo_node");

  tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/oakd/rgb/preview/image_raw", 10,
    std::bind(&TakePhoto::imageCallback, this, std::placeholders::_1)
  );
}

void TakePhoto::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_image_ = msg;
}

BT::NodeStatus TakePhoto::tick()
{
  try {
    std::string base_path;
    if (!getInput("save_path", base_path)) {
      RCLCPP_ERROR(node_->get_logger(), "[TakePhoto] ❌ Missing input [save_path]");
      return BT::NodeStatus::SUCCESS;
    }

    int index;
    if (!getInput("current_index", index)) {
      RCLCPP_ERROR(node_->get_logger(), "[TakePhoto] ❌ Missing input [current_index]");
      return BT::NodeStatus::SUCCESS;
    }

    bool is_arrived = true;
    if (!getInput("is_arrived", is_arrived)) {
      RCLCPP_WARN(node_->get_logger(), "[TakePhoto] ⚠️ Missing input [is_arrived], defaulting to true");
    }

    RCLCPP_INFO(node_->get_logger(), "📸 Waiting for image...");
    for (int i = 0; i < 100 && !latest_image_; ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!latest_image_) {
      RCLCPP_WARN(node_->get_logger(), "📛 No image received");
      saveMetadata(base_path, index, geometry_msgs::msg::Pose(), false, "No image received");
      return BT::NodeStatus::SUCCESS;
    }

    auto cv_img = cv_bridge::toCvCopy(latest_image_, "bgr8");

    // Get current pose
    geometry_msgs::msg::Pose robot_pose;
    try {
      auto tf_msg = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
      tf2::Transform tf2_transform;
      tf2::fromMsg(tf_msg.transform, tf2_transform);
      tf2::toMsg(tf2_transform, robot_pose);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "⚠️ TF lookup failed: %s", ex.what());
      robot_pose = geometry_msgs::msg::Pose();
    }

    // Set save path
    std::string folder, filename;
    if (!is_arrived) {
      folder = base_path + "/failed";
      filename = std::to_string(index) + "_failed.jpg";
    } else {
      folder = base_path + "/object" + std::to_string(index);
      filename = std::to_string(index) + ".jpg";
    }

    std::filesystem::create_directories(folder);
    std::string full_path = folder + "/" + filename;

    if (cv::imwrite(full_path, cv_img->image)) {
      RCLCPP_INFO(node_->get_logger(), "✅ Saved image: %s", full_path.c_str());
      saveMetadata(base_path, index, robot_pose, is_arrived, is_arrived ? "" : "Not arrived at target");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "❌ Failed to save image");
      saveMetadata(base_path, index, robot_pose, false, "Image save failed");
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
    latest_image_.reset();

    return BT::NodeStatus::SUCCESS;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "🔥 TakePhoto exception: %s", e.what());
    return BT::NodeStatus::SUCCESS;
  }
}

void TakePhoto::saveMetadata(
  const std::string& base_path, int index,
  const geometry_msgs::msg::Pose& pose,
  bool success, const std::string& reason)
{
  auto now = std::chrono::system_clock::now();
  auto time = std::chrono::system_clock::to_time_t(now);

  std::ostringstream folder, filename;
  if (!success) {
    folder << base_path << "/failed";
    filename << index << "_failed.yaml";
  } else {
    folder << base_path << "/object" << index;
    filename << index << ".yaml";
  }

  std::filesystem::create_directories(folder.str());
  std::ofstream fout(folder.str() + "/" + filename.str());
  if (!fout.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "❌ Failed to open metadata file: %s",
                 (folder.str() + "/" + filename.str()).c_str());
    return;
  }

  fout << "id: " << index << "\n";
  fout << "timestamp: \"" << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << "\"\n";
  fout << "position:\n";
  fout << "  x: " << pose.position.x << "\n";
  fout << "  y: " << pose.position.y << "\n";
  fout << "  z: " << pose.position.z << "\n";
  fout << "rotation:\n";
  fout << "  x: " << pose.orientation.x << "\n";
  fout << "  y: " << pose.orientation.y << "\n";
  fout << "  z: " << pose.orientation.z << "\n";
  fout << "  w: " << pose.orientation.w << "\n";
  fout << "status: \"" << (success ? "normal" : "failed") << "\"\n";
  fout << "filename: \"" << (success ? std::to_string(index) + ".jpg"
                                   : std::to_string(index) + "_failed.jpg") << "\"\n";
  if (!success && !reason.empty()) {
    fout << "reason: \"" << reason << "\"\n";
  }

  fout.close();
}

BT::PortsList TakePhoto::providedPorts()
{
  return {
    BT::InputPort<std::string>("save_path"),
    BT::InputPort<int>("current_index"),
    BT::InputPort<bool>("is_arrived")
  };
}
