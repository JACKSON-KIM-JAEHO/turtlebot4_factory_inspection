#include "turtlebot4_factory_inspection/utils/load_waypoints.hpp"

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace turtlebot4_factory_inspection {

std::pair<std::vector<geometry_msgs::msg::PoseStamped>, std::vector<double>> load_waypoints_from_yaml(const std::string& file_path)
{
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::vector<double> yaw_list;

  YAML::Node yaml = YAML::LoadFile(file_path);
  if (!yaml["waypoints"]) {

    return {waypoints, yaw_list};
  }

  for (const auto& wp : yaml["waypoints"]) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wp["x"].as<double>();
    pose.pose.position.y = wp["y"].as<double>();

    double yaw_deg = wp["yaw"].as<double>();
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_deg * M_PI / 180.0);
    pose.pose.orientation = tf2::toMsg(q);

    waypoints.push_back(pose);
    yaw_list.push_back(yaw_deg);
  }

  return {waypoints, yaw_list};
}

}  // namespace turtlebot4_factory_inspection