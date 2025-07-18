#ifndef TURTLEBOT4_FACTORY_INSPECTION__UTILS__LOAD_WAYPOINTS_HPP_
#define TURTLEBOT4_FACTORY_INSPECTION__UTILS__LOAD_WAYPOINTS_HPP_

#include <vector>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace turtlebot4_factory_inspection {

std::pair<std::vector<geometry_msgs::msg::PoseStamped>, std::vector<double>>
load_waypoints_from_yaml(const std::string& file_path);

}  // namespace turtlebot4_factory_inspection

#endif
