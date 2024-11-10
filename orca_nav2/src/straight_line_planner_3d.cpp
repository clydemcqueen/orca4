// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Inspired by
// https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

#include <cmath>
#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "orca_nav2/param_macro.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_nav2
{

class StraightLinePlanner3D : public nav2_core::GlobalPlanner
{
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
  std::string global_frame_id_;

  // Parameters
  double planning_dist_{};
  bool z_before_xy_{};

public:
  StraightLinePlanner3D() = default;
  ~StraightLinePlanner3D() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    auto parent = weak_parent.lock();

    clock_ = parent->get_clock();
    logger_ = parent->get_logger();
    global_frame_id_ = costmap_ros->getGlobalFrameID();  // Map frame

    PARAMETER(parent, name, planning_dist, 0.1)
    PARAMETER(parent, name, z_before_xy, false)

    if (planning_dist_ < 0.01) {
      RCLCPP_WARN(logger_, "planning_dist too low, setting to default 0.1");
      planning_dist_ = 0.1;
    }

    RCLCPP_INFO(logger_, "StraightLinePlanner3D configured");
  }

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  // Move in one 3D segment
  void createOneSegmentPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & global_path)
  {
    int num_poses = static_cast<int>(std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y,
        goal.pose.position.z - start.pose.position.z) /
      planning_dist_);

    double x_increment = (goal.pose.position.x - start.pose.position.x) / num_poses;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / num_poses;
    double z_increment = (goal.pose.position.z - start.pose.position.z) / num_poses;

    geometry_msgs::msg::PoseStamped pose = start;
    pose.header.stamp = clock_->now();
    pose.header.frame_id = global_frame_id_;

    // Controller should ignore orientation, except for final pose
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    for (int i = 0; i < num_poses; ++i) {
      pose.pose.position.x = start.pose.position.x + x_increment * i;
      pose.pose.position.y = start.pose.position.y + y_increment * i;
      pose.pose.position.z = start.pose.position.z + z_increment * i;
      global_path.poses.push_back(pose);
    }
  }

  // Move vertically, then horizontally
  void createTwoSegmentPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & global_path)
  {
    int num_z_poses = static_cast<int>(std::abs(goal.pose.position.z - start.pose.position.z) /
      planning_dist_);

    int num_xy_poses = static_cast<int>(std::hypot(
        goal.pose.position.x - start.pose.position.x,
        goal.pose.position.y - start.pose.position.y) /
      planning_dist_);

    double x_increment = (goal.pose.position.x - start.pose.position.x) / num_xy_poses;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / num_xy_poses;
    double z_increment = (goal.pose.position.z - start.pose.position.z) / num_z_poses;

    geometry_msgs::msg::PoseStamped pose = start;
    pose.header.stamp = clock_->now();
    pose.header.frame_id = global_frame_id_;

    // Controller should ignore orientation, except for final pose
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    // Move vertically
    for (int i = 0; i < num_z_poses; ++i) {
      pose.pose.position.z = start.pose.position.z + z_increment * i;
      global_path.poses.push_back(pose);
    }

    // Goal z
    pose.pose.position.z = goal.pose.position.z;
    global_path.poses.push_back(pose);

    // Move horizontally
    for (int i = 0; i < num_xy_poses; ++i) {
      pose.pose.position.x = start.pose.position.x + x_increment * i;
      pose.pose.position.y = start.pose.position.y + y_increment * i;
      global_path.poses.push_back(pose);
    }
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> /*cancel_checker*/) override
  {
    nav_msgs::msg::Path global_path;

    if (start.header.frame_id != global_frame_id_) {
      RCLCPP_ERROR(logger_, "Start pose must be in the %s frame", global_frame_id_.c_str());
      return global_path;
    }

    if (goal.header.frame_id != global_frame_id_) {
      RCLCPP_ERROR(logger_, "Goal pose must be in the %s frame", global_frame_id_.c_str());
      return global_path;
    }

    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_id_;

    if (z_before_xy_) {
      createTwoSegmentPlan(start, goal, global_path);
    } else {
      createOneSegmentPlan(start, goal, global_path);
    }

    // Goal x, y, z and yaw
    global_path.poses.push_back(goal);

    return global_path;
  }
};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::StraightLinePlanner3D, nav2_core::GlobalPlanner)
