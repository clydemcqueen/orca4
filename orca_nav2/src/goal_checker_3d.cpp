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

#include <limits>
#include <string>

#include "angles/angles.h"
#include "nav2_core/goal_checker.hpp"
#include "orca_nav2/param_macro.hpp"

/***
* Waiting for PurePursuitController3D to finish decelerating makes the interaction between
* progress_checker & goal_checker somewhat fragile and generally slows down navigation.
*/
#undef WAIT_FOR_DECEL

namespace orca_nav2
{

class GoalChecker3D : public nav2_core::GoalChecker
{
  double xy_goal_tolerance_{};
  double z_goal_tolerance_{};

#ifdef WAIT_FOR_DECEL
  // Most recent cmd_vel message
  geometry_msgs::msg::Twist cmd_vel_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
#endif

public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    auto parent = weak_parent.lock();

    PARAMETER(parent, plugin_name, xy_goal_tolerance, 0.25)
    PARAMETER(parent, plugin_name, z_goal_tolerance, 0.25)

#ifdef WAIT_FOR_DECEL
    cmd_vel_sub_ = parent->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) // NOLINT
      {
        cmd_vel_ = *msg;
      });
#endif

    RCLCPP_INFO(parent->get_logger(), "GoalChecker3D configured");
  }

  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
#ifdef WAIT_FOR_DECEL
    // Wait for PurePursuitController3D to finish decelerating
    if (cmd_vel_.linear.x > 0 || cmd_vel_.linear.z > 0 || cmd_vel_.angular.z > 0) {
      return false;
    }
#endif

    double dx = query_pose.position.x - goal_pose.position.x;
    double dy = query_pose.position.y - goal_pose.position.y;
    double dz = query_pose.position.z - goal_pose.position.z;

    // Check xy position
    if (dx * dx + dy * dy > xy_goal_tolerance_ * xy_goal_tolerance_) {
      return false;
    }

    // Check z position
    return abs(dz) <= z_goal_tolerance_;
  }

  // Return tolerances for use by the controller (added in Galactic)
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    double invalid_field = std::numeric_limits<double>::lowest();

    pose_tolerance.position.x = xy_goal_tolerance_;
    pose_tolerance.position.y = xy_goal_tolerance_;
    pose_tolerance.position.z = z_goal_tolerance_;

    pose_tolerance.orientation.w = invalid_field;
    pose_tolerance.orientation.x = invalid_field;
    pose_tolerance.orientation.y = invalid_field;
    pose_tolerance.orientation.z = invalid_field;

    vel_tolerance.linear.x = invalid_field;
    vel_tolerance.linear.y = invalid_field;
    vel_tolerance.linear.z = invalid_field;

    vel_tolerance.angular.x = invalid_field;
    vel_tolerance.angular.y = invalid_field;
    vel_tolerance.angular.z = invalid_field;

    return true;
  }
};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::GoalChecker3D, nav2_core::GoalChecker)
