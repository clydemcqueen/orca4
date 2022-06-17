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

#include <memory>
#include <string>

#include "orca_base/underwater_motion.hpp"
#include "orca_shared/util.hpp"

namespace orca_base
{

// Purpose: given cmd_vel, calculate motion.
//
// Pose is in the world frame; velocity, acceleration and thrust are in the robot frame.
//
// There are several sources of cmd_vel:
// -- orca_nav2::PurePursuitController3D, the primary path following controller.
//    PurePursuitController3D builds a constant acceleration motion (trapezoidal velocity) plan,
//    so [in theory] there's no need to clamp acceleration and velocity.
// -- recovery controllers (spin, wait) interrupt PurePursuitController3D and may cause rapid
//    acceleration or deceleration. These will be clamped.

UnderwaterMotion::UnderwaterMotion(
  const rclcpp::Time & t, const rclcpp::Logger & logger,
  const BaseContext & cxt, const geometry_msgs::msg::Pose & initial_pose)
: logger_{logger}, cxt_{cxt}
{
  motion_.header.stamp = t;
  motion_.header.frame_id = cxt_.odom_frame_id_;
  motion_.pose.position.x = initial_pose.position.x;
  motion_.pose.position.y = initial_pose.position.y;
  motion_.pose.position.z = initial_pose.position.z;
  auto initial_yaw = orca::get_yaw(initial_pose.orientation);
  orca::set_yaw(motion_.pose.orientation, initial_yaw);
  RCLCPP_INFO(
    logger_, "initialize odometry to {{%g, %g, %g}, {0, 0, %g}}",
    motion_.pose.position.x, motion_.pose.position.y, motion_.pose.position.z, initial_yaw);
}

// Loud vs quiet clamp functions
// #define CLAMP(v, minmax) report_and_clamp(__func__, #v, v, minmax)
#define CLAMP(v, minmax) orca::clamp(v, minmax)
#define EPSILON 0.00001

double
UnderwaterMotion::report_and_clamp(std::string func, std::string name, double v, double minmax)
{
  if (v > minmax + EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too high, clamp to %g", func.c_str(), name.c_str(), v,
      minmax);
    return minmax;
  } else if (v < -minmax - EPSILON) {
    RCLCPP_INFO(
      logger_, "%s: {%s} %g too low, clamp to %g", func.c_str(), name.c_str(), v,
      -minmax);
    return -minmax;
  } else {
    return v;
  }
}

// a = (v1 - v0) / dt
geometry_msgs::msg::Accel UnderwaterMotion::calc_accel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Twist & v1) const
{
  geometry_msgs::msg::Accel result;
  result.linear.x = CLAMP((v1.linear.x - v0.linear.x) / motion_.dt, cxt_.x_accel_);
  result.linear.y = CLAMP((v1.linear.y - v0.linear.y) / motion_.dt, cxt_.y_accel_);
  result.linear.z = CLAMP((v1.linear.z - v0.linear.z) / motion_.dt, cxt_.z_accel_);
  result.angular.z = CLAMP((v1.angular.z - v0.angular.z) / motion_.dt, cxt_.yaw_accel_);
  return result;
}

// v = v0 + a * dt
geometry_msgs::msg::Twist UnderwaterMotion::calc_vel(
  const geometry_msgs::msg::Twist & v0,
  const geometry_msgs::msg::Accel & a) const
{
  geometry_msgs::msg::Twist result;
  result.linear.x = CLAMP(v0.linear.x + a.linear.x * motion_.dt, cxt_.x_vel_);
  result.linear.y = CLAMP(v0.linear.y + a.linear.y * motion_.dt, cxt_.y_vel_);
  result.linear.z = CLAMP(v0.linear.z + a.linear.z * motion_.dt, cxt_.z_vel_);
  result.angular.z = CLAMP(v0.angular.z + a.angular.z * motion_.dt, cxt_.yaw_vel_);
  return result;
}

// p = p0 + v * dt
geometry_msgs::msg::Pose UnderwaterMotion::calc_pose(
  const geometry_msgs::msg::Pose & p0,
  const geometry_msgs::msg::Twist & v) const
{
  geometry_msgs::msg::Pose result;
  auto yaw = orca::get_yaw(p0.orientation);
  result.position.x = p0.position.x + (v.linear.x * cos(yaw) + v.linear.y * sin(-yaw)) * motion_.dt;
  result.position.y = p0.position.y + (v.linear.x * sin(yaw) + v.linear.y * cos(-yaw)) * motion_.dt;
  result.position.z = p0.position.z + v.linear.z * motion_.dt;
  yaw += v.angular.z * motion_.dt;
  orca::set_yaw(result.orientation, yaw);

  if (result.position.z > 0) {
    // Don't go above the surface
    result.position.z = 0;
  }

  return result;
}

nav_msgs::msg::Odometry UnderwaterMotion::odometry() const
{
  static std::array<double, 36> covariance{
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  };

  nav_msgs::msg::Odometry result;
  result.header.stamp = motion_.header.stamp;
  result.header.frame_id = cxt_.odom_frame_id_;
  result.child_frame_id = cxt_.base_frame_id_;
  result.pose.pose = motion_.pose;
  result.pose.covariance = covariance;
  result.twist.twist =
    orca::robot_to_world_frame(motion_.vel, orca::get_yaw(motion_.pose.orientation));
  result.twist.covariance = covariance;
  return result;
}

// Coast: if cmd_vel is 0, then force acceleration to 0. Only applies to x, y and yaw.
// Helpful for some ROV operations. TODO(clyde)
void coast(const geometry_msgs::msg::Twist & cmd_vel, geometry_msgs::msg::Accel & model_plus_drag)
{
  static double epsilon = 0.01;
  if (std::abs(cmd_vel.linear.x) < epsilon) {
    model_plus_drag.linear.x = 0;
  }
  if (std::abs(cmd_vel.linear.y) < epsilon) {
    model_plus_drag.linear.y = 0;
  }
  if (std::abs(cmd_vel.angular.z) < epsilon) {
    model_plus_drag.angular.z = 0;
  }
}

void UnderwaterMotion::update(const rclcpp::Time & t, const geometry_msgs::msg::Twist & cmd_vel)
{
  motion_.header.stamp = t;
  motion_.dt = 1.0 / cxt_.timer_rate_;
  motion_.cmd_vel = cmd_vel;

  // Pose and vel don't honor coast TODO(clyde)
  motion_.pose = calc_pose(motion_.pose, motion_.vel);
  motion_.vel = calc_vel(motion_.vel, motion_.accel_model);

  // Accelerate to cmd_vel
  motion_.accel_model = calc_accel(motion_.vel, cmd_vel);

  // Counteract drag
  motion_.accel_drag = -cxt_.drag_accel(motion_.vel);

  // Combine model and drag
  auto accel_total = motion_.accel_model + motion_.accel_drag;

  // Experiment for ROV operations
  if (cxt_.coast_) {
    coast(cmd_vel, accel_total);
  }

  motion_.accel_total = accel_total;
  motion_.force = cxt_.accel_to_wrench(accel_total);
  motion_.effort = cxt_.wrench_to_effort(motion_.force);
}

}  // namespace orca_base
