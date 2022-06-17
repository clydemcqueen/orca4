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

#include "orca_shared/model.hpp"
#include "orca_shared/util.hpp"
#include "orca_shared/pwm.hpp"
#include "rclcpp/logging.hpp"

namespace orca
{

geometry_msgs::msg::Accel Model::drag_accel(const geometry_msgs::msg::Twist & vel) const
{
  geometry_msgs::msg::Accel result;
  result.linear.x = drag_accel_x(vel.linear.x);
  result.linear.y = drag_accel_y(vel.linear.y);
  result.linear.z = drag_accel_z(vel.linear.z);
  result.angular.z = drag_accel_yaw(vel.angular.z);
  return result;
}

geometry_msgs::msg::Wrench Model::accel_to_wrench(const geometry_msgs::msg::Accel & accel) const
{
  geometry_msgs::msg::Wrench result;
  result.force.x = accel_to_force(accel.linear.x);
  result.force.y = accel_to_force(accel.linear.y);
  result.force.z = accel_to_force(accel.linear.z);
  result.torque.z = accel_to_torque_yaw(accel.angular.z);
  return result;
}

orca_msgs::msg::Effort Model::wrench_to_effort(const geometry_msgs::msg::Wrench & wrench) const
{
  orca_msgs::msg::Effort result;
  result.force.x = clamp(force_to_effort_xy(wrench.force.x), 1.0);
  result.force.y = clamp(force_to_effort_xy(wrench.force.y), 1.0);
  result.force.z = clamp(force_to_effort_z(wrench.force.z), 1.0);
  result.torque.z = clamp(torque_to_effort_yaw(wrench.torque.z), 1.0);
  return result;
}

orca_msgs::msg::Effort Model::accel_to_effort(const geometry_msgs::msg::Accel & accel) const
{
  return wrench_to_effort(accel_to_wrench(accel));
}

void Model::log_info(const rclcpp::Logger & logger) const
{
  // Describe hover force, effort and pwm
  auto hover_accel = hover_accel_z();
  auto hover_force = accel_to_force(hover_accel);
  auto hover_effort = force_to_effort_z(hover_force);
  auto hover_pwm = orca::effort_to_pwm(mdl_thrust_dz_pwm_, hover_effort);
  RCLCPP_INFO(
    logger, "hover accel: %g, force: %g, effort: %g, pwm: %d",
    hover_accel, hover_force, hover_effort, hover_pwm);

  // Describe force, effort and pwm for a representative forward velocity
  double fwd_velo = 0.4;
  auto fwd_accel = -drag_accel_x(fwd_velo);
  auto fwd_force = accel_to_force(fwd_accel);
  auto fwd_effort = force_to_effort_xy(fwd_force);
  auto fwd_pwm = orca::effort_to_pwm(mdl_thrust_dz_pwm_, fwd_effort);
  RCLCPP_INFO(
    logger, "fwd velo: %g, accel: %g, force: %g, effort: %g, pwm: %d",
    fwd_velo, fwd_accel, fwd_force, fwd_effort, fwd_pwm);
}

}  // namespace orca
