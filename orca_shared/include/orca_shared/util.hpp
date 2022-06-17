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

#ifndef ORCA_SHARED__UTIL_HPP_
#define ORCA_SHARED__UTIL_HPP_

#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace orca
{

//=====================================================================================
// C++ std stuff, e.g., std::clamp is in C++17, but Foxy is still on C++14
//=====================================================================================

template<typename T>
constexpr T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

template<typename T>
constexpr T clamp(const T v, const T minmax)
{
  return clamp(v, -minmax, minmax);
}

template<typename A, typename B>
constexpr B scale(const A a, const A a_min, const A a_max, const B b_min, const B b_max)
{
  return clamp(
    static_cast<B>(b_min + static_cast<double>(b_max - b_min) / (a_max - a_min) * (a - a_min)),
    b_min,
    b_max);
}

//=====================================================================================
// Geometry
//=====================================================================================

double dist_sq(double x, double y);

double dist(double x, double y);

double dist_sq(double x, double y, double z);

double dist(double x, double y, double z);

double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

void get_rpy(const geometry_msgs::msg::Quaternion & q, double & r, double & p, double & y);

void set_rpy(
  geometry_msgs::msg::Quaternion & q, const double & r, const double & p, const double & y);

double get_yaw(const geometry_msgs::msg::Quaternion & q);

void set_yaw(geometry_msgs::msg::Quaternion & q, const double & yaw);

geometry_msgs::msg::Twist robot_to_world_frame(
  const geometry_msgs::msg::Twist & vel, const double & yaw_f_world);

bool is_zero(const geometry_msgs::msg::Twist & v);

//=====================================================================================
// Time
//=====================================================================================

// True if time is valid (non-zero)
bool valid(const rclcpp::Time & stamp);

//=====================================================================================
// Common tf2 functions -- easy to find
//=====================================================================================

tf2::Transform pose_msg_to_transform(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Pose transform_to_pose_msg(const tf2::Transform & transform);

geometry_msgs::msg::Transform transform_to_transform_msg(const tf2::Transform & transform);

geometry_msgs::msg::Transform pose_msg_to_transform_msg(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::TransformStamped pose_msg_to_transform_msg(
  const geometry_msgs::msg::PoseStamped & msg,
  const std::string & child_frame_id);

tf2::Transform transform_msg_to_transform(const geometry_msgs::msg::Transform & msg);

tf2::Transform transform_msg_to_transform(const geometry_msgs::msg::TransformStamped & msg);

geometry_msgs::msg::PoseStamped transform_msg_to_pose_msg(
  const geometry_msgs::msg::TransformStamped & msg);

geometry_msgs::msg::Pose invert(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::PoseStamped invert(
  const geometry_msgs::msg::PoseStamped & msg,
  const std::string & frame_id);

//=====================================================================================
// tf2_ros::Buffer functions
//=====================================================================================

bool transform_with_wait(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  int wait_ms);

bool transform_with_tolerance(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & tolerance);

bool do_transform(
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose);

//=====================================================================================
// str()
//=====================================================================================

#define OSTR(v) std::cout << #v << ": " << orca::str(v) << std::endl;

std::string str(const builtin_interfaces::msg::Time & v);

std::string str(const geometry_msgs::msg::Accel & v);

std::string str(const geometry_msgs::msg::Point & v);

std::string str(const geometry_msgs::msg::Pose & v);

std::string str(const geometry_msgs::msg::PoseStamped & v);

std::string str(const geometry_msgs::msg::Quaternion & v);

std::string str(const geometry_msgs::msg::Twist & v);

std::string str(const geometry_msgs::msg::Vector3 & v);

std::string str(const geometry_msgs::msg::Wrench & v);

std::string str(const rclcpp::Time & v);

std::string str(const std_msgs::msg::Header & v);

std::string str(const tf2::Matrix3x3 & r);

std::string str(const tf2::Transform & t);

std::string str(const tf2::Vector3 & v);

}  // namespace orca

//=====================================================================================
// geometry_msgs::msg operators
//=====================================================================================

namespace geometry_msgs::msg
{

geometry_msgs::msg::Accel operator+(
  const geometry_msgs::msg::Accel & lhs, const geometry_msgs::msg::Accel & rhs);

geometry_msgs::msg::Accel operator-(
  const geometry_msgs::msg::Accel & lhs, const geometry_msgs::msg::Accel & rhs);

geometry_msgs::msg::Accel operator-(const geometry_msgs::msg::Accel & a);

geometry_msgs::msg::Twist operator-(const geometry_msgs::msg::Twist & v);

}  // namespace geometry_msgs::msg

#endif  // ORCA_SHARED__UTIL_HPP_
