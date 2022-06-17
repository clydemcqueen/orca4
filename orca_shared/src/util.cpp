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

#include "orca_shared/util.hpp"

#include <iomanip>
#include <memory>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca
{

//=====================================================================================
// Geometry
//=====================================================================================

double dist_sq(double x, double y)
{
  return x * x + y * y;
}

double dist(double x, double y)
{
  return sqrt(dist_sq(x, y));
}

double dist_sq(double x, double y, double z)
{
  return x * x + y * y + z * z;
}

double dist(double x, double y, double z)
{
  return sqrt(dist_sq(x, y, z));
}

double dist(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return dist(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

void get_rpy(const geometry_msgs::msg::Quaternion & q, double & r, double & p, double & y)
{
  tf2::Quaternion tf2_q;
  tf2::fromMsg(q, tf2_q);
  tf2::Matrix3x3(tf2_q).getRPY(r, p, y);
}

void
set_rpy(geometry_msgs::msg::Quaternion & q, const double & r, const double & p, const double & y)
{
  tf2::Quaternion tf2_q;
  tf2_q.setRPY(r, p, y);
  q = tf2::toMsg(tf2_q);
}

double get_yaw(const geometry_msgs::msg::Quaternion & q)
{
  double r, p, y;
  get_rpy(q, r, p, y);
  return y;
}

void set_yaw(geometry_msgs::msg::Quaternion & q, const double & yaw)
{
  double r, p, y;
  get_rpy(q, r, p, y);
  set_rpy(q, r, p, yaw);
}

geometry_msgs::msg::Twist
robot_to_world_frame(const geometry_msgs::msg::Twist & vel, const double & yaw_f_world)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = vel.linear.x * std::cos(yaw_f_world) - vel.linear.y * sin(yaw_f_world);
  result.linear.y = vel.linear.x * std::sin(yaw_f_world) + vel.linear.y * cos(yaw_f_world);
  result.linear.z = vel.linear.z;
  result.angular.z = vel.angular.z;
  return result;
}

bool is_zero(const geometry_msgs::msg::Twist & v)
{
  return v.linear.x == 0 && v.linear.y == 0 && v.linear.z == 0 &&
         v.angular.x == 0 && v.angular.y == 0 && v.angular.z == 0;
}

//=====================================================================================
// Time
//=====================================================================================

bool valid(const rclcpp::Time & stamp)
{
  return stamp.nanoseconds() > 0;
}

//=====================================================================================
// Common tf2 functions -- easy to find
//=====================================================================================

tf2::Transform pose_msg_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

geometry_msgs::msg::Pose transform_to_pose_msg(const tf2::Transform & transform)
{
  geometry_msgs::msg::Pose pose;
  tf2::toMsg(transform, pose);
  return pose;
}

geometry_msgs::msg::Transform transform_to_transform_msg(const tf2::Transform & transform)
{
  return tf2::toMsg(transform);
}

geometry_msgs::msg::Transform pose_msg_to_transform_msg(const geometry_msgs::msg::Pose & pose)
{
  return transform_to_transform_msg(pose_msg_to_transform(pose));
}

geometry_msgs::msg::TransformStamped pose_msg_to_transform_msg(
  const geometry_msgs::msg::PoseStamped & msg, const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped result;
  result.header = msg.header;
  result.child_frame_id = child_frame_id;
  result.transform = orca::pose_msg_to_transform_msg(msg.pose);
  return result;
}

tf2::Transform transform_msg_to_transform(const geometry_msgs::msg::Transform & msg)
{
  tf2::Transform transform;
  tf2::fromMsg(msg, transform);
  return transform;
}

tf2::Transform transform_msg_to_transform(const geometry_msgs::msg::TransformStamped & msg)
{
  return transform_msg_to_transform(msg.transform);
}

geometry_msgs::msg::PoseStamped transform_msg_to_pose_msg(
  const geometry_msgs::msg::TransformStamped & msg)
{
  geometry_msgs::msg::PoseStamped result;
  result.header = msg.header;
  result.pose = orca::transform_to_pose_msg(transform_msg_to_transform(msg.transform));
  return result;
}

geometry_msgs::msg::Pose invert(const geometry_msgs::msg::Pose & pose)
{
  return transform_to_pose_msg(pose_msg_to_transform(pose).inverse());
}

geometry_msgs::msg::PoseStamped invert(
  const geometry_msgs::msg::PoseStamped & msg, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame_id;
  result.header.stamp = msg.header.stamp;
  result.pose = orca::invert(msg.pose);
  return result;
}

//=====================================================================================
// tf2_ros::Buffer functions
//=====================================================================================

bool transform_with_wait(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  int wait_ms)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    out_pose = tf->transform(in_pose, frame, std::chrono::milliseconds(wait_ms));
    return true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return false;
  }
}

bool transform_with_tolerance(
  const rclcpp::Logger & logger,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & tolerance)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // Interpolate
    out_pose = tf->transform(in_pose, frame);
    return true;
  } catch (const tf2::ExtrapolationException & e) {
    // Use the most recent transform if possible
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    if ((rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      tolerance)
    {
      RCLCPP_ERROR(
        logger, "Transform too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(), frame.c_str());
      RCLCPP_ERROR(
        logger, "Data: %ds %uns, transform: %ds %uns",
        in_pose.header.stamp.sec, in_pose.header.stamp.nanosec,
        transform.header.stamp.sec, transform.header.stamp.nanosec);
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (const tf2::TransformException & e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return false;
  }
}

bool do_transform(
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::string & frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)
{
  if (tf->canTransform(frame, in_pose.header.frame_id, tf2::TimePointZero)) {
    auto transform = tf->lookupTransform(frame, in_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(in_pose, out_pose, transform);
    out_pose.header.stamp = in_pose.header.stamp;
    return true;
  } else {
    return false;
  }
}

//=====================================================================================
// str()
//=====================================================================================

std::string str(const builtin_interfaces::msg::Time & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    v.sec << "." <<
    v.nanosec <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Accel & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.linear) <<
    str(v.angular) <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Point & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    v.x << ", " <<
    v.y << ", " <<
    v.z <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Pose & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.position) << ", " <<
    str(v.orientation) << ", " <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::PoseStamped & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.header) << ", " <<
    str(v.pose) <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Quaternion & v)
{
  double r, p, y;
  get_rpy(v, r, p, y);
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    r << ", " <<
    p << ", " <<
    y <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Twist & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.linear) << ", " <<
    str(v.angular) <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Vector3 & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    v.x << ", " <<
    v.y << ", " <<
    v.z <<
    "}";

  return s.str();
}

std::string str(const geometry_msgs::msg::Wrench & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.force) << ", " <<
    str(v.torque) <<
    "}";

  return s.str();
}

std::string str(const rclcpp::Time & v)
{
  return str(builtin_interfaces::msg::Time{v});
}

std::string str(const std_msgs::msg::Header & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    str(v.stamp) << ", " <<
    v.frame_id <<
    "}";

  return s.str();
}

std::string str(const tf2::Matrix3x3 & r)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{";
  for (int i = 0; i < 3; ++i) {
    tf2::Vector3 v = r.getRow(i);
    s << "[" << v.x() << ", " << v.y() << ", " << v.z() << "]";
  }
  s << "}";

  return s.str();
}

std::string str(const tf2::Transform & t)
{
  std::stringstream s;

  s << "{" <<
    str(t.getBasis()) << ", " <<
    str(t.getOrigin()) <<
    "}";

  return s.str();
}

std::string str(const tf2::Vector3 & v)
{
  std::stringstream s;

  s << std::fixed << std::setprecision(3) << "{" <<
    v.x() << ", " <<
    v.y() << ", " <<
    v.z() <<
    "}";

  return s.str();
}

}  // namespace orca

//=====================================================================================
// geometry_msgs::msg operators
//=====================================================================================

namespace geometry_msgs::msg
{

geometry_msgs::msg::Accel operator+(
  const geometry_msgs::msg::Accel & lhs,
  const geometry_msgs::msg::Accel & rhs)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = lhs.linear.x + rhs.linear.x;
  result.linear.y = lhs.linear.y + rhs.linear.y;
  result.linear.z = lhs.linear.z + rhs.linear.z;
  result.angular.x = lhs.angular.x + rhs.angular.x;
  result.angular.y = lhs.angular.y + rhs.angular.y;
  result.angular.z = lhs.angular.z + rhs.angular.z;
  return result;
}

geometry_msgs::msg::Accel operator-(
  const geometry_msgs::msg::Accel & lhs,
  const geometry_msgs::msg::Accel & rhs)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = lhs.linear.x - rhs.linear.x;
  result.linear.y = lhs.linear.y - rhs.linear.y;
  result.linear.z = lhs.linear.z - rhs.linear.z;
  result.angular.x = lhs.angular.x - rhs.angular.x;
  result.angular.y = lhs.angular.y - rhs.angular.y;
  result.angular.z = lhs.angular.z - rhs.angular.z;
  return result;
}

geometry_msgs::msg::Accel operator-(const geometry_msgs::msg::Accel & a)
{
  geometry_msgs::msg::Accel result;
  result.linear.x = -a.linear.x;
  result.linear.y = -a.linear.y;
  result.linear.z = -a.linear.z;
  result.angular.x = -a.angular.x;
  result.angular.y = -a.angular.y;
  result.angular.z = -a.angular.z;
  return result;
}

geometry_msgs::msg::Twist operator-(const geometry_msgs::msg::Twist & v)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = -v.linear.x;
  result.linear.y = -v.linear.y;
  result.linear.z = -v.linear.z;
  result.angular.x = -v.angular.x;
  result.angular.y = -v.angular.y;
  result.angular.z = -v.angular.z;
  return result;
}

}  // namespace geometry_msgs::msg
