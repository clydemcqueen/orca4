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

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca_vision
{

#define PARAMS \
  CXT_MACRO_MEMBER(subscribe_best_effort, bool, true) \
  CXT_MACRO_MEMBER(max_poses, int, 500) \
/* End of list */

struct Parameters
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)
  CXT_MACRO_DEFINE_MEMBERS(PARAMS)
};

class OdomToPathNode : public rclcpp::Node
{
  Parameters params_;
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), params_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), params_, PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER( \
    RCLCPP_INFO, get_logger(), params_, n, t, d)
    PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), PARAMS)
  }

  void validate_parameters()
  {
  }

public:
  OdomToPathNode()
  : Node{"odom_to_path"}
  {
    (void) odom_sub_;

    init_parameters();

    // Gazebo p3d plugin uses best-effort QoS
    rclcpp::QoS qos(10);
    if (params_.subscribe_best_effort_) {
      qos.best_effort();
    } else {
      qos.reliable();
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        if (path_pub_->get_subscription_count() > 0) {
          path_.header = msg->header;
          if (path_.poses.size() > (uint64_t)params_.max_poses_) {
            path_.poses.clear();
          }
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header = msg->header;
          pose_stamped.pose = msg->pose.pose;
          path_.poses.push_back(pose_stamped);
          path_pub_->publish(path_);
        }
      });
  }
};

}  // namespace orca_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_vision::OdomToPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
