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

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orca_vision
{

class PoseToPathNode : public rclcpp::Node
{
  nav_msgs::msg::Path path_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

public:
  PoseToPathNode()
  : Node{"pose_to_path"}
  {
    (void) pose_sub_;

    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
      {
        if (path_pub_->get_subscription_count() > 0) {
          path_.header = msg->header;
          if (path_.poses.size() > 200) {
            path_.poses.clear();
          }
          path_.poses.push_back(*msg);
          path_pub_->publish(path_);
        }
      });
  }
};

}  // namespace orca_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_vision::PoseToPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
