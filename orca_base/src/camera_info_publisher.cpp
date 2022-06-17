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

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_shared/context_macros.hpp"

namespace orca_base
{

#define CAMERA_INFO_PARAMS \
  CXT_MACRO_MEMBER(timer_period_ms, int, 1000) \
  CXT_MACRO_MEMBER(camera_info_url, std::string, "/path/to/camera/info") \
  CXT_MACRO_MEMBER(camera_name, std::string, "forward_camera") \
  CXT_MACRO_MEMBER(frame_id, std::string, "forward_camera_frame") \
  /* End of list */

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_DEFINE_MEMBER(n, t, d)

struct CameraInfoContext
{
  CXT_MACRO_DEFINE_MEMBERS(CAMERA_INFO_PARAMS)
};

class CameraInfoPublisher : public rclcpp::Node
{
  CameraInfoContext cxt_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr spin_timer_;

  void validate_parameters()
  {
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this);
    camera_info_manager_->setCameraName(cxt_.camera_name_);

    if (camera_info_manager_->validateURL(cxt_.camera_info_url_)) {
      camera_info_manager_->loadCameraInfo(cxt_.camera_info_url_);
      RCLCPP_INFO(get_logger(), "Loaded camera info from %s", cxt_.camera_info_url_.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "Camera info url '%s' is not valid, missing 'file://' prefix?",
        cxt_.camera_info_url_.c_str());
    }
  }

public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    (void) camera_info_pub_;
    (void) spin_timer_;

    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(CAMERA_INFO_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, CAMERA_INFO_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    CAMERA_INFO_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), CAMERA_INFO_PARAMS)

    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    spin_timer_ = create_wall_timer(
      std::chrono::milliseconds{cxt_.timer_period_ms_}, [this]()
      {
        auto camera_info_msg = camera_info_manager_->getCameraInfo();
        // camera_info_msg.header.stamp = now();
        // camera_info_msg.header.frame_id = cxt_.frame_id_;
        camera_info_pub_->publish(camera_info_msg);
      });
  }
};

}  // namespace orca_base

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::CameraInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
