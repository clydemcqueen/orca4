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
#include <utility>

#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_base/underwater_motion.hpp"
#include "orca_shared/model.hpp"
#include "orca_shared/pwm.hpp"
#include "orca_shared/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace orca_base
{

// Purposes:
// 1. estimate motion from /cmd_vel, publish /odom, /motion, odom->base transform
// 2. if we're in AUV mode:
//    a. publish pose.z to /mavros/setpoint_position/global
//    b. publish RC overrides for x, y and yaw to /mavros/rc/override

class BaseController : public rclcpp::Node
{
  // Parameters
  BaseContext cxt_;
  rclcpp::Duration slam_timeout_{std::chrono::milliseconds{0}};
  rclcpp::Duration transform_expiration_{std::chrono::milliseconds{0}};

  enum class State
  {
    BOOT,               // Waiting for static transforms
    HAVE_TF,            // Waiting for the EKF to initialize, etc.
    RUN_NO_MAP,         // Running, SLAM has not initialized yet (dead reckoning)
    RUN_LOCALIZED,      // Running, have a map, localized
    RUN_NOT_LOCALIZED,  // Running, have a map, not localized (dead reckoning)
  };

  static const char * state_name(State state)
  {
    if (state == State::BOOT) {
      return "BOOT";
    } else if (state == State::HAVE_TF) {
      return "HAVE_TF";
    } else if (state == State::RUN_NO_MAP) {
      return "RUN_NO_MAP";
    } else if (state == State::RUN_LOCALIZED) {
      return "RUN_LOCALIZED";
    } else {
      return "RUN_NOT_LOCALIZED";
    }
  }

  State state_{State::BOOT};

  bool running() const
  {
    return state_ == State::RUN_NO_MAP || state_ == State::RUN_LOCALIZED ||
           state_ == State::RUN_NOT_LOCALIZED;
  }

  // We are in control
  bool conn_{false};

  // Most recent messages
  geometry_msgs::msg::PoseStamped ardu_pose_;
  rclcpp::Time slam_pose_time_;
  geometry_msgs::msg::Twist cmd_vel_;

  // Motion model
  std::unique_ptr<UnderwaterMotion> underwater_motion_;

  // Static transforms
  tf2::Transform tf_slam_down_;
  tf2::Transform tf_cam_base_;

  // Dynamic transforms
  tf2::Transform tf_ardu_slam_;
  tf2::Transform tf_ardu_odom_;
  tf2::Transform tf_odom_base_;

  // Inverse transforms for speed & stability
  tf2::Transform tf_slam_ardu_;
  tf2::Transform tf_base_odom_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Service provided by this node
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr conn_srv_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ardu_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;

  // Publications
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ext_nav_pub_;
  rclcpp::Publisher<orca_msgs::msg::Motion>::SharedPtr motion_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr setpoint_pub_;

  // TF2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  void change_state(const char * message, State new_state)
  {
    RCLCPP_INFO(get_logger(), "%s, state => %s", message, state_name(new_state));
    state_ = new_state;
  }

  bool get_static_transforms()
  {
    if (tf_buffer_->canTransform(cxt_.slam_frame_id_, cxt_.down_frame_id_, tf2::TimePointZero) &&
      tf_buffer_->canTransform(cxt_.base_frame_id_, cxt_.camera_frame_id_, tf2::TimePointZero))
    {
      tf_slam_down_ = orca::transform_msg_to_transform(
        tf_buffer_->lookupTransform(
          cxt_.slam_frame_id_, cxt_.down_frame_id_, tf2::TimePointZero));

      tf_cam_base_ = orca::transform_msg_to_transform(
        tf_buffer_->lookupTransform(
          cxt_.base_frame_id_, cxt_.camera_frame_id_, tf2::TimePointZero)).inverse();

      return true;
    } else {
      return false;
    }
  }

  void publish_tf(std::string parent, std::string child, const tf2::Transform & tf)
  {
    geometry_msgs::msg::TransformStamped tm;
    tm.header.frame_id = std::move(parent);
    tm.child_frame_id = std::move(child);
    tm.transform = orca::transform_to_transform_msg(tf);
    // Adding time to the transform avoids problems and improves rviz2 display
    tm.header.stamp = now() + transform_expiration_;
    tf_broadcaster_->sendTransform(tm);
  }

  void publish_ext_nav(const tf2::Transform & ext_nav)
  {
    geometry_msgs::msg::PoseStamped ext_nav_msg;
    ext_nav_msg.header.stamp = now();
    ext_nav_msg.header.frame_id = cxt_.ardu_frame_id_;
    ext_nav_msg.pose = orca::transform_to_pose_msg(ext_nav);
    ext_nav_pub_->publish(ext_nav_msg);
  }

  void publish_setpoint()
  {
    tf2::Transform setpoint;
    if (state_ == State::RUN_NO_MAP) {
      setpoint = tf_odom_base_;
    } else {
      setpoint = tf_ardu_odom_ * tf_odom_base_;
    }

    geographic_msgs::msg::GeoPoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = cxt_.ardu_frame_id_;
    msg.pose.position.altitude = orca::transform_to_pose_msg(setpoint).position.z;
    setpoint_pub_->publish(msg);
  }

  void publish_rc()
  {
    if (rc_pub_->get_subscription_count() > 0) {
      mavros_msgs::msg::OverrideRCIn msg;

      // Most channels are not affected
      for (uint16_t & channel : msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }

      // https://www.ardusub.com/developers/rc-input-and-output.html

      // Forward (>1500 is forward)
      msg.channels[5 - 1] = orca::effort_to_pwm(
        cxt_.mdl_thrust_dz_pwm_,
        underwater_motion_->motion().effort.force.x);

      // Lateral (>1500 is to the right when viewed top-down, so flip the sign)
      msg.channels[6 - 1] = orca::effort_to_pwm(
        cxt_.mdl_thrust_dz_pwm_,
        -underwater_motion_->motion().effort.force.y);

      // Yaw (>1500 is clockwise when viewed top-down, so flip the sign)
      msg.channels[4 - 1] = orca::effort_to_pwm(
        cxt_.mdl_thrust_dz_pwm_,
        -underwater_motion_->motion().effort.torque.z);

      rc_pub_->publish(msg);
    }
  }

  void timer_cb()
  {
    if (running()) {
      if (!underwater_motion_) {
        // Initialize underwater motion from the EKF pose
        underwater_motion_ = std::make_unique<UnderwaterMotion>(
          now(),
          get_logger(), cxt_, ardu_pose_.pose);
      } else {
        // Update motion from t-(1/rate) to t
        underwater_motion_->update(now(), cmd_vel_);
      }

      tf_odom_base_ = orca::pose_msg_to_transform(underwater_motion_->motion().pose);
      tf_base_odom_ = tf_odom_base_.inverse();

      motion_pub_->publish(underwater_motion_->motion());
      odom_pub_->publish(underwater_motion_->odometry());

      if (conn_) {
        publish_rc();
        publish_setpoint();
      }
    }

    // Build/update the TF tree
    publish_tf(cxt_.ardu_frame_id_, cxt_.slam_frame_id_, tf_ardu_slam_);
    publish_tf(cxt_.ardu_frame_id_, cxt_.odom_frame_id_, tf_ardu_odom_);
    publish_tf(cxt_.odom_frame_id_, cxt_.base_frame_id_, tf_odom_base_);

    // If we don't have a SLAM pose, send odom as external navigation to the EKF
    if (state_ == State::RUN_NOT_LOCALIZED) {
      publish_ext_nav(tf_ardu_odom_ * tf_odom_base_);
    } else if (state_ != State::RUN_LOCALIZED) {
      publish_ext_nav(tf_odom_base_);
    }

    // State changes
    if (state_ == State::BOOT && get_static_transforms()) {
      change_state("found static transforms", State::HAVE_TF);
    } else if (state_ == State::RUN_LOCALIZED && now() - slam_pose_time_ > slam_timeout_) {
      change_state("SLAM timeout", State::RUN_NOT_LOCALIZED);
    }
  }

  // Output from ArduSub EKF
  void ardu_pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    if (state_ != State::BOOT) {
      ardu_pose_ = *msg;
      if (state_ == State::RUN_LOCALIZED) {
        // The entire pose is usable
        auto tf_ardu_base = orca::pose_msg_to_transform(ardu_pose_.pose);
        tf_ardu_odom_ = tf_ardu_base * tf_base_odom_;
      } else {
        // Only position.z is usable
        auto tab_z = ardu_pose_.pose.position.z;
        auto tob_z = tf_odom_base_.getOrigin().z();
        auto tao_z = tab_z - tob_z;
        RCLCPP_DEBUG(get_logger(), "tf_ardu_odom.z %g", tao_z);
        tf_ardu_odom_.getOrigin().setZ(tao_z);
      }

      if (state_ == State::HAVE_TF) {
        change_state("EKF is running", State::RUN_NO_MAP);
      }
    }
  }

  // Output from SLAM system
  void slam_pose_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
  {
    slam_pose_time_ = msg->header.stamp;

    if (running()) {
      auto tf_orb_cam = orca::pose_msg_to_transform(msg->pose);
      auto tf_slam_base = tf_slam_down_ * tf_orb_cam * tf_cam_base_;

      if (state_ == State::RUN_NO_MAP) {
        // This is our first SLAM pose, so set tf_ardu_slam. Note that the SLAM pose is unfiltered.
        // Future: average N SLAM poses.
        tf_ardu_slam_ = tf_ardu_odom_ * tf_odom_base_ * tf_slam_base.inverse();
        RCLCPP_INFO(get_logger(), "tf_ardu_slam %s", orca::str(tf_ardu_slam_).c_str());
      }

      // Send to ArduSub EKF
      publish_ext_nav(tf_ardu_slam_ * tf_slam_base);

      if (state_ == State::RUN_NO_MAP) {
        change_state("map created", State::RUN_LOCALIZED);
      } else if (state_ == State::RUN_NOT_LOCALIZED) {
        change_state("re-localized", State::RUN_LOCALIZED);
      }
    }
  }

  void validate_parameters()
  {
    slam_timeout_ = {std::chrono::milliseconds{cxt_.slam_timeout_ms_}};
    transform_expiration_ = {std::chrono::milliseconds{cxt_.transform_expiration_ms_}};

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / cxt_.timer_rate_), [this]
      {
        timer_cb();
      });
  }

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

    // Register parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, BASE_ALL_PARAMS, validate_parameters)

    // Log parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

    // Check that all command line parameters are defined
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

public:
  BaseController()
  : Node("base_controller")
  {
    // Suppress IDE warnings
    (void) cmd_vel_sub_;
    (void) conn_srv_;
    (void) ardu_pose_sub_;
    (void) timer_;

    init_parameters();

    // Initial pose
    geometry_msgs::msg::Pose base_f_odom{};
    base_f_odom.position.z = -0.2;

    // Init dynamic transforms
    tf_ardu_slam_.setIdentity();
    tf_ardu_odom_.setIdentity();
    tf_odom_base_ = orca::pose_msg_to_transform(base_f_odom);

    // Init inverse transforms
    tf_slam_ardu_.setIdentity();
    tf_base_odom_ = tf_odom_base_.inverse();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    rclcpp::QoS reliable(10);
    reliable.reliable();

    ext_nav_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/mavros/vision_pose/pose",
      reliable);
    motion_pub_ = create_publisher<orca_msgs::msg::Motion>("motion", reliable);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", reliable);
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", reliable);
    setpoint_pub_ = create_publisher<geographic_msgs::msg::GeoPoseStamped>(
      "/mavros/setpoint_position/global", reliable);

    conn_srv_ = create_service<std_srvs::srv::SetBool>(
      "conn",
      [this](
        const std::shared_ptr<rmw_request_id_t>,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) -> void
      {
        if (conn_ != request->data) {
          if (request->data) {
            RCLCPP_INFO(get_logger(), "reset odometry and start driving");
            underwater_motion_.reset();
          } else {
            RCLCPP_INFO(get_logger(), "stop driving");
            cmd_vel_ = geometry_msgs::msg::Twist{};
          }
          conn_ = request->data;
        }
        response->success = true;
      },
      rmw_qos_profile_services_default);

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", reliable,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) -> void
      {
        cmd_vel_ = *msg;
      });

    ardu_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", best_effort,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void
      {
        ardu_pose_cb(msg);
      });

    slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "camera_pose", reliable,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) -> void
      {
        slam_pose_cb(msg);
      });

    RCLCPP_INFO(get_logger(), "base_controller ready");
  }
};

}  // namespace orca_base

//=============================================================================
// Main
//=============================================================================

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<orca_base::BaseController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
