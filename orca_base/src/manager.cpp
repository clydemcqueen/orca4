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
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_util/service_client.hpp"
#include "orca_msgs/action/target_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace orca_base
{

// I'm seeing crashes when I re-start nav2. For now just leave it running if it was started.
#undef ALLOW_NAV2_SHUTDOWN

using namespace std::chrono_literals;
using TargetMode = orca_msgs::action::TargetMode;
using GoalHandleTargetMode = rclcpp_action::ServerGoalHandle<TargetMode>;

class Manager : public rclcpp::Node
{
  // String constants
  const std::string MAVROS_ARM_SRV = "/mavros/cmd/arming";
  const std::string MAVROS_SET_MODE_SRV = "/mavros/set_mode";
  const std::string MAVROS_SET_MSG_INTERVAL_SRV = "/mavros/set_message_interval";
  const std::string NAV2_MGR_SRV = "/lifecycle_manager_navigation/manage_nodes";
  const std::string BASE_SRV = "/conn";

  // Parameters
  std::vector<int64_t> mav_msg_ids_;
  int64_t mav_msg_rate_{};

  // ArduSub has quite a few operating modes, we care about these
  // During boot mavros will briefly report the mode as "CMODE(19)", this is the same as "MANUAL"
  const std::string ARDUSUB_MODE_ALT_HOLD = "ALT_HOLD";
  const std::string ARDUSUB_MODE_POS_HOLD = "POSHOLD";
  const std::string ARDUSUB_MODE_MANUAL = "MANUAL";

  // The transition from current_mode to target_mode might take some time, e.g., we might have
  // to wait for the ArduSub EKF to warm up
  std::shared_ptr<GoalHandleTargetMode> goal_handle_;
  uint8_t current_mode_{TargetMode::Goal::ORCA_MODE_DISARMED};
  uint8_t target_mode_{TargetMode::Goal::ORCA_MODE_DISARMED};

  // State
  bool ardusub_connected_{};
  bool ardusub_armed_{};
  std::string ardusub_mode_;
  bool have_pose_{};
  bool nav2_active_{};
  bool base_driving_{};

  // Timers
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr mode_timer_;
  rclcpp::TimerBase::SharedPtr mav_msg_rate_timer_;

  // Action server provided by this node
  rclcpp_action::Server<TargetMode>::SharedPtr set_target_mode_srv_;

  // Services called by this node
  std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>> mavros_arm_client_;
  std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>> mavros_set_mode_client_;
  std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::MessageInterval>>
  mavros_set_msg_interval_client_;
  std::shared_ptr<nav2_util::ServiceClient<nav2_msgs::srv::ManageLifecycleNodes>> nav2_mgr_client_;
  std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> base_client_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ekf_pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

  bool set_arm(bool arm)
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    auto response = std::make_shared<mavros_msgs::srv::CommandBool::Response>();
    request->value = arm;

    RCLCPP_INFO(get_logger(), arm ? "arming..." : "disarming...");
    auto result = mavros_arm_client_->invoke(request, response);
    result = result && response->success;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
  }

  bool set_ardusub_mode(uint8_t base_mode, const std::string & custom_mode)
  {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    auto response = std::make_shared<mavros_msgs::srv::SetMode::Response>();
    request->base_mode = base_mode;
    request->custom_mode = custom_mode;

    RCLCPP_INFO(get_logger(), "setting mode to %s...", custom_mode.c_str());
    auto result = mavros_set_mode_client_->invoke(request, response);
    result = result && response->mode_sent;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
  }

  bool set_message_rate(uint8_t msg_id)
  {
    auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
    auto response = std::make_shared<mavros_msgs::srv::MessageInterval::Response>();
    request->message_id = msg_id;
    request->message_rate = static_cast<float>(mav_msg_rate_);

    RCLCPP_DEBUG(
      get_logger(), "set message rate for %d to %g hz",
      request->message_id, request->message_rate);

    return mavros_set_msg_interval_client_->invoke(request, response) && response->success;
  }

  void set_message_rates()
  {
    RCLCPP_INFO_ONCE(get_logger(), "setting message rates to %ld hz every 10s", mav_msg_rate_);

    for (auto msg_id : mav_msg_ids_) {
      set_message_rate(msg_id);
    }
  }

  bool call_nav2(uint8_t command)
  {
    auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
    auto response = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Response>();
    request->command = command;

    RCLCPP_INFO(get_logger(), "calling nav2...");
    auto result = nav2_mgr_client_->invoke(request, response);
    result = result && response->success;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
  }

  bool call_base(bool conn)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
    request->data = conn;

    RCLCPP_INFO(get_logger(), "calling base...");
    auto result = base_client_->invoke(request, response);
    result = result && response->success;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
  }

  void go_auv()
  {
    if (ardusub_connected_ && have_pose_) {
      if (!ardusub_armed_) {
        set_arm(true);
      }

      if (ardusub_mode_ != ARDUSUB_MODE_ALT_HOLD) {
        set_ardusub_mode(0, ARDUSUB_MODE_ALT_HOLD);
      }

      if (!base_driving_) {
        if (call_base(true)) {
          base_driving_ = true;
        }
      }

      if (!nav2_active_) {
        if (call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP)) {
          nav2_active_ = true;
        }
      }

      // We are in AUV mode when all of these things are true
      // The mode_timer will keep trying until this happens
      if (ardusub_armed_ && ardusub_mode_ == ARDUSUB_MODE_ALT_HOLD && base_driving_ &&
        nav2_active_)
      {
        current_mode_ = TargetMode::Goal::ORCA_MODE_AUV;

        if (goal_handle_) {
          goal_handle_->succeed(std::make_shared<TargetMode::Result>());
          goal_handle_ = nullptr;
        }

        RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_AUV");
      }
    }
  }

  void go_rov()
  {
    if (ardusub_connected_ && have_pose_) {
      // Don't arm or disarm, leave this to the pilot

      // Don't change ArduSub modes, leave this to the pilot

      if (base_driving_) {
        call_base(false);
        base_driving_ = false;
      }

#ifdef ALLOW_NAV2_SHUTDOWN
      // TODO(clyde): this was flaky in Galactic, try in Humble
      if (nav2_active_) {
        call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN);
        nav2_active_ = false;
      }
#endif

      // We are in ROV mode if we have a connection to ArduSub, the rest is optional
      current_mode_ = TargetMode::Goal::ORCA_MODE_ROV;

      if (goal_handle_) {
        goal_handle_->succeed(std::make_shared<TargetMode::Result>());
        goal_handle_ = nullptr;
      }

      RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_ROV");
    }
  }

  void go_disarmed()
  {
    if (ardusub_connected_ /* don't care about have_pose */) {
      if (ardusub_armed_) {
        set_arm(false);
      }

      if (ardusub_mode_ != ARDUSUB_MODE_MANUAL) {
        set_ardusub_mode(0, ARDUSUB_MODE_MANUAL);
      }

      if (base_driving_) {
        call_base(false);
        base_driving_ = false;
      }

#ifdef ALLOW_NAV2_SHUTDOWN
      // TODO(clyde): this was flaky in Galactic, try in Humble
      if (nav2_active_) {
        call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN);
        nav2_active_ = false;
      }
#endif

      // We are DISARMED if we have a connection to ArduSub and the sub is actually disarmed
      // The mode_timer will keep trying until this happens
      if (!ardusub_armed_) {
        current_mode_ = TargetMode::Goal::ORCA_MODE_DISARMED;

        if (goal_handle_) {
          goal_handle_->succeed(std::make_shared<TargetMode::Result>());
          goal_handle_ = nullptr;
        }

        RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_DISARMED");
      }
    }
  }

  void go_to_target_mode()
  {
    if (current_mode_ != target_mode_) {
      switch (target_mode_) {
        case orca_msgs::action::TargetMode::Goal::ORCA_MODE_AUV:
          go_auv();
          break;
        case orca_msgs::action::TargetMode::Goal::ORCA_MODE_ROV:
          go_rov();
          break;
        case orca_msgs::action::TargetMode::Goal::ORCA_MODE_DISARMED:
        default:
          go_disarmed();
          break;
      }
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TargetMode::Goal> goal)  // NOLINT
  {
    (void) uuid;

    // Are we busy?
    if (goal_handle_ || current_mode_ != target_mode_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Is there work to do?
    if (goal->target_mode == current_mode_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTargetMode> goal_handle)  // NOLINT
  {
    assert(goal_handle_ == goal_handle);

    // Cancel the goal
    target_mode_ = current_mode_;
    goal_handle_ = nullptr;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTargetMode> goal_handle)  // NOLINT
  {
    // Execute the goal
    target_mode_ = goal_handle->get_goal()->target_mode;
    goal_handle_ = goal_handle;
  }

  void post_construction_cb()
  {
    // shared_from_this() isn't possible until after the object is fully constructed
    mavros_arm_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>>(
      MAVROS_ARM_SRV, shared_from_this());
    mavros_set_mode_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>>(
      MAVROS_SET_MODE_SRV, shared_from_this());
    mavros_set_msg_interval_client_ =
      std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::MessageInterval>>(
      MAVROS_SET_MSG_INTERVAL_SRV, shared_from_this());
    nav2_mgr_client_ =
      std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::ManageLifecycleNodes>>(
      NAV2_MGR_SRV, shared_from_this());
    base_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(
      BASE_SRV, shared_from_this());

    RCLCPP_INFO(get_logger(), "manager ready");
  }

public:
  Manager()
  : Node{"manager"}
  {
    // Suppress IDE warnings
    (void) ekf_pose_sub_;
    (void) set_target_mode_srv_;
    (void) mode_timer_;
    (void) mav_msg_rate_timer_;
    (void) state_sub_;

    // Key MAV messages:
    // Purpose         MAV id                                    AP id
    // local_position  MAVLINK_MSG_ID_ATTITUDE_QUATERNION (31)   MSG_ATTITUDE_QUATERNION (2)
    // local_position  MAVLINK_MSG_ID_LOCAL_POSITION_NED (32)    MSG_LOCAL_POSITION (48)
    mav_msg_ids_ =
      declare_parameter<std::vector<int64_t>>("msg_ids", std::vector<int64_t>({31, 32}));

    // Common message rate across all MAV messages:
    mav_msg_rate_ = declare_parameter("msg_rate", 20);

    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    rclcpp::QoS reliable(10);
    reliable.reliable();

    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", reliable,
      [this](mavros_msgs::msg::State::ConstSharedPtr msg) // NOLINT
      {
        if (ardusub_connected_ != msg->connected) {
          ardusub_connected_ = msg->connected;
          if (ardusub_connected_) {
            RCLCPP_INFO(get_logger(), "ArduSub connected");

            // Change modes
            mode_timer_ = create_wall_timer(
              1s,
              [this]() -> void {
                go_to_target_mode();
              });

            // A 2nd GCS (e.g., QGC) might change message rates on launch, e.g.,:
            // https://discuss.bluerobotics.com/t/qgroundcontrol-stream-rates/12204
            // Set up a timer to periodically set message rates
            mav_msg_rate_timer_ = create_wall_timer(
              10s,
              [this]() -> void {
                set_message_rates();
              });

            using namespace std::placeholders;
            set_target_mode_srv_ = rclcpp_action::create_server<TargetMode>(
              this,
              "set_target_mode",
              std::bind(&Manager::handle_goal, this, _1, _2),  // NOLINT
              std::bind(&Manager::handle_cancel, this, _1),  // NOLINT
              std::bind(&Manager::handle_accepted, this, _1));  // NOLINT
          } else {
            RCLCPP_INFO(get_logger(), "ArduSub disconnected");
            mode_timer_ = nullptr;
            mav_msg_rate_timer_ = nullptr;
            set_target_mode_srv_ = nullptr;
          }
        }

        if (ardusub_armed_ != msg->armed) {
          ardusub_armed_ = msg->armed;
          RCLCPP_INFO(get_logger(), msg->armed ? "armed" : "disarmed");
        }

        if (ardusub_mode_ != msg->mode) {
          ardusub_mode_ = msg->mode;
          RCLCPP_INFO(get_logger(), "ArduSub mode is %s", msg->mode.c_str());
        }
      });

    ekf_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", best_effort,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr) // NOLINT
      {
        if (!have_pose_) {
          have_pose_ = true;
          RCLCPP_INFO(get_logger(), "EKF is running");
        }
      });

    // Postpone some construction
    init_timer_ = create_wall_timer(
      0s,
      [this]() -> void {
        init_timer_->cancel();  // One-shot
        post_construction_cb();
      });
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
  auto node = std::make_shared<orca_base::Manager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
