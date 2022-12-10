# Set orca4 to AUV mode; base_controller will respond to /cmd_vel messages
ros2 action send_goal /set_target_mode orca_msgs/action/TargetMode "{target_mode: 3}"
