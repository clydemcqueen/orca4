# Set orca4 to ROV mode; abort active mission
ros2 action send_goal /set_target_mode orca_msgs/action/TargetMode "{target_mode: 2}"
