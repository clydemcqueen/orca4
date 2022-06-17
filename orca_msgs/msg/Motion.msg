std_msgs/Header header

# Inputs
geometry_msgs/Twist cmd_vel
float64 dt

# Outputs
# Pose is in the world frame; acceleration, velocity, force and effort are in the robot frame

geometry_msgs/Accel accel_model   # Accel to move from v0 to v1
geometry_msgs/Accel accel_drag    # Accel to counteract drag
geometry_msgs/Accel accel_total

geometry_msgs/Twist vel

geometry_msgs/Pose pose

geometry_msgs/Wrench force
orca_msgs/Effort effort  # Force normalized to [-1, 1]
