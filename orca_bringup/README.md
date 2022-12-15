## Launch files

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo.
Calls [bringup.py](launch/bringup.py).

To see parameters: `ros2 launch --show-args orca_bringup sim_launch.py`

A few parameters:
* `bag` bag interesting topics, default false
* `gzclient` run the Gazebo GUI, default true
* `rviz` run rviz2, default true

E.g., run headless: `ros2 launch orca_bringup sim_launch.py gzclient:=false rviz:=false`

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including ORB_SLAM2 and Nav2.
Calls [navigation_launch.py](launch/navigation_launch.py).

### [navigation_launch.py](launch/navigation_launch.py)

Nav2 navigation launch file, modified to avoid launch the velocity smoother.

## Video pipeline

The simulation uses Gazebo camera sensors to generate uncompressed 800x600 images in an
ideal stereo configuration. The frame rate is throttled to 5Hz to reduce CPU load in ORB_SLAM2, but
it can easily go higher.
