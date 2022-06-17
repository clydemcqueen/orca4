## Launch files

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo.
Calls [bringup.py](launch/bringup.py).

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including ORB_SLAM2 and Nav2.
Calls [navigation_launch.py](launch/navigation_launch.py).

### [navigation_launch.py](launch/navigation_launch.py)

Nav2 navigation launch file, copied here so I can change the log output.

## Video pipeline

The simulation uses Gazebo camera sensors to generate uncompressed 800x600 images in an
ideal stereo configuration. The frame rate is throttled to 5Hz to reduce CPU load in ORB_SLAM2, but
it can easily go higher.
