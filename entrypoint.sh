#!/bin/bash

source ${IGN_WS}/install/setup.bash
source ${COLCON_WS}/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=${COLCON_WS}/src/bluerov2_ignition/models:${IGN_GAZEBO_RESOURCE_PATH}
export IGN_GAZEBO_RESOURCE_PATH=${COLCON_WS}/src/bluerov2_ignition/worlds:${IGN_GAZEBO_RESOURCE_PATH}
export IGN_GAZEBO_RESOURCE_PATH=${COLCON_WS}/src/orca4/orca_description/models:${IGN_GAZEBO_RESOURCE_PATH}
export IGN_GAZEBO_RESOURCE_PATH=${COLCON_WS}/src/orca4/orca_description/worlds:${IGN_GAZEBO_RESOURCE_PATH}
#ros2 launch orca_bringup sim_launch.py rviz:=false slam:=false nav:=false gzclient:=false
ign gazebo -v 3 -r underwater.world