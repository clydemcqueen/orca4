#!/usr/bin/env bash

# Modify this for your environment

# Add results of ArduSub build
export PATH=$HOME/ardupilot/build/sitl/bin:$PATH

# Optional: add autotest to the PATH, helpful for running sim_vehicle.py
export PATH=$HOME/ardupilot/Tools/autotest:$PATH

# Add results of Gazebo Garden build
source $HOME/ignition_ws/install/setup.bash

# Add results of orca4 build
source $HOME/orca4_ws/install/setup.bash

# Add ardupilot_gazebo plugin
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH

# Optional: add ardupilot_gazebo models and worlds
export IGN_GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$IGN_GAZEBO_RESOURCE_PATH

# Add bluerov2_ignition models and worlds
export IGN_GAZEBO_RESOURCE_PATH=$HOME/orca4_ws/src/bluerov2_ignition/models:$HOME/orca4_ws/src/bluerov2_ignition/worlds:$IGN_GAZEBO_RESOURCE_PATH

# Add orca4 models and worlds
export IGN_GAZEBO_RESOURCE_PATH=$HOME/orca4_ws/src/orca4/orca_description/models:$HOME/orca4_ws/src/orca4/orca_description/worlds:$IGN_GAZEBO_RESOURCE_PATH
