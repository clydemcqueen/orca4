#!/bin/bash

# Multi-AUV Setup Validation Script
# This script checks that all necessary files are in place for the multi-AUV simulation

echo "=== Multi-AUV Setup Validation ==="
echo

# Check model files
echo "Checking AUV model files..."
models=(auv1 auv2 auv3)
for model in "${models[@]}"; do
    if [[ -f "/home/udy/orca4/orca_description/models/$model/model.sdf" ]]; then
        echo "✓ $model model found"
    else
        echo "✗ $model model missing"
    fi
done
echo

# Check world file
echo "Checking world file..."
if [[ -f "/home/udy/orca4/orca_description/worlds/multi_auv_sand.world" ]]; then
    echo "✓ Multi-AUV world file found"
else
    echo "✗ Multi-AUV world file missing"
fi
echo

# Check launch files
echo "Checking launch files..."
launch_files=(
    "/home/udy/orca4/orca_bringup/launch/multi_auv_sim_launch.py"
    "/home/udy/orca4/orca_bringup/launch/multi_auv_bringup.py"
)
for file in "${launch_files[@]}"; do
    if [[ -f "$file" ]]; then
        echo "✓ $(basename $file) found"
    else
        echo "✗ $(basename $file) missing"
    fi
done
echo

# Check parameter files
echo "Checking parameter files..."
param_files=(
    "/home/udy/orca4/orca_bringup/params/auv1_mavros_params.yaml"
    "/home/udy/orca4/orca_bringup/params/auv2_mavros_params.yaml"
    "/home/udy/orca4/orca_bringup/params/auv3_mavros_params.yaml"
    "/home/udy/orca4/orca_bringup/params/auv1_orca_params.yaml"
    "/home/udy/orca4/orca_bringup/params/auv2_orca_params.yaml"
    "/home/udy/orca4/orca_bringup/params/auv3_orca_params.yaml"
)
for file in "${param_files[@]}"; do
    if [[ -f "$file" ]]; then
        echo "✓ $(basename $file) found"
    else
        echo "✗ $(basename $file) missing"
    fi
done
echo

# Check scripts
echo "Checking control scripts..."
scripts=(
    "/home/udy/orca4/orca_base/scripts/formation_controller.py"
    "/home/udy/orca4/orca_base/scripts/multi_auv_path_publisher.py"
    "/home/udy/orca4/orca_base/scripts/formation_mission_runner.py"
)
for script in "${scripts[@]}"; do
    if [[ -f "$script" ]]; then
        if [[ -x "$script" ]]; then
            echo "✓ $(basename $script) found and executable"
        else
            echo "⚠ $(basename $script) found but not executable"
        fi
    else
        echo "✗ $(basename $script) missing"
    fi
done
echo

# Check RViz config
echo "Checking RViz configuration..."
if [[ -f "/home/udy/orca4/orca_bringup/cfg/multi_auv_sim_launch.rviz" ]]; then
    echo "✓ Multi-AUV RViz config found"
else
    echo "✗ Multi-AUV RViz config missing"
fi
echo

# Check camera config files
echo "Checking camera configuration files..."
camera_configs=(
    "/home/udy/orca4/orca_bringup/cfg/auv1_sim_left.ini"
    "/home/udy/orca4/orca_bringup/cfg/auv1_sim_right.ini"
    "/home/udy/orca4/orca_bringup/cfg/auv2_sim_left.ini"
    "/home/udy/orca4/orca_bringup/cfg/auv2_sim_right.ini"
    "/home/udy/orca4/orca_bringup/cfg/auv3_sim_left.ini"
    "/home/udy/orca4/orca_bringup/cfg/auv3_sim_right.ini"
)
for config in "${camera_configs[@]}"; do
    if [[ -f "$config" ]]; then
        echo "✓ $(basename $config) found"
    else
        echo "✗ $(basename $config) missing"
    fi
done
echo

echo "=== Summary ==="
echo "Multi-AUV setup validation complete!"
echo
echo "To run the multi-AUV simulation:"
echo "1. cd /home/udy/orca4/docker"
echo "2. ./build.sh  # Build the Docker image"
echo "3. ./run.sh    # Start the Docker container"
echo "4. ros2 launch orca_bringup multi_auv_sim_launch.py"
echo
echo "To run a formation mission:"
echo "5. ros2 run orca_base formation_mission_runner.py"
echo
