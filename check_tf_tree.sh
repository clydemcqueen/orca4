#!/bin/bash

# Script to check the TF tree for the multi-AUV simulation
echo "=== Checking TF tree for multi-AUV simulation ==="
echo "This script checks if all required TF transforms are available"
echo ""

echo "--- Checking if tf2_tools is available ---"
which ros2 >/dev/null 2>&1 || { echo "ROS2 not found! Make sure ROS2 is sourced."; exit 1; }

echo "--- Waiting for TF tree to populate (5 seconds) ---"
sleep 5

echo "--- TF Tree Structure ---"
ros2 run tf2_tools view_frames.py --no-gui

echo ""
echo "--- Checking specific transforms ---"

# Function to check if a transform exists
check_transform() {
    local parent=$1
    local child=$2
    echo -n "Checking $parent -> $child: "
    if ros2 run tf2_ros tf2_echo $parent $child --timeout 1.0 >/dev/null 2>&1; then
        echo "✓ Available"
    else
        echo "✗ Missing"
    fi
}

echo "Global transforms:"
check_transform "map" "auv1/map"
check_transform "map" "auv2/map" 
check_transform "map" "auv3/map"

echo ""
echo "AUV1 transforms:"
check_transform "auv1/map" "auv1/slam"
check_transform "auv1/slam" "auv1/down"
check_transform "auv1/map" "auv1/odom"
check_transform "auv1/odom" "auv1/base_link"
check_transform "auv1/base_link" "auv1/left_camera_link"

echo ""
echo "AUV2 transforms:"
check_transform "auv2/map" "auv2/slam"
check_transform "auv2/slam" "auv2/down"
check_transform "auv2/map" "auv2/odom"
check_transform "auv2/odom" "auv2/base_link"
check_transform "auv2/base_link" "auv2/left_camera_link"

echo ""
echo "AUV3 transforms:"
check_transform "auv3/map" "auv3/slam"
check_transform "auv3/slam" "auv3/down"
check_transform "auv3/map" "auv3/odom"
check_transform "auv3/odom" "auv3/base_link"
check_transform "auv3/base_link" "auv3/left_camera_link"

echo ""
echo "--- TF Tree File Generated ---"
echo "Check frames.pdf for a visual representation of the TF tree"
echo ""
echo "--- Multi-AUV TF Check Complete ---"
