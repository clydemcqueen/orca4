#!/bin/bash

# Script to create TF tree debugging output during simulation
echo "=== Multi-AUV TF Debugging Script ==="
echo "This script helps debug TF transform issues"
echo ""

echo "--- Checking active static transform publishers ---"
ros2 node list | grep static_transform_publisher | sort

echo ""
echo "--- Checking all TF frames ---"
ros2 run tf2_ros tf2_monitor --all-frames

echo ""
echo "--- Checking transform chain from map to each AUV base_link ---"
echo "AUV1 transform chain:"
ros2 run tf2_ros tf2_echo map auv1/base_link --timeout 2.0

echo ""
echo "AUV2 transform chain:"
ros2 run tf2_ros tf2_echo map auv2/base_link --timeout 2.0

echo ""
echo "AUV3 transform chain:"
ros2 run tf2_ros tf2_echo map auv3/base_link --timeout 2.0

echo ""
echo "--- Checking base controller states ---"
echo "Looking for base controller log messages..."
ros2 topic echo /rosout --once | grep -E "(auv[1-3]_base_controller|manager)" | head -10

echo ""
echo "=== TF Debug Complete ==="
