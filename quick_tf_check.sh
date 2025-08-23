#!/bin/bash

# Simple TF checker script that can be run from within the Docker container
echo "=== Multi-AUV TF Status Check ==="
echo ""

echo "Active static transform publishers:"
ros2 node list | grep static_transform_publisher | wc -l
echo ""

echo "Checking key transforms (timeout 2s each):"
echo -n "map -> auv1/base_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv1/base_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo -n "map -> auv1/left_camera_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv1/left_camera_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo -n "map -> auv2/base_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv2/base_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo -n "map -> auv2/left_camera_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv2/left_camera_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo -n "map -> auv3/base_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv3/base_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo -n "map -> auv3/left_camera_link: "
timeout 2s ros2 run tf2_ros tf2_echo map auv3/left_camera_link 2>/dev/null >/dev/null && echo "✓ OK" || echo "✗ Missing"

echo ""
echo "=== TF Check Complete ==="
