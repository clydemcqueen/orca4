#!/bin/bash

# Quick Start Script for Multi-AUV Formation Simulation
# This script provides a one-command launch for the multi-AUV simulation

echo "========================================"
echo "  Multi-AUV Formation Simulation"
echo "  Starting 3 AUVs in Formation"
echo "========================================"
echo

echo "Launching multi-AUV simulation with:"
echo "- AUV1 (Leader) - Red"
echo "- AUV2 (Left Follower) - Green" 
echo "- AUV3 (Right Follower) - Blue"
echo

echo "Features enabled:"
echo "✓ 3 ArduSub instances (I0, I1, I2)"
echo "✓ Formation flight control"
echo "✓ Individual camera feeds"
echo "✓ Trajectory visualization"
echo "✓ RViz multi-AUV display"
echo

echo "Starting simulation..."
ros2 launch orca_bringup multi_auv_sim_launch.py

echo
echo "To run a formation mission in another terminal:"
echo "ros2 run orca_base formation_mission_runner.py"
