#!/bin/bash

# Camera Test Launch Script
echo "Starting Camera Test Node..."
echo "This will display camera images with real-time FPS counter"
echo "Press 'q' or ESC in the image window to quit"
echo ""

# Source the ROS2 environment
source /opt/ros/rolling/setup.bash
source install/setup.bash

# Launch the camera test node
ros2 run camera_test camera_test_node
