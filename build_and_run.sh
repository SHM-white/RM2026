colcon build --parallel-workers 12
source install/setup.bash
ros2 launch auto_aim auto_aim_launch.py
