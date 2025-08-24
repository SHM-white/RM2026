#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_test',
            executable='camera_test_node',
            name='camera_test_node',
            output='screen',
            parameters=[],
            remappings=[
                # 如果需要重映射话题名称，可以在这里添加
                # ('image_raw', '/camera/image_raw'),
            ]
        )
    ])
