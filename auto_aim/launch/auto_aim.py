import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('camera_driver'), 'launch'),
            '/camera.launch.py']),
    )

    detect_node = Node(
        package="marker_detector",
        executable="marker_detector_node",
        output="screen",
        emulate_tty=True,
        # parameters=[yaml_path],
        name="marker_detector_node",
        respawn=True,
        namespace="camera1"
    )

    tracker_node = Node(
        package="marker_tracker",
        executable="marker_tracker_node",
        output="screen",
        emulate_tty=True,
        # parameters=[yaml_path],
        name="marker_tracker_node",
        respawn=True,
        namespace="camera1"
    )

    launch_description = LaunchDescription(
        [camera_node, detect_node, tracker_node]
    )
    return launch_description
