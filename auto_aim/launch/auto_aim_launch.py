import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

car_name = "quan_xiang"


def generate_launch_description():
    camera_node = Node(
        package="camera_driver",
        executable="camera_driver_node",
        output="screen",
        emulate_tty=True,  # 用来保证printf可以打印
        parameters=[
            os.path.join(
                get_package_share_directory('auto_aim'),
                'config',
                'camera.yaml'
            )
        ],
        name="camera_driver_node",
        respawn=True,
        namespace=car_name
    )

    serial_node = Node(
        package="robot_serial",
        executable="robot_serial_node",
        output="screen",
        emulate_tty=True,
        name="robot_serial_node",
        respawn=True
    )

    detect_node = Node(
        package="marker_detector",
        executable="marker_detector_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(
                get_package_share_directory('auto_aim'),
                'config',
                car_name,
                'detector.yaml'
            ),
            {
                "model_path":
                    os.path.join(
                        get_package_share_directory('marker_detector'),
                        'buffer_detector',
                        'model',
                        'dafu.xml'
                    )
            }
        ],
        name="marker_detector_node",
        respawn=True,
        namespace=car_name
    )

    tracker_node = Node(
        package="marker_tracker",
        executable="marker_tracker_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(
                get_package_share_directory('auto_aim'),
                'config',
                car_name,
                'tracker.yaml'
            )
        ],
        name="marker_tracker_node",
        respawn=True
    )

    launch_description = LaunchDescription(
        [camera_node, serial_node, detect_node, tracker_node]
    )
    return launch_description
