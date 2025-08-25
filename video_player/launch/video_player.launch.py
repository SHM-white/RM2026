from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    package_dir = get_package_share_directory('video_player')
    
    # 声明启动参数
    video_directory_arg = DeclareLaunchArgument(
        'video_directory',
        default_value='/home/shm-white/RM2026/videos',
        description='Directory containing video files'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Video playback framerate'
    )
    
    video_width_arg = DeclareLaunchArgument(
        'video_width',
        default_value='640',
        description='Output video width'
    )
    
    video_height_arg = DeclareLaunchArgument(
        'video_height',
        default_value='480',
        description='Output video height'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera',
        description='Camera frame ID'
    )
    
    # 创建视频播放器节点
    video_player_node = Node(
        package='video_player',
        executable='video_player_node',
        name='video_player_node',
        parameters=[{
            'video_directory': LaunchConfiguration('video_directory'),
            'framerate': LaunchConfiguration('framerate'),
            'video_width': LaunchConfiguration('video_width'),
            'video_height': LaunchConfiguration('video_height'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'loop_videos': True
        }],
        remappings=[
            # 可以在这里重新映射话题名称，以匹配相机驱动模块的话题
            # ('/image_raw', '/camera/image_raw'),
            # ('/camera_info', '/camera/camera_info')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        video_directory_arg,
        framerate_arg,
        video_width_arg,
        video_height_arg,
        camera_frame_id_arg,
        video_player_node
    ])
