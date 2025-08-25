# Video Player 模块

这个ROS2模块用于从本地视频文件循环播放视频，提供与相机驱动模块相同的接口。当相机不可用时，可以使用该模块替代相机模块提供视频或图片进行调试。

## 功能特点

- **循环播放**: 自动读取指定目录中的所有视频文件并循环播放
- **相机接口兼容**: 发布与相机驱动模块相同的话题和服务接口
- **多格式支持**: 支持常见的视频格式（.mp4, .avi, .mov, .mkv, .wmv, .flv, .webm, .m4v）
- **参数可配置**: 支持运行时调整分辨率、帧率等参数
- **自动容错**: 当视频文件不存在或损坏时，显示提示信息

## 发布的话题

- `/image_raw` (sensor_msgs/Image): 视频帧图像
- `/camera_info` (sensor_msgs/CameraInfo): 相机信息

## 提供的服务

- `/param_event` (camera_interfaces/srv/ParamEvent): 参数调整服务

## 参数配置

### 主要参数

- `video_directory`: 视频文件目录路径（默认: `/home/shm-white/RM2026/videos`）
- `framerate`: 播放帧率（默认: 30.0）
- `video_width`: 输出图像宽度（默认: 640）
- `video_height`: 输出图像高度（默认: 480）
- `camera_frame_id`: 相机坐标系ID（默认: "camera"）
- `loop_videos`: 是否循环播放（默认: true）

## 使用方法

### 1. 编译模块

```bash
cd /home/shm-white/RM2026
colcon build --packages-select video_player
source install/setup.bash
```

### 2. 准备视频文件

将视频文件放入 `/home/shm-white/RM2026/videos` 目录，或者在启动时指定其他目录。

### 3. 启动节点

使用默认参数启动：
```bash
ros2 run video_player video_player_node
```

使用启动文件启动（可配置参数）：
```bash
ros2 launch video_player video_player.launch.py
```

自定义参数启动：
```bash
ros2 launch video_player video_player.launch.py video_directory:=/path/to/videos framerate:=25.0 video_width:=1280 video_height:=720
```

### 4. 替代相机驱动

要使用视频播放器替代相机驱动模块，可以：

1. 停止相机驱动节点
2. 启动视频播放器节点
3. 如果需要重新映射话题名称，可以在启动文件中修改remappings部分

### 5. 运行时参数调整

可以通过服务调用来动态调整参数：

```bash
# 调整宽度
ros2 service call /param_event camera_interfaces/srv/ParamEvent '{param_name: 2, value: 1280}'

# 调整高度
ros2 service call /param_event camera_interfaces/srv/ParamEvent '{param_name: 1, value: 720}'

# 调整帧率
ros2 service call /param_event camera_interfaces/srv/ParamEvent '{param_name: 3, value: 25.0}'
```

参数名称对应：
- 1: CAMERA_HEIGHT
- 2: CAMERA_WIDTH  
- 3: CAMERA_FRAMERATE

## 目录结构

```
video_player/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── video_player/
│       └── video_player.h
├── src/
│   ├── video_player.cpp
│   └── video_player_node.cpp
├── launch/
│   └── video_player.launch.py
└── config/
    └── video_player_params.yaml
```

## 注意事项

1. **视频格式**: 确保视频文件格式被OpenCV支持
2. **性能**: 大分辨率视频可能影响播放性能，建议根据实际需求调整分辨率
3. **路径权限**: 确保ROS2节点有权限访问视频文件目录
4. **内存使用**: 节点会预先打开所有视频文件，大量视频文件可能占用较多内存

## 调试信息

节点启动时会输出以下信息：
- 找到的视频文件列表
- 当前播放的视频文件
- 视频切换信息
- 参数更改确认

如果视频目录为空或不存在，节点会显示提示信息图像而不是崩溃。
