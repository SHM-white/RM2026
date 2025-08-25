#!/bin/bash

# 视频播放器模块测试脚本
echo "=== Video Player 模块测试 ==="

# 检查视频目录
VIDEO_DIR="/home/shm-white/RM2026/videos"
echo "检查视频目录: $VIDEO_DIR"

if [ ! -d "$VIDEO_DIR" ]; then
    echo "创建视频目录..."
    mkdir -p "$VIDEO_DIR"
fi

# 列出视频文件
echo "目录中的视频文件:"
ls -la "$VIDEO_DIR"

VIDEO_COUNT=$(find "$VIDEO_DIR" -name "*.mp4" -o -name "*.avi" -o -name "*.mov" -o -name "*.mkv" 2>/dev/null | wc -l)
echo "找到 $VIDEO_COUNT 个视频文件"

if [ $VIDEO_COUNT -eq 0 ]; then
    echo "警告: 未找到视频文件。节点将显示提示信息。"
    echo "您可以将视频文件放入 $VIDEO_DIR 目录中。"
fi

echo ""
echo "编译video_player模块..."
cd /home/shm-white/RM2026

# 编译模块
colcon build --packages-select video_player --cmake-args -DCMAKE_BUILD_TYPE=Debug

if [ $? -eq 0 ]; then
    echo "编译成功！"
    echo ""
    echo "要测试模块，请运行:"
    echo "source install/setup.bash"
    echo "ros2 run video_player video_player_node"
    echo ""
    echo "或使用启动文件:"
    echo "ros2 launch video_player video_player.launch.py"
    echo ""
    echo "查看发布的话题:"
    echo "ros2 topic list"
    echo "ros2 topic echo /image_raw --once"
else
    echo "编译失败，请检查错误信息"
    exit 1
fi
