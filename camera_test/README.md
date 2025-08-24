# Camera Test Package

这个包用于测试相机驱动，实时显示相机图像并显示FPS。

## 功能特性

- 订阅相机驱动的 `image_raw` 话题
- 实时显示图像到屏幕
- 显示实时FPS计数器
- 显示图像尺寸信息
- 支持键盘退出控制（按 'q' 或 ESC 键退出）

## 编译

在工作空间根目录下运行：

```bash
colcon build --packages-select camera_test
source install/setup.bash
```

## 运行

### 方法1：使用提供的启动脚本
```bash
cd src/camera_test
./launch_camera_test.sh
```

### 方法2：使用launch文件
```bash
source install/setup.bash
ros2 launch camera_test camera_test_launch.py
```

### 方法3：直接运行节点
```bash
# 确保已经 source 了环境
source install/setup.bash

# 运行节点
ros2 run camera_test camera_test_node
```

### 方法4：使用完整测试脚本
```bash
# 在工作空间根目录下运行
./test_camera_driver.sh
```

## 使用说明

1. 确保相机驱动节点已经启动并发布图像到 `image_raw` 话题
2. 运行测试节点，会打开一个OpenCV窗口显示图像
3. 窗口左上角会显示实时FPS
4. 窗口左下角会显示图像尺寸和退出提示
5. 按 'q' 或 ESC 键退出程序

## 依赖项

- ROS2 Rolling
- OpenCV
- cv_bridge
- sensor_msgs

## 话题接口

**订阅的话题：**
- `image_raw` (sensor_msgs/Image) - 从相机驱动接收图像数据

## 注意事项

- 使用 BEST_EFFORT QoS 策略以获得最佳性能
- 支持多种图像编码格式，自动转换为BGR8用于显示
- FPS计算基于实际接收到的帧数，每秒更新一次
- 程序会在终端输出当前FPS信息
