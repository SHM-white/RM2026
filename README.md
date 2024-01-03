# RM2024赛季自瞄总仓库
## 使用方法
### 编译
```
mkdir -p ./RM2024/src
cd ./RM2024/src
git clone --recursive git@github.com:nuaa-rm/RM2024.git .
cd ..
colcon build --parallel-workers 12
```
### 运行
```ros2 launch auto_aim auto_aim.py```
