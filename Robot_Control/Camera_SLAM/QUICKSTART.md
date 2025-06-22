# 快速启动指南

## 环境准备

### 1. 硬件要求
- Jetson Orin Nano Super 8GB
- 博升 P100R RGB-D 相机
- 外接 IMU (BNO055/MPU9250)
- 标定板 (6x6 Aprilgrid)

### 2. 软件依赖
```bash
# ROS 2 Humble 基础包
sudo apt install ros-humble-desktop

# Isaac ROS Visual SLAM
sudo apt install ros-humble-isaac-ros-visual-slam

# 其他依赖
sudo apt install ros-humble-robot-localization \
                 ros-humble-message-filters \
                 ros-humble-cv-bridge \
                 ros-humble-image-transport

# Kalibr 标定工具
pip3 install rosbags
```

## 使用流程

### 第一步：硬件连接与测试
```bash
# 1. 连接相机和IMU
# 2. 测试设备是否正常识别
lsusb  # 检查USB设备
ls /dev/ttyUSB*  # 检查IMU串口

# 3. 启动单独的驱动测试
ros2 launch p100r_camera_driver p100r_cam.launch.py
ros2 launch imu_driver imu.launch.py

# 4. 检查话题数据
ros2 topic echo /p100r/color/image_raw
ros2 topic echo /imu/data_raw
```

### 第二步：相机-IMU联合标定
```bash
# 1. 录制标定数据 (60秒)
cd calibration/scripts
bash kalibr_record.sh

# 2. 执行离线标定
bash kalibr_run.sh <bag_name>

# 3. 查看标定结果
ls ../doc/calibration_results/
```

### 第三步：更新标定参数
```bash
# 根据标定结果更新以下文件:
# - tf_pub/launch/static_tf.launch.py (外参)
# - p100r_camera_driver/config/camera_info.yaml (内参)
```

### 第四步：系统启动
```bash
# 启动完整系统 (纯定位模式)
ros2 launch bringup basketbot_bringup.launch.py \
    localization_only:=true \
    map_file:=/path/to/your/map.yaml

# 可选：启用EKF融合
ros2 launch bringup basketbot_bringup.launch.py \
    use_ekf:=true
```

## 验证与调试

### 话题监控
```bash
# 检查关键话题
ros2 topic list | grep -E "(visual_slam|synced|imu)"

# 监控位姿输出
ros2 topic echo /visual_slam/pose

# 检查TF树
ros2 run tf2_tools view_frames
```

### 性能监控
```bash
# 话题频率检查
ros2 topic hz /synced/rgb
ros2 topic hz /visual_slam/pose

# GPU占用 (Jetson)
sudo tegrastats
```

### 常见问题

1. **相机图像不清晰**
   - 检查焦距设置
   - 调整曝光参数
   - 确保充足光照

2. **IMU数据异常**
   - 检查串口权限: `sudo chmod 666 /dev/ttyUSB0`
   - 确认波特率设置
   - 验证IMU校准状态

3. **SLAM定位失败**
   - 确保环境有足够纹理特征
   - 检查相机-IMU时间同步
   - 验证外参标定质量

4. **系统延迟过高**
   - 降低图像分辨率
   - 调整SLAM参数
   - 检查CPU/GPU负载

## 性能优化建议

### Jetson 优化
```bash
# 设置最大性能模式
sudo nvpmodel -m 0
sudo jetson_clocks

# 检查功耗状态
sudo nvpmodel -q
```

### ROS 2 优化
- 使用 composition 减少进程间通信开销
- 调整 QoS 策略提高实时性
- 启用多线程执行器

### SLAM 参数调优
- 特征点数量: 1000-2000
- 关键帧间隔: 根据运动速度调整
- 循环检测: 平衡精度与计算量

## 技术支持

如遇问题，请检查：
1. README.md 中的详细架构说明
2. 各模块的launch文件参数配置
3. 标定报告中的误差分析
4. ROS 2 和 Isaac ROS 官方文档 