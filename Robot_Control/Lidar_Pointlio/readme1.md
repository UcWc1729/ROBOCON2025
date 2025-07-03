# POINT-LIO-ROS2

## 项目简介
这是一个基于ROS2的POINT-LIO (Point cloud and IMU based LiDAR-Inertial Odometry) 实现。该系统通过融合LiDAR点云数据和IMU数据，实现高精度的实时定位和建图功能。

## 系统要求
- Ubuntu 22.04
- ROS2 Humble
- PCL >= 1.8
- Eigen >= 3.3.4
- OpenMP

## 安装步骤

### 1. 安装依赖
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libeigen3-dev \
    libpcl-dev \
    libgoogle-glog-dev
```

### 2. 编译项目
```bash
# 创建工作空间
mkdir -p ~/point_lio_ws/src
cd ~/point_lio_ws/src

# 克隆代码
git clone https://github.com/YOUR_USERNAME/point_lio_ros2.git

# 编译
cd ..
colcon build
```

### 3. 配置环境
```bash
source install/setup.bash
```

## 使用说明

### 1. 启动系统
```bash
ros2 launch point_lio_ros2 point_lio.launch.py
```

### 2. 数据输入要求
- LiDAR数据: PointCloud2格式
- IMU数据: sensor_msgs/Imu格式

### 3. 参数配置
主要配置文件位于`config`目录下:
- `point_lio.yaml`: 系统主要参数配置
- `sensor.yaml`: 传感器参数配置

## 系统输出
- `/cloud_registered`: 配准后的点云
- `/aft_mapped_to_init`: 里程计信息
- `/path`: 轨迹信息
- `/Laser_map`: 全局地图

## 注意事项
1. 使用前请确保IMU已经正确标定
2. 确保LiDAR和IMU的外参准确
3. 系统启动时需要保持静止以完成初始化

## 常见问题
1. IMU数据偏移
   - 检查IMU标定参数
   - 验证重力向量设置
   - 调整初始化时间

2. 点云配准不准确
   - 检查点云预处理参数
   - 调整特征提取阈值
   - 验证运动畸变补偿

## 维护者
[您的名字]

## 许可证
该项目采用 [许可证类型] 许可证

## 致谢
本项目基于POINT-LIO开发，感谢原作者的工作。
