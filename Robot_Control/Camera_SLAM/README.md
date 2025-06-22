# 投篮车视觉SLAM定位系统

基于 **Jetson Orin Nano Super 8 GB + 博升 P100R RGB-D 相机 + 外接 IMU** 的**纯定位（无路径规划）代码架构蓝图**。架构以 ROS 2 Humble + Isaac ROS Visual SLAM 为核心，并预留了 Kalibr 联合标定与 robot_localization EKF 融合通路，方便后期调优与替换。

## 总览：四层分工

| 层           | 关键包 / 目录                                    | 作用                                     |
| ----------- | ------------------------------------------- | -------------------------------------- |
| **设备驱动层**   | `p100r_camera_driver/`、`imu_driver/`        | 采集 RGB、深度、IMU 原始数据，并发布 `sensor_msgs`   |
| **同步与预处理层** | `sync_proc/`                                | 用 `message_filters` 做时序对齐、去畸变、IMU 坐标变换 |
| **定位内核层**   | `isaac_ros_visual_slam/`（外部包），`ekf_fusion/` | GPU-加速 VSLAM；可选 EKF 融合轮速/IMU           |
| **接口发布层**   | `tf_pub/`、`diagnostics/`                    | 维护 `map→odom→base_link` TF，健康监控、bag 录制 |

### 为什么选择 Isaac ROS Visual SLAM？

* 直接支持 **RGB-D + IMU** 输入，并在 Jetson GPU 上做特征提取/优化，实时性最强
* 官方示例已经给出 **纯定位模式** （预先建图→比赛加载）
* 与 ROS 2 的 `composition` API 深度集成，可把节点装进同一个进程减少拷贝开销

## 目录结构与包划分

```
basketbot_ws/
└── src/
    ├── p100r_camera_driver/
    │   └── launch/p100r_cam.launch.py
    ├── imu_driver/
    │   └── launch/imu.launch.py
    ├── sync_proc/
    │   ├── include/sync_proc/sync.hpp
    │   └── src/sync_node.cpp
    ├── ekf_fusion/              # 可选，robot_localization EKF
    │   ├── config/ekf.yaml
    │   └── launch/ekf.launch.py
    ├── tf_pub/
    │   └── launch/static_tf.launch.py
    ├── calibration/
    │   ├── scripts/kalibr_record.sh
    │   ├── scripts/kalibr_run.sh
    │   └── doc/kalibr_report.pdf
    └── bringup/
        └── launch/basketbot_bringup.launch.py
```

### 要点

* **每个目录是一个独立 ROS 2 package**（`ament_cmake` 或 `ament_python`）
* 把 **Isaac ROS Visual SLAM** 通过 `rosdep` 方式列入 `basketbot_bringup` 的依赖，而不复制源码
* `calibration/` 仅在离线阶段使用，收录 Kalibr yaml、分析脚本，形成外参文件供运行时读取

## 关键节点与话题

| 节点                                 | 订阅                               | 发布                                                    | 说明                                                                 |
| ---------------------------------- | -------------------------------- | ----------------------------------------------------- | ------------------------------------------------------------------ |
| `p100r_cam_node`                   | –                                | `/p100r/color/image_raw`,<br>`/p100r/depth/image_raw` | V4L2/UVC 捕获，FPS 30                                                 |
| `imu_node`                         | –                                | `/imu/data_raw`                                       | BNO055/MPU9250 驱动                                                  |
| `sync_node`                        | 上述三个话题                           | `/synced/rgb`, `/synced/depth`, `/synced/imu`         | `ApproximateTime` 同步；必要时做时戳重写                                      |
| `visual_slam_node` <br>(Isaac ROS) | 同步后话题                            | `/visual_slam/pose`, TF `map→base_link`               | 主定位引擎                |
| `ekf_node` (可选)                    | SLAM pose + `/imu/data_raw` + 轮速 | `/odometry/filtered`, TF `odom→base_link`             | robot_localization EKF |
| `static_tf_pub`                    | –                                | `camera_link↔base_link` 静态 TF                         | 手量测 or 标定给出的外参               |

### 完整 TF 树
`map (VSLAM) → odom (EKF 可选) → base_link → camera_link`

## 典型 Launch 组合

```python
# basketbot_bringup/launch/basketbot_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg = lambda p: os.path.join(get_package_share_directory(p), 'launch')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg('p100r_camera_driver') + '/p100r_cam.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg('imu_driver') + '/imu.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg('sync_proc') + '/sync.launch.py')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('isaac_ros_visual_slam'),
                         'launch/visual_slam_launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg('tf_pub') + '/static_tf.launch.py')),
        # 如需融合再加 EKF
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg('ekf_fusion') + '/ekf.launch.py')),
    ])
```

## 建图

### 建图流程
1. **离线建图阶段**：使用Isaac ROS Visual SLAM的建图模式，收集环境特征点和关键帧
2. **地图保存**：将建图结果保存为地图文件，用于比赛时的纯定位模式
3. **地图验证**：通过回放数据验证地图质量和定位精度

### 建图参数配置
- 相机分辨率：1920x1080 @ 30fps
- 深度图像格式：16UC1
- IMU频率：100Hz
- 特征点数量：1000-2000个

## 定位

### 纯定位模式
比赛时加载预建地图，仅进行位姿估计，不更新地图结构。

### 定位精度优化
- 使用Kalibr进行相机-IMU联合标定
- EKF融合多传感器数据
- 实时监控定位置信度

## 标定与融合流水线

### 1. 单目/双目 + IMU 联合标定

* 录制 `ros2 bag` → 转 `rosbag1` → Kalibr batch 优化
* 生成 `camera.yaml`、`imu.yaml`、`T_cam_imu.yaml`

### 2. 运行时

* `sync_node` 根据 Kalibr 输出加载外参并重映射 IMU 坐标
* `isaac_ros_visual_slam` 读相机内参（`camera_info` topic 自动带）
* 若启用 EKF：`ekf_node` 按 REP-105 参数模板填写 `map_frame`,`odom_frame`,`base_link_frame`

## 扩展：替换 / 对比其他定位核心

| 方案                    | 插槽                 | 替换步骤                                               | 优势                           |
| --------------------- | ------------------ | -------------------------------------------------- | ---------------------------- |
| **VINS-Fusion ROS 2** | `visual_slam_node` | 克隆 `zinuok/VINS-Fusion-ROS2`，更改 `launch` 中的可执行文件即可 | 开源、可定制性强 |
| **ORB-SLAM3 RGB-D**   | 同上                 | 改调用社区 port 并确保 depth image encoding = 32FC1        | 经典算法、鲁棒性好      |

## 持续集成与调试建议

* 在 `bringup` 包加入 **GitHub Actions**：编译 + `colcon test`
* 使用 **ros2 bag play** + `pytest` 对 `synced/*` 话题检查频率和延迟
* 借助 **NVidia Nsight Systems** 分析 GPU 占用，确保 VSLAM 与 CV 模块帧率≥30 Hz

## 系统特性

完成以上架构，系统将具备：

1. **模块化目录**——易于团队分工与迭代
2. **清晰数据流**——任何节点替换都只需改 launch
3. **标定-融合闭环**——相机、IMU、SLAM、EKF 参数各归其位
4. **向 RoboCon 2025 赛场迁移**——赛前建图，赛时纯定位，满足"只定位不规划"的需求

## 参考文档

- [Isaac ROS Visual SLAM 官方文档](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html)
- [Kalibr 相机-IMU标定指南](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration)
- [robot_localization EKF融合](https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html)
- [ROS 2 TF树最佳实践](https://answers.ros.org/question/363919)


