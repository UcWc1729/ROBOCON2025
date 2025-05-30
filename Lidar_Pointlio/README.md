# ROBOCON2025

# lidar_pointlio 激光雷达与IMU定位模块

本目录为ROBOCON 2025“飞身上篮”项目运球车的**激光雷达+IMU融合定位**相关代码及文档。  
主要功能包括：
- 采用宇树雷达 L2（含IMU）进行点云采集与数据融合
- 基于PointLIO算法实现实时定位与构图
- 提供定位信息给运动控制、通信等系统

---

## 目录结构（示例，实际情况先提交上来我看一下）

```plaintext
lidar_pointlio/
├── src/               # 主要源代码（定位、融合、接口）
│   ├── pointlio_main.cpp / .py
│   ├── imu_fusion.cpp / .py
│   └── ...
├── config/            # 配置文件（雷达参数、IMU参数、点云处理参数等）
├── test_data/         # 测试用的rosbag/点云/IMU数据等
├── scripts/           # 实用脚本，如数据可视化、格式转换等
├── utils/             # 公用函数与工具模块
├── requirements.txt   # 依赖列表（Python项目）
└── README.md          # 本说明文件
```

---

## 主要功能（这块由你们来写，这里只是给出格式）

* **点云与IMU融合定位**

  * 利用宇树L2激光雷达点云+内置IMU，实时计算小车在篮球场内的二维/三维位姿
* **PointLIO算法实现**

  * 参考并改进现有PointLIO开源方案，兼容自定义传感器数据格式
* **定位数据输出**

  * 输出x, y, θ（朝向）、速度等信息，可对接ROS话题或自定义通信协议
* **地图构建与调试**

  * 支持场地静态建图/运行时动态修正

---

## 环境配置（这块由你们来写，这里只是给出格式）

1. **推荐环境**

   * Ubuntu 20.04+/ROS Noetic 或 ROS2 Foxy/Humble
   * C++17/GCC 9+ 或 Python 3.8+
2. **依赖安装**

   * 若为ROS节点：

     ```bash
     sudo apt install ros-<distro>-pcl* ros-<distro>-tf2* ros-<distro>-sensor-msgs
     ```
   * Python依赖（如用Open3D/NumPy等）：

     ```bash
     pip install -r requirements.txt
     ```
   * 宇树雷达SDK与驱动，详见`config/`或相关文档

---

## 文件说明（这块由你们来写，这里只是给出格式）

* `src/pointlio_main.cpp` / `.py`
  主体算法入口，完成点云+IMU的融合定位
* `src/imu_fusion.cpp` / `.py`
  单独处理IMU数据，完成与点云的时间对齐
* `config/`
  激光雷达参数、IMU参数、算法超参数等配置
* `test_data/`
  真实或仿真采集的数据包、用于单元测试
* `scripts/`
  数据回放、轨迹可视化、点云格式转换等脚本
* `utils/`
  通用工具代码，如坐标变换、滤波器等

---

## 典型使用方法（这块由你们来写，这里只是给出格式）

* **启动ROS节点（C++/Python）**

  ```bash
  rosrun lidar_pointlio pointlio_main
  ```

  或ROS2：

  ```bash
  ros2 run lidar_pointlio pointlio_main
  ```

* **测试数据回放**

  ```bash
  python scripts/play_rosbag.py --bag test_data/test1.bag
  ```

* **单独可视化定位效果**

  ```bash
  python scripts/plot_trajectory.py --log output/pose.log
  ```

---

## 数据接口与输出（这块由你们来写，这里只是给出格式）

* **主要输出数据**

  * 机器人在场地坐标系下的 (x, y, θ)
  * 当前速度、加速度（可选）
  * 定位置信度/质量因子
* **支持方式**

  * ROS/ROS2话题发布（如`/robot_pose`, `/robot_odom`）
  * 或自定义socket/串口/ESP-NOW协议对接communication模块

---

## 代码规范与贡献约定（这块由你们来写，这里只是给出格式）

* 新增/修改算法或接口，请更新本README和相关文档
* 保持与camera\_yolo、communication等模块的接口一致性


---

## 负责人

* 雷达定位组组长：王昊男
* 其他组员可补充

---

## FAQ与常见问题（这块由你们来写，这里只是给出格式）

* **激光雷达通信异常？**
  检查串口/USB连接、SDK配置，建议用官方工具先单独调试
* **定位漂移大？**
  检查IMU初始化，尝试重置算法参数或增加地图锚点
* **与camera\_yolo融合误差？**
  建议统一场地坐标系，接口前加转换关系

---

如有疑问，请联系组长或在主仓库issues区留言。

---

```

---
