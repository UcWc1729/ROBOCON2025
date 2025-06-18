#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    投篮车视觉SLAM系统主启动文件
    基于 Jetson Orin Nano + P100R RGB-D + IMU 的纯定位架构
    """
    
    # 启动参数
    use_ekf = LaunchConfiguration('use_ekf')
    localization_only = LaunchConfiguration('localization_only')
    map_file = LaunchConfiguration('map_file')
    
    # 参数声明
    declare_use_ekf = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='是否启用EKF融合'
    )
    
    declare_localization_only = DeclareLaunchArgument(
        'localization_only',
        default_value='true',
        description='是否仅定位模式（不建图）'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='预建地图文件路径'
    )
    
    # 辅助函数获取包路径
    def pkg_launch_path(package_name):
        return os.path.join(get_package_share_directory(package_name), 'launch')
    
    return LaunchDescription([
        # 启动参数声明
        declare_use_ekf,
        declare_localization_only,
        declare_map_file,
        
        # 系统启动日志
        LogInfo(msg="启动投篮车视觉SLAM定位系统..."),
        
        # 1. 设备驱动层
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_launch_path('p100r_camera_driver'), '/p100r_cam.launch.py'
            ])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_launch_path('imu_driver'), '/imu.launch.py'
            ])
        ),
        
        # 2. 同步与预处理层
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_launch_path('sync_proc'), '/sync.launch.py'
            ])
        ),
        
        # 3. 静态TF发布
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_launch_path('tf_pub'), '/static_tf.launch.py'
            ])
        ),
        
        # 4. Isaac ROS Visual SLAM 定位内核
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('isaac_ros_visual_slam'),
                           'launch', 'visual_slam_launch.py')
            ]),
            launch_arguments={
                'localization_only': localization_only,
                'map_file': map_file,
            }.items()
        ),
        
        # 5. 可选EKF融合层
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         pkg_launch_path('ekf_fusion'), '/ekf.launch.py'
        #     ]),
        #     condition=IfCondition(use_ekf)
        # ),
        
        LogInfo(msg="所有节点已启动，系统运行中...")
    ]) 