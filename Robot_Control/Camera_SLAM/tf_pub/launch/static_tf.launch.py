#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    投篮车静态TF发布文件
    定义 base_link -> camera_link -> imu_link 坐标系关系
    基于 Kalibr 联合标定结果和实际机械结构测量
    """
    
    return LaunchDescription([
        # 参数声明 - 可根据实际标定结果调整
        DeclareLaunchArgument(
            'camera_x', default_value='0.15',
            description='相机相对base_link的X偏移 (前方向)'
        ),
        DeclareLaunchArgument(
            'camera_y', default_value='0.0',
            description='相机相对base_link的Y偏移 (左方向)'  
        ),
        DeclareLaunchArgument(
            'camera_z', default_value='0.25',
            description='相机相对base_link的Z偏移 (上方向)'
        ),
        DeclareLaunchArgument(
            'camera_roll', default_value='0.0',
            description='相机roll角 (绕X轴旋转)'
        ),
        DeclareLaunchArgument(
            'camera_pitch', default_value='0.0', 
            description='相机pitch角 (绕Y轴旋转)'
        ),
        DeclareLaunchArgument(
            'camera_yaw', default_value='0.0',
            description='相机yaw角 (绕Z轴旋转)'
        ),
        
        # base_link -> camera_link 静态变换
        # 相机光学中心相对于机器人底盘中心的位置
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                LaunchConfiguration('camera_x'),
                LaunchConfiguration('camera_y'), 
                LaunchConfiguration('camera_z'),
                LaunchConfiguration('camera_roll'),
                LaunchConfiguration('camera_pitch'),
                LaunchConfiguration('camera_yaw'),
                'base_link',
                'camera_link'
            ]
        ),
        
        # camera_link -> camera_color_optical_frame
        # ROS 相机光学坐标系 (Z向前, X向右, Y向下)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher', 
            name='camera_to_optical',
            arguments=[
                '0', '0', '0',                    # 无位移
                '-0.5', '0.5', '-0.5', '0.5',     # 旋转90度对齐光学坐标系
                'camera_link',
                'camera_color_optical_frame'
            ]
        ),
        
        # camera_link -> camera_depth_optical_frame
        # 深度相机光学坐标系 (通常与彩色相机对齐)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_depth_optical',
            arguments=[
                '0', '0', '0',                    # P100R RGB-D 已对齐
                '-0.5', '0.5', '-0.5', '0.5',
                'camera_link', 
                'camera_depth_optical_frame'
            ]
        ),
        
        # base_link -> imu_link 
        # IMU传感器相对于机器人底盘的位置 (需要根据实际安装位置调整)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=[
                '0.1', '0.0', '0.3',   # IMU安装位置 (根据实际情况调整)
                '0', '0', '0', '1',    # 假设IMU与base_link同向
                'base_link',
                'imu_link'
            ]
        ),
        
        # 机器人模型中心 -> 底盘中心 (如果需要)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_to_base',
            arguments=[
                '0', '0', '0',
                '0', '0', '0', '1',
                'robot_base',
                'base_link'
            ]
        ),
        
        # 激光雷达TF (如果安装了激光雷达)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_to_laser',
        #     arguments=[
        #         '0.2', '0', '0.1',  # 激光雷达安装位置
        #         '0', '0', '0', '1',
        #         'base_link',
        #         'laser_link'
        #     ]
        # ),
    ]) 