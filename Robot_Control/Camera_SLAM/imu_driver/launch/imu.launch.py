#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    外接IMU传感器驱动启动文件
    支持BNO055/MPU9250等常见IMU芯片
    """
    
    # 启动参数
    imu_type = LaunchConfiguration('imu_type')
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate = LaunchConfiguration('publish_rate')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'imu_type',
            default_value='bno055',
            description='IMU传感器类型 (bno055/mpu9250/icm20948)'
        ),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='IMU串口设备路径'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='IMU坐标系名称'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='100',
            description='IMU数据发布频率 (Hz)'
        ),
        
        # BNO055 IMU 节点
        Node(
            package='bno055',
            executable='bno055_driver',
            name='bno055_node',
            parameters=[{
                'device': serial_port,
                'frame_id': frame_id,
                'frequency': publish_rate,
                'operation_mode': 'NDOF',  # 9-DOF 融合模式
                'publish_tf': False,  # 不发布TF，由外部管理
                'use_magnetometer': True,
                'calibration_mode': False,
            }],
            remappings=[
                ('/bno055/imu', '/imu/data_raw'),
                ('/bno055/mag', '/imu/mag'),
                ('/bno055/temperature', '/imu/temperature'),
            ]
        ),
        
        # IMU数据预处理节点
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='imu_filter',
            parameters=[{
                'use_mag': True,
                'do_bias_estimation': True,
                'do_adaptive_gain': True,
                'gain_acc': 0.01,
                'gain_mag': 0.01,
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data_filtered'),
                ('/imu/mag', '/imu/mag'),
            ]
        ),
        
        # IMU 静态变换发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '0', '1',  # qx, qy, qz, qw
                'base_link',
                'imu_link'
            ]
        ),
        
        # IMU 数据监控节点
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='imu_diagnostics',
            parameters=[{
                'analyzers': {
                    'imu': {
                        'type': 'diagnostic_aggregator/GenericAnalyzer',
                        'path': 'IMU',
                        'find_and_remove_prefix': ['imu'],
                    }
                }
            }]
        ),
    ]) 