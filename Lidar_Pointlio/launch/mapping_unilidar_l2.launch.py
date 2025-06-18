from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the RViz argument
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to launch RViz.')

    # #节点参数，包括来自YAML配置文件的参数
    laser_mapping_params = [
        PathJoinSubstitution([
            FindPackageShare('point_lio'), # 在这里包名叫point_lio
            'config', 'unilidar_l2.yaml'
        ]),
        {
            'use_imu_as_input':  False,  # Change to True to use IMU as input of Point-LIO
            # 是否使用IMU作为系统输入，当imu偏差大时，应该设置为false
            'prop_at_freq_of_imu': True,# 是否在IMU频率下进行状态传播，与上面一致
            'check_satu': True, # 是否检查IMU数据饱和(防止IMU数据超出量程)
            'init_map_size': 10, # 初始地图大小(单位：米)，原10
            'point_filter_num': 1, # Options: 1, 3 # 点云滤波参数，可选1或3
            'space_down_sample': True, # 是否进行空间降采样
            'filter_size_surf': 0.1,  # Options: 0.5, 0.3, 0.2, 0.15, 0.1 # 表面特征滤波尺寸(米)，可选0.5,0.3,0.2,0.15,0.1
            'filter_size_map': 0.1,  # Options: 0.5, 0.3, 0.15, 0.1 # 地图滤波尺寸(米)，可选0.5,0.3,0.15,0.1
            'cube_side_length': 1000.0,  # Option: 1000 # 地图立方体边长(米)，可选1000
            'runtime_pos_log_enable': True,  # Option: True # 是否启用运行时位置日志记录
        }
    ]

    # Node definition for laserMapping with Point-LIO
    laser_mapping_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
        # prefix='gdb -ex run --args'
    )

    # Conditional RViz node launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('point_lio'),
            'rviz_cfg', 'loam_livox.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        prefix='nice'
    )

    # Assemble the launch description
    ld = LaunchDescription([
        rviz_arg,
        laser_mapping_node,
        GroupAction(
            actions=[rviz_node],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])

    return ld
