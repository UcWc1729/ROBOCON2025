#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    博升 P100R RGB-D 相机驱动启动文件
    支持RGB图像和深度图像同时输出
    """
    
    # 启动参数
    camera_name = LaunchConfiguration('camera_name')
    device_id = LaunchConfiguration('device_id')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'camera_name',
            default_value='p100r',
            description='相机名称空间'
        ),
        
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='USB设备ID'
        ),
        
        DeclareLaunchArgument(
            'width',
            default_value='1920',
            description='图像宽度'
        ),
        
        DeclareLaunchArgument(
            'height',
            default_value='1080',
            description='图像高度'
        ),
        
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='帧率设置'
        ),
        
        # P100R RGB-D 相机节点
        Node(
            package='usb_cam',  # 使用标准USB相机驱动包
            executable='usb_cam_node_exe',
            name='p100r_rgb_node',
            namespace=camera_name,
            parameters=[{
                'video_device': ['/dev/video', device_id],
                'framerate': fps,
                'image_width': width,
                'image_height': height,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
            }],
            remappings=[
                ('/p100r/image_raw', '/p100r/color/image_raw'),
                ('/p100r/camera_info', '/p100r/color/camera_info'),
            ]
        ),
        
        # P100R 深度相机节点（通过UVC深度接口）
        Node(
            package='realsense2_camera',  # 可替换为P100R专用驱动
            executable='realsense2_camera_node',
            name='p100r_depth_node',
            namespace=camera_name,
            parameters=[{
                'enable_color': False,
                'enable_depth': True,
                'depth_width': width,
                'depth_height': height,
                'depth_fps': fps,
                'camera_frame_id': 'camera_link',
                'depth_format': '16UC1',
            }],
            remappings=[
                ('/p100r/depth/image_rect_raw', '/p100r/depth/image_raw'),
                ('/p100r/depth/camera_info', '/p100r/depth/camera_info'),
            ]
        ),
        
        # 相机信息发布节点（内参标定结果）
        Node(
            package='camera_info_manager',
            executable='camera_info_manager_node',
            name='camera_info_manager',
            namespace=camera_name,
            parameters=[{
                'camera_name': 'p100r',
                'camera_info_url': 'file://$(find p100r_camera_driver)/config/p100r_camera_info.yaml',
            }]
        ),
    ]) 