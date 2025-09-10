import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='ceiling_camera',
            output='screen',
            parameters=[
                {
                    'video_device': '/dev/video0',
                    'image_width': 1280,
                    'image_height': 720,
                    'framerate': 10.0,
                    'pixel_format': 'yuyv2rgb',
                    'camera_name': 'ceiling_camera',
                    'camera_info_url': 'package://map_table_publisher/config/camera_info.yaml',
                    'io_method': 'mmap',
                    'camera_frame_id': 'default_cam',
                }
            ],
            remappings=[
                ('image_raw', '/ceiling_camera/image_raw'),
                ('camera_info', '/ceiling_camera/camera_info'),
            ]
        ),
    ])