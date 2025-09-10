import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/ceiling_camera/image_raw',
        description='The image topic from the ceiling camera.'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/ceiling_camera/camera_info',
        description='The camera info topic from the ceiling camera.'
    )

    # Path to the processor parameter file
    params_file_path = os.path.join(
        get_package_share_directory('map_table_publisher'),
        'config',
        'ceiling_cam_params.yaml'
    )

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video1',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_name': 'ceiling_camera',
            'camera_info_url': 'package://map_table_publisher/config/camera_info.yaml',
            'io_method': 'mmap',
        }],
        remappings=[
            ('/image_raw', '/ceiling_camera/image_raw'),
            ('/camera_info', '/ceiling_camera/camera_info'),
        ],
    )

    # Image Proc Node via ExecuteProcess (ros2 run)
    image_proc_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'image_proc', 'image_proc', '--ros-args',
            '--remap', ['image:=', LaunchConfiguration('camera_topic')],
            '--remap', ['camera_info:=', LaunchConfiguration('camera_info_topic')],
            '--remap', 'image_rect:=/ceiling_camera/image_rect',
            '--remap', 'camera_info_rect:=/ceiling_camera/camera_info_rect',
            '--remap', '__node:=image_proc_ceiling'
        ],
        output='screen'
    )

    # Apriltag Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ceiling_camera',
        output='screen',
        parameters=[
            {'image_transport': 'raw'},
            {'use_camera_info': True}
        ],
        remappings=[
            ('image_rect', '/ceiling_camera/image_rect'),
            ('camera_info', '/ceiling_camera/camera_info_rect'),
            ('detections', '/detections')
        ]
    )

    # Ceiling Camera Processor Node
    ceiling_camera_processor_node = Node(
        package='map_table_publisher',
        executable='ceiling_camera_processor',
        name='ceiling_camera_processor',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        camera_topic_arg,
        camera_info_topic_arg,
        usb_cam_node,
        image_proc_process,
        apriltag_node,
        ceiling_camera_processor_node
    ])

