import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_table_publisher_pkg = get_package_share_directory('map_table_publisher')
    
    # All parameters are now loaded from this single YAML file
    param_file = os.path.join(map_table_publisher_pkg, 'config', 'landmark_tags.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

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

        Node(
            package='map_table_publisher',
            executable='image_preprocessor_node',
            name='image_preprocessor_new',
            output='screen'
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_ceiling_camera',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('image_rect', '/ceiling_camera/image_raw'),
                ('camera_info', '/ceiling_camera/camera_info'),
                ('detections', '/tag_detections')
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_frame_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'default_cam', '--child-frame-id', 'ceiling_camera_link'],
            output='screen'
        ),

        Node(
            package='map_table_publisher',
            executable='calibrated_camera_processor',
            name='calibrated_camera_processor',
            output='screen',
            parameters=[param_file],
            remappings=[
                ('/tag_detections', '/tag_detections')
            ]
        ),

        Node(
            package='map_table_publisher',
            executable='table_obstacle_merger',
            name='table_obstacle_merger',
            output='screen'
        ),

        Node(
            package='map_table_publisher',
            executable='polygon_to_pointcloud_node',
            name='polygon_to_pointcloud_node',
            output='screen'
        ),
        
        Node(
            package='map_table_publisher',
            executable='landmark_visualizer',
            name='landmark_visualizer',
            output='screen',
            parameters=[param_file]
        ),

        Node(
            package='map_table_publisher',
            executable='debug_visualizer',
            name='debug_visualizer',
            output='screen',
            arguments=['--ros-args', '--remap', 'detections:=/persistent_tag_detections']
        ),
        
        Node(
            package='map_table_publisher',
            executable='camera_monitor_node',
            name='camera_monitor',
            output='screen'
        ),
    ])