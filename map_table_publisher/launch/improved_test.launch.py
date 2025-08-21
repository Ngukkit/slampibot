import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    # Paths
    slampibot_gazebo_pkg = get_package_share_directory('slampibot_gazebo')
    map_table_publisher_pkg = get_package_share_directory('map_table_publisher')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_path = os.path.join(slampibot_gazebo_pkg, 'myCar', 'BMW.urdf.xacro')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file_path]),
                value_type=str
            )
        }]
    )

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv2rgb',
            'camera_name': 'ceiling_camera',
            'camera_info_url': f'package://map_table_publisher/config/camera_info.yaml',
            'io_method': 'mmap',
        }],
        remappings=[
            ('/image_raw', '/ceiling_camera/image_raw'),
            ('/camera_info', '/ceiling_camera/camera_info'),
        ]
    )

    # Image Proc Node (ExecuteProcess) - Now rectifies and converts to mono
    image_proc_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'image_proc', 'mono', '--ros-args', # Changed to mono nodelet
            '--remap', 'image_raw:=/ceiling_camera/image_raw',
            '--remap', 'camera_info:=/ceiling_camera/camera_info',
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
        name='apriltag_ceiling',
        output='screen',
        remappings=[
            ('image', '/ceiling_camera/image_rect'),
            ('camera_info', '/ceiling_camera/camera_info_rect'),
            ('detections', '/detections') 
        ],
        parameters=[{
            'tag_family': 'tag36h11',
            'image_processing_rate': 30.0,
            'use_camera_info': True,
            'tag_size': 0.05 # Set actual AprilTag size
        }],
    )

    # Ceiling Camera Processor Node
    ceiling_camera_processor_node = Node(
        package='map_table_publisher',
        executable='ceiling_camera_processor',
        name='ceiling_camera_processor',
        output='screen',
        parameters=[os.path.join(map_table_publisher_pkg, 'config', 'ceiling_cam_params.yaml')]
    )

    # Debug Visualizer Node
    debug_visualizer_node = Node(
        package='map_table_publisher',
        executable='debug_visualizer',
        name='debug_visualizer',
        output='screen'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(map_table_publisher_pkg, 'config', 'test_static_world.rviz')],
        output='screen'
    )

    # TF echo Node for debugging
    tf_echo_node = Node(
        package='tf2_ros',
        executable='tf2_echo',
        name='tf_echo',
        arguments=['map', 'ceiling_camera_link'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        usb_cam_node,
        image_proc_process,  # ExecuteProcess로 rectified 이미지 생성
        apriltag_node,
        ceiling_camera_processor_node,
        debug_visualizer_node,
        rviz_node,
        tf_echo_node
    ])