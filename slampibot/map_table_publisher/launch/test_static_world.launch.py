import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
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
            'robot_description': ParameterValue(Command(['xacro ', xacro_file_path]), value_type=str)
        }]
    )

    # USB Camera Node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            {
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                                'camera_name': 'ceiling_camera', # Match camera_info.yaml
                'camera_info_url': f'package://map_table_publisher/config/camera_info.yaml',
                'io_method': 'mmap',
            }
        ],
        remappings=[
            # Publish raw image and info to global topics
            # ('image', '/rectify/image_rect'),
            ('/image_raw', '/ceiling_camera/image_raw'),
            ('/camera_info', '/ceiling_camera/camera_info'),
        ]
    )

    # Image Proc Node for rectification and monochrome conversion.
    # We run it in its own namespace to avoid topic name collisions for 'camera_info'.
    image_proc_node = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc_node',
        namespace='rectify', # A dedicated namespace
        output='screen',
        remappings=[
            # It expects 'image' and 'camera_info' in its namespace ('/rectify/image')
            # We remap them to the global topics from the camera.
            ('image', '/ceiling_camera/image_raw'), # Changed from 'image_raw'
            ('camera_info', '/ceiling_camera/camera_info'),
        ]
    )

    # Rectified Camera Info Publisher node
    rectified_camera_info_publisher_node = Node(
        package='map_table_publisher',
        executable='rectified_camera_info_publisher',
        name='rectified_camera_info_publisher',
        output='screen'
    )

    # Apriltag Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ceiling',
        output='screen',
        remappings=[
            # Subscribe to the rectified image stream
            ('image_rect', '/ceiling_camera/image_raw'),  # 원본 이미지 사용
            ('camera_info', '/ceiling_camera/camera_info'), # 원본 카메라 정보 사용
            # ('image_rect', '/rectify/image_rect'),
            # ('camera_info', '/rectify/camera_info'),   # ← 추가!
            # camera_info is now provided by the rectified_camera_info_publisher node
            ('detections', '/detections') 
        ],
        parameters=[{
            'tag_family': 'tag36h11',
            'tag_size': 0.05, # IMPORTANT: Set actual AprilTag size in meters
            'publish_tf': True,
            'camera_frame': 'ceiling_camera' # Explicitly set camera frame for apriltag_node
        }],
    )

    # Node for processing the detections
    ceiling_camera_processor_node = Node(
        package='map_table_publisher',
        executable='ceiling_camera_processor', # Changed executable
        # executable='calibrated_camera_processor',
        name='ceiling_camera_processor',
        output='screen',
        parameters=[
            os.path.join(map_table_publisher_pkg, 'config', 'ceiling_cam_params.yaml'),
            {'camera_frame': 'ceiling_camera'} # Change this from 'camera' to 'ceiling_camera'
        ]
    )

    # Debug visualizer node
    debug_visualizer_node = Node(
        package='map_table_publisher',
        executable='debug_visualizer',
        name='debug_visualizer',
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(map_table_publisher_pkg, 'config', 'test_static_world.rviz')],
        output='screen'
    )

    # TF echo node for debugging
    tf_echo_node = Node(
        package='tf2_ros',
        executable='tf2_echo',
        name='tf_echo',
        
        arguments=['map', 'ceiling_camera'], # Use default_cam as the camera frame
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        # robot_state_publisher_node,
        usb_cam_node,
        
        # image_proc_node, # Re-add standalone image_proc
        # rectified_camera_info_publisher_node, # Re-add standalone rectified_camera_info_publisher
        apriltag_node, # Re-add standalone apriltag_node
        
        ceiling_camera_processor_node,
        debug_visualizer_node,
        rviz_node,
        tf_echo_node
    ])