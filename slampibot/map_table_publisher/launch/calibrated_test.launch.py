import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def generate_launch_description():
    # Paths
    slampibot_gazebo_pkg = get_package_share_directory('slampibot_gazebo')
    map_table_publisher_pkg = get_package_share_directory('map_table_publisher')

    # Define sensor_data QoS profile
    sensor_data_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_file_path = os.path.join(slampibot_gazebo_pkg, 'myCar', 'BMW.urdf.xacro')

    # Nodes and Launch Includes
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
                'framerate': 30.0, # Reduced from 30.0
                'pixel_format': 'yuyv',
                'camera_name': 'ceiling_camera',
                'camera_info_url': 'package://map_table_publisher/config/camera_info.yaml',
                'io_method': 'mmap',
                'camera_frame_id': 'default_cam',
                # 'brightness': 50,   # Added for testing static scene detection
                # 'contrast': 50,     # Added for testing static scene detection
                # 'exposure': 100     # Added for testing static scene detection
            }
        ],
        remappings=[
            ('image_raw', '/ceiling_camera/image_raw'),
            ('camera_info', '/ceiling_camera/camera_info'),
        ]
    )

    # Image Noise Adder Node
    image_noise_adder_node = Node(
        package='map_table_publisher', # Assuming it's in this package
        executable='image_noise_adder',
        name='image_noise_adder',
        output='screen'
    )

    # AprilTag detection node (using raw image directly)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_ceiling_camera',
        output='screen',
        parameters=[
            {'image_transport': 'raw'},
            {'use_camera_info': True},
            {'tag_size': 0.05},
            {'compute_pose': True}, # Enable pose computation
            # {'tag_refine_edges': True}, 
            # {'tag_refine_decode': True}
        ],
        remappings=[
            # ('image_rect', '/ceiling_camera/image_noisy_raw'), # <--- REMAP TO NOISY IMAGE
            ('image_rect', '/ceiling_camera/image_raw'),
            ('camera_info', '/ceiling_camera/camera_info'),
            ('detections', '/tag_detections') # Remap apriltag_node output to /tag_detections
        ]
    )

    # Calibrated camera processor node
    calibrated_camera_processor_node = Node(
        package='map_table_publisher',
        executable='calibrated_camera_processor',
        name='calibrated_camera_processor',
        output='screen',
        parameters=[os.path.join(map_table_publisher_pkg, 'config', 'ceiling_cam_params.yaml')]
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
        # image_noise_adder_node,
        apriltag_node,
        calibrated_camera_processor_node,
        debug_visualizer_node,
        rviz_node,
        tf_echo_node
    ]) 