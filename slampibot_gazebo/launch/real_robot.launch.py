
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 1. Find packages and file paths
    pkg_share = get_package_share_directory('slampibot_gazebo')
    # Use the new, clean URDF for the real robot
    urdf_xacro_path = os.path.join(pkg_share, 'myCar', 'real_BMW.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'spb_urdf.rviz')

    # 2. Process the URDF file
    robot_description_content = xacro.process_file(urdf_xacro_path).toxml()

    # 3. robot_state_publisher: Publishes TF transforms for the robot model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 4. [THE MOST IMPORTANT PART] The real hardware driver node
    # This is the node that communicates with the OpenCR board.
    # You need to build this C++ node (real_driver_node.cpp).
    real_driver_node = Node(
        package='slampibot_gazebo',
        executable='real_driver_node', # The name of the compiled C++ executable
        name='real_driver',
        output='screen',
    )

    # 5. Real sensor driver nodes (Lidar, Camera)
    # These must be replaced with the actual launch files for your specific hardware.
    # Example for RPLidar:
    # rplidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
    #         '/rplidar.launch.py'
    #     ])
    # )
    # Example for a USB Camera:
    # camera_launch = Node(
    #     package='v4l2_camera',
    #     executable='v4l2_camera_node',
    #     name='camera',
    #     parameters=[{'video_device': '/dev/video0'}]
    # )

    # 6. RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        real_driver_node,
        # rplidar_launch, # Uncomment when you have the lidar driver
        # camera_launch,  # Uncomment when you have the camera driver
        rviz_node,
    ])
