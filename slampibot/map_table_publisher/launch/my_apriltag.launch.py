import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family to detect'
    )

    # Load the YAML file
    params_file_path = os.path.join(
        get_package_share_directory('map_table_publisher'),
        'config',
        'table_params.yaml'
    )

    # Node with parameters
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[
            params_file_path, # Load all parameters from the YAML
            {'image_transport': 'raw'} # Override image_transport
        ],
        remappings=[
            ('image_rect', '/camera1/image_raw')
        ]
    )

    return LaunchDescription([
        tag_family_arg,
        apriltag_node
    ])