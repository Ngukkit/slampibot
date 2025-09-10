import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'map_table_publisher'
    pkg_share_dir = get_package_share_directory(pkg_name)

    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share_dir, 'config', 'table_params.yaml'),
        description='Path to the YAML file with parameter values'
    )

    # Node with parameters from YAML
    tag_publisher_node = Node(
        package=pkg_name,
        executable='tag_publisher_node', # This is the executable name from setup.py
        name='table_obstacle_node', # This is the node name used in the YAML file
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file_arg,
        tag_publisher_node
    ])