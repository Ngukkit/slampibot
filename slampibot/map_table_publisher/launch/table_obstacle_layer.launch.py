import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    map_table_publisher_pkg = get_package_share_directory('map_table_publisher')
    
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(map_table_publisher_pkg, 'config', 'table_obstacle_layer.yaml'),
        description='Path to the YAML file with table obstacle layer parameter values'
    )
    
    # Table obstacle layer node
    table_obstacle_layer_node = Node(
        package='table_obstacle_layer',
        executable='table_obstacle_layer_node',
        name='table_obstacle_layer',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    return LaunchDescription([
        params_file_arg,
        table_obstacle_layer_node
    ])