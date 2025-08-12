from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the full path to the waypoints.yaml file
    config_file_path = os.path.join(
        get_package_share_directory('robot_commander'),
        'config',
        'waypoints.yaml'
    )

    # Declare launch arguments
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=config_file_path
    )

    # Node for the WaypointCommander
    waypoint_commander_node = Node(
        package='robot_commander',
        executable='waypoint_commander_node',
        name='waypoint_commander',
        output='screen',
        parameters=[
            {'waypoints_file': LaunchConfiguration('waypoints_file')}
        ]
    )

    return LaunchDescription([
        waypoints_file_arg,
        waypoint_commander_node
    ])