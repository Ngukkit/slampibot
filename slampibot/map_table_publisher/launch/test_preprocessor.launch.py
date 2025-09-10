from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_table_publisher',
            executable='image_preprocessor_node',
            name='image_preprocessor_test',
            output='screen'
        )
    ])
