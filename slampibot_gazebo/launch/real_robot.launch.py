import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # 1. Find packages and file paths
    pkg_share = get_package_share_directory('slampibot_gazebo')
    urdf_xacro_path = os.path.join(pkg_share, 'myCar', 'real_BMW.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'spb_urdf.rviz')
    tb3_param_dir = os.path.join(pkg_share, 'parameter', 'slampibot.yaml')

    # 2. Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    # 3. Process the URDF file
    robot_description_content = xacro.process_file(urdf_xacro_path).toxml()
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 4. robot_state_publisher: Publishes TF transforms for the robot model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # 5. TurtleBot3 Node: Bridges communication between Raspberry Pi and OpenCR
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=[tb3_param_dir],
        arguments=['-i', usb_port],
        output='screen')

    # 6. Joint State Publisher: Maps 2-wheel joint states from OpenCR to 4-wheel URDF
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'source_list': ['/joint_states']},
            {'joint_map': {
                'wheel_left_joint': ['LF_wheel_joint', 'LB_wheel_joint'],
                'wheel_right_joint': ['RF_wheel_joint', 'RB_wheel_joint']
            }}
        ]
    )

    # 7. Lidar and Camera Drivers (uncomment when needed)
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
            '/rplidar.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 8. RViz for visualization (Commented out for Raspberry Pi)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    # 9. Keyboard Teleop
    teleop_keyboard_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('usb_port', default_value='/dev/ttyACM0', description='Connected USB port with OpenCR'),
        
        robot_state_publisher_node,
        turtlebot3_node,
        joint_state_publisher_node,
        # lidar_driver_launch, # Uncomment when you have the lidar driver
        # rviz_node,
        teleop_keyboard_node,
    ])