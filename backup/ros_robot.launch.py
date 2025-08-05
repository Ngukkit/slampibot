
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Find packages and file paths
    pkg_share = FindPackageShare(package='slampibot_gazebo').find('slampibot_gazebo')
    
    # Use the real robot URDF
    urdf_xacro_path = os.path.join(pkg_share, 'myCar', 'real_BMW.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'spb_urdf.rviz')

    # Process the URDF file
    robot_description_content = xacro.process_file(urdf_xacro_path).toxml()
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # Real robot, so use_sim_time is false

    # 1. Robot State Publisher: Publishes TF transforms for the robot model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # 2. rosserial_python node: Bridges communication between Raspberry Pi and OpenCR
    rosserial_node = Node(
        package='rosserial_python',
        executable='rosserial_node.py',
        name='rosserial_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyACM0'}, # Adjust port if necessary
            {'baud': 1000000} # Updated baud rate to 1Mbps
        ]
    )

    # 3. Joint State Publisher: Maps 2-wheel joint states from OpenCR to 4-wheel URDF
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_param},
            {'source_list': ['/joint_states']},
            {'joint_map': {
                'wheel_left_joint': ['LF_wheel_joint', 'LB_wheel_joint'],
                'wheel_right_joint': ['RF_wheel_joint', 'RB_wheel_joint']
            }} # Map firmware joints to URDF joints
        ]
    )

    # 4. Lidar Driver (Example: RPLidar ROS2 driver)
    # Replace with your actual lidar driver launch file
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
            '/rplidar.launch.py' # Or your specific lidar launch file
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        rosserial_node,
        joint_state_publisher_node, # Added joint_state_publisher
        lidar_driver_launch,
        rviz_node,
    ])
