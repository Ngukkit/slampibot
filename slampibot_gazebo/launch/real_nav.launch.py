
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    # Parameter files
    nav2_params_file = os.path.join(pkg_share, 'parameter', 'spb_nav2_params.yaml')
    tb3_param_dir = os.path.join(pkg_share, 'parameter', 'slampibot.yaml')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

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

    # 2. TurtleBot3 Node: Bridges communication between Raspberry Pi and OpenCR
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=[tb3_param_dir],
        arguments=['-i', usb_port],
        output='screen')

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

    # 5. Nav2 Stack
    # This includes core Nav2 nodes like amcl, controller_server, planner_server, etc.
    nav2_bringup_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file, # Activated
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # 6. SLAM (Mapping) or Map Server (Loading a pre-built map)
    # Choose one based on your needs:
    # a) SLAM Toolbox for real-time mapping:
    # slam_toolbox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(FindPackageShare('slam_toolbox').find('slam_toolbox'), 'launch', 'online_sync_launch.py')),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )
    # b) Map Server for loading a static map:
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_share, 'maps', 'my_map.yaml')}, 
                    {'use_sim_time': use_sim_time}]
    )

    # 7. Custom Waypoint Commander Node
    waypoint_commander_node = Node(
        package='robot_commander',
        executable='waypoint_commander_node',
        name='waypoint_commander',
        output='screen',
        parameters=[{'waypoints_file': os.path.join(pkg_share, 'parameter', 'waypoints.yaml')}]
    )

    # 8. RViz for visualization (Commented out for Raspberry Pi)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_path],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        turtlebot3_node,
        joint_state_publisher_node, # Added joint_state_publisher
        lidar_driver_launch,
        nav2_launch,
        # slam_toolbox_launch, # Uncomment if using SLAM
        map_server_node,     # Uncomment if loading a static map
        waypoint_commander_node, # Added waypoint commander
        # rviz_node,
    ])
