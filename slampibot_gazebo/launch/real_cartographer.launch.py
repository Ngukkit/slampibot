
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')

    pkg_share = FindPackageShare(package='slampibot_gazebo').find('slampibot_gazebo')
    
    # Use the real robot URDF
    urdf_xacro = os.path.join(pkg_share, 'myCar', 'real_BMW.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'spb_urdf_map.rviz')    
    
    # Parameter files
    tb3_param_dir = os.path.join(pkg_share, 'parameter', 'slampibot.yaml')
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    # Cartographer config for real robot
    cartographer_config = LaunchConfiguration('cartographer_config', default=os.path.join(pkg_share, 'params'))
    lua_config = LaunchConfiguration('lua_config', default='spb_cartographer_real.lua') # New real robot config
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # Real robot, so use_sim_time is false
    
    dla_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot State Publisher
    robot_description_content = xacro.process_file(urdf_xacro).toxml()
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}])

    # TurtleBot3 Node
    turtlebot3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=[tb3_param_dir],
        arguments=['-i', usb_port],
        output='screen')

    # Joint State Publisher: Maps 2-wheel joint states from OpenCR to 4-wheel URDF
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

    # Lidar Driver (Example: RPLidar ROS2 driver)
    # Replace with your actual lidar driver launch file
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rplidar_ros'), 'launch'),
            '/rplidar.launch.py' # Or your specific lidar launch file
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # rviz_cmd = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config])
    
    rqt_robot_steering_cmd = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen')
    
    cartoprapher_cmd = Node(
	    package='cartographer_ros', 
	    executable='cartographer_node',
	    output='screen',
	    parameters=[{'use_sim_time': use_sim_time}],
	    arguments=[
		'-configuration_directory', cartographer_config,
		'-configuration_basename', lua_config],
	    remappings=[('/scan', '/scan')]  # Assuming lidar publishes to /scan
	)

        
    occupancy_grid_cmd = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.02', '-publish_period_sec', '1.0'])
    
    ld = LaunchDescription()
    
    ld.add_action(set_domain)
    ld.add_action(dla_use_sim_time)
    ld.add_action(robot_state_publisher_node)    
    ld.add_action(turtlebot3_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(lidar_driver_launch)
    ld.add_action(cartoprapher_cmd)
    ld.add_action(occupancy_grid_cmd)
    # ld.add_action(rviz_cmd)
    ld.add_action(rqt_robot_steering_cmd)

    return ld
