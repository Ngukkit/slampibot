import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set ROS_DOMAIN_ID
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '66')

    # Find the necessary packages and files
    pkg_share = FindPackageShare(package='slampibot_gazebo').find('slampibot_gazebo')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    world_file = os.path.join(pkg_share, "worlds", "spb_empty_space.world")
    urdf_xacro = os.path.join(pkg_share, 'myCar', 'BMW.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'spb_urdf.rviz')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Gazebo Execution ---
    gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # --- Simulation Drive Node (for simple testing without full ros2_control setup) ---
    # This node will subscribe to /cmd_vel and publish /odom and /joint_states
    # sim_drive_node = Node(
    #     package='slampibot_gazebo',
    #     executable='slampibot_gazebo_node',
    #     name='sim_drive_node',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         {'wheel_radius': 0.033},
    #         {'wheel_separation': 0.167}
    #     ]
    # )

    # --- Robot Description and State Publisher ---
    # Process the URDF file and create the robot_description parameter
    robot_description_content = xacro.process_file(urdf_xacro).toxml()
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}],
        output="screen"
    )

    # --- Joint State Publisher ---
    # This node is crucial for providing joint states to robot_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_param}]
    )

    # --- Spawning the Robot in Gazebo ---
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'slampibot']
    )

    # Delay spawning until Gazebo server is up
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity_node])

    # --- Other Nodes ---
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    rqt_robot_steering_cmd = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
    )

    # --- Launch Description ---
    return LaunchDescription([
        set_domain,
        gazebo_server_cmd, # Launch Gazebo server
        gazebo_client_cmd, # Launch Gazebo client
        # sim_drive_node,    # Launch the simple simulation drive node (now handled by Gazebo plugin)
        robot_state_publisher_node,
        joint_state_publisher_node, # Re-added joint_state_publisher
        delayed_spawn_entity, # Spawn robot into Gazebo
        rviz_cmd,
        rqt_robot_steering_cmd,
    ])