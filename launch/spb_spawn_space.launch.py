import os
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Set ROS_DOMAIN_ID
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '66')

    # Find the necessary packages and files
    pkg_share = FindPackageShare(package='slampibot_gazebo').find('slampibot_gazebo')
    world_file = os.path.join(pkg_share, "worlds", "spb_empty_space.world")
    urdf_xacro = os.path.join(pkg_share, 'myCar', 'BMW.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'spb_urdf.rviz')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --- Gazebo Execution ---
    # Execute Gazebo as a separate process to completely isolate it from ROS parameters.
    # This prevents the 'robot_description' from leaking into the Gazebo process.
    gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
    )

    gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # Process the URDF file and write it to a temporary file
    robot_description_content = xacro.process_file(urdf_xacro).toxml()
    temp_urdf_file = '/tmp/bmw.urdf'
    with open(temp_urdf_file, 'w') as f:
        f.write(robot_description_content)

    # --- Gazebo Execution ---
    # Execute Gazebo as a separate process to completely isolate it from ROS parameters.
    gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
    )

    gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # --- Robot Description and State Publisher ---
    # robot_state_publisher still publishes the topic for other nodes like RViz
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}],
        output="screen"
    )

    # --- Spawning the Robot ---
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'slampibot'],
        output='screen'
    )

    # Delay spawning until Gazebo server is up
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity_node])

    # --- Controllers ---
    # The controller spawner nodes will automatically find the controller_manager
    # service provided by the gazebo_ros2_control plugin.
    controller_spawner_group = GroupAction(actions=[
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['LF_wheel_vel', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['RF_wheel_vel', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['LB_wheel_vel', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['RB_wheel_vel', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ])

    # Delay controller spawning until the robot is spawned
    delayed_controller_spawners = TimerAction(period=8.0, actions=[controller_spawner_group])

    # --- Other Nodes ---
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

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
        gazebo_server_cmd,
        gazebo_client_cmd,
        robot_state_publisher_node,
        delayed_spawn_entity,
        joint_state_publisher_cmd,
        delayed_controller_spawners,
        rviz_cmd,
        rqt_robot_steering_cmd,
    ])