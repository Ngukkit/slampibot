# !/usr/bin/env/ python3
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leo Cho

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')

    pkg_share_gazebo = get_package_share_directory('slampibot_gazebo')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    urdf_xacro = os.path.join(pkg_share_gazebo, 'urdf', 'spb_urdf_gazebo_spawn.xacro')
    rviz_config = os.path.join(pkg_share_gazebo, 'rviz', 'spb_urdf_map.rviz')    
    world_file = os.path.join(pkg_share_gazebo, "worlds", "spb_empty_space.world")
    
    cartographer_config = LaunchConfiguration('cartographer_config', default=os.path.join(pkg_share_gazebo, 'params'))
    lua_config = LaunchConfiguration('lua_config', default='spb_cartographer_gazebo.lua')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    
    dla_world = DeclareLaunchArgument(
        name='world',
        default_value=world_file,
        description='Full path to the world model file to load'
    )
 
    dla_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start Gazebo server
    gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client    
    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': Command([f'xacro {urdf_xacro}'])}])

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])
    
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'slampibot']
    )

    
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
	    remappings=[('/scan', '/ldlidar_node/scan')]  # 이 부분 추가
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
    ld.add_action(dla_world)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)    
    ld.add_action(cartoprapher_cmd)
    ld.add_action(occupancy_grid_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(rqt_robot_steering_cmd)

    return ld
