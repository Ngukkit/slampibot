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

def generate_launch_description():

    set_domain = SetEnvironmentVariable('ROS_DOMAIN_ID', '30')
    pkg_share = FindPackageShare(package='slampibot_gazebo').find('slampibot_gazebo')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    urdf_xacro = os.path.join(pkg_share, 'myCar', 'BMW.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'spb_urdf_map.rviz')    
    world_file = os.path.join(pkg_share, "worlds", "spb_building_slampibot.world")
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
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
    
    dla_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share, 'params', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

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
                     'robot_description': Command(['xacro',' ', urdf_xacro])}])
    
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])
    
    rqt_robot_steering_cmd = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen')
    
    async_slam_toolbox_cmd = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    ld = LaunchDescription()
    ld.add_action(set_domain)
    ld.add_action(dla_use_sim_time)
    ld.add_action(dla_world)
    ld.add_action(dla_slam_params_file)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)    
    ld.add_action(rviz_cmd)
    ld.add_action(rqt_robot_steering_cmd)
    ld.add_action(async_slam_toolbox_cmd)

    return ld