#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script spawns Jetbot. 

Revision History:

        2021-07-09 (Animesh): Baseline Software.

Example:
        $ colcon build && source install/setup.bash && ros2 launch jetbot_description jetbot_description_launch.py
        $ source install/setup.bash && ros2 launch jetbot_description jetbot_description_launch.py
        $ ros2 launch jetbot_description jetbot_description_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
	
    # Get the package directory
    package_dir = get_package_share_directory('jetbot_description')
    urdf_file_name = 'jetbot.xml'
    
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    urdf = os.path.join(package_dir,'models','urdf',urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    
    
    # Specify the actions
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
            'robot_description': robot_description}],
        arguments=[urdf])
    
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description}],
        output='screen')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add all actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
