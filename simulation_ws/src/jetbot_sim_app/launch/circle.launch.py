#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 Jetbot Description Launch File.

This script launches robot state publisher and joint state publisher node for 
jetbot and spawns it.

Revision History:

        2021-07-09 (Animesh): Baseline Software.

Example:
        $ ros2 launch jetbot_description gazebo.launch.py

"""


#___Import Modules:
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.conditions import UnlessCondition

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    jetbot_description_DIR = get_package_share_directory('jetbot_description')
    aws_robomaker_racetrack_world_DIR = get_package_share_directory('aws_robomaker_racetrack_world')
    
    jetbot_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetbot_description_DIR,'launch','gazebo.launch.py')),
    )
    
    aws_robomaker_racetrack_world_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aws_robomaker_racetrack_world_DIR,'launch','racetrack.launch.py')),
    )
    
    controller_manager_node = Node(
        package = 'controller_manager',
        executable = 'spawner.py',
        arguments = [
            os.path.join(jetbot_description_DIR,'config','jetbot_diff_drive_control.yaml'),
            ]
        )
    
    jetbot_sim_circle_node = Node(
        package = 'jetbot_sim_app',
        executable = 'circle',
        )
    
    return LaunchDescription([
        jetbot_description_launch_include,
        #aws_robomaker_racetrack_world_launch_include,
        controller_manager_node,
        #jetbot_sim_circle_node,
    ])


#                                                                              
# end of file
"""ANI717"""