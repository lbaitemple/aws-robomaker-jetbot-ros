#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script rotates Jetbot. 

Revision History:

        2021-07-09 (Animesh): Baseline Software.

Example:
        $ colcon build && source install/setup.bash && ros2 launch jetbot_sim_app circle_launch.py
        $ source install/setup.bash && ros2 launch jetbot_sim_app circle_launch.py
        $ ros2 launch jetbot_sim_app circle_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


#___Function:
def generate_launch_description():
    
    circle_cmd = Node(
        package = 'jetbot_app',
        node_executable = 'circle')
    
    move_cmd = Node(
        package = 'jetbot_app',
        node_executable = 'move')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(circle_cmd)
    #ld.add_action(move_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""
