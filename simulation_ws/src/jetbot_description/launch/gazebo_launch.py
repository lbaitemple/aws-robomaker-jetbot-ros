#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script spawns Jetbot. 

Revision History:

        2021-07-09 (Animesh): Baseline Software.

Example:
        $ colcon build && source install/setup.bash && ros2 launch jetbot_description gazebo_launch.py
        $ source install/setup.bash && ros2 launch jetbot_description gazebo_launch.py
        $ ros2 launch jetbot_description gazebo_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


#___Function:
def generate_launch_description():
	
    # Get the package directory
    racetrack_dir = get_package_share_directory('aws_robomaker_racetrack_world')
    jetbot_description_dir = get_package_share_directory('jetbot_description')
    
    
    # Specify the actions
    world_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(racetrack_dir,
                'launch','racetrack_launch.py')))
    
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetbot_description_dir,
                'launch','spawn_launch.py')))
    
    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jetbot_description_dir,
                'launch','jetbot_description_launch.py')))

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add all actions
    ld.add_action(world_launch_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(robot_description_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""