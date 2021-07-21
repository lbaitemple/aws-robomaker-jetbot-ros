#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script spawns Jetbot. 

Revision History:

        2021-13-09 (Animesh): Baseline Software.

Example:
        $ colcon build && source install/setup.bash && ros2 launch jetbot_description spawn_launch.py
        $ source install/setup.bash && ros2 launch jetbot_description spawn_launch.py
        $ ros2 launch jetbot_description spawn_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
	
    # Get the package directory
    package_dir = get_package_share_directory('jetbot_description')
    urdf_file_name = 'jetbot_xacro_urdf.xml'
    
    
    # Create launch configuration variables
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    
    
    # Declare the launch arguments    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value=TextSubstitution(text="2.75"))
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value=TextSubstitution(text="-14"))
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value=TextSubstitution(text="0.5"))
    
    urdf = os.path.join(package_dir,'models','urdf',urdf_file_name)
    
    
    # Specify the actions
    spawn_robot_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity',
            'robot',
            '-file',
            urdf,
            '-x',
            x_pos,
            '-y',
            y_pos,
            '-z',
            z_pos],
        output='screen')

    
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_x_pos_cmd)
    ld.add_action(declare_y_pos_cmd)
    ld.add_action(declare_z_pos_cmd)
    
    # Add all actions
    ld.add_action(spawn_robot_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""