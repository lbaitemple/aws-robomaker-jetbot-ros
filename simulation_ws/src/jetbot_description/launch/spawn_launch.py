#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script spawns Jetbot. 

Revision History:

        2021-13-09 (Animesh): Baseline Software.

Example:
        $ ros2 launch jetbot_description spawn_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression, TextSubstitution

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
	
    # Get the package directory
    package_dir = get_package_share_directory('jetbot_description')
    
    
    # Create launch configuration variables
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    
    
    # Declare the launch arguments
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value=TextSubstitution(text="0.0"))
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value=TextSubstitution(text="0.0"))
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value=TextSubstitution(text="0.5"))
    
    urdf = os.path.join(package_dir, 'models', 'urdf', 'jetbot.urdf')
    
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        cwd=[package_dir],
        output='screen')
    
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[package_dir],
        output='screen')
    
    spawn_robot_cmd = Node(
        package='gazebo_ros', 
        node_executable='spawn_entity.py',
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
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_x_pos_cmd)
    ld.add_action(declare_y_pos_cmd)
    ld.add_action(declare_z_pos_cmd)
    
    # Add all actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_robot_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""