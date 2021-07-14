#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""ROS2 AWS Robomaker Racetrack World Launch File.

This script spawns Jetbot. 

Revision History:

        2021-07-09 (Animesh): Baseline Software.

Example:
        $ ros2 launch jetbot_description spawn_launch.py

"""


#___Import Modules:
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PythonExpression, TextSubstitution

from launch_ros.actions import Node


#___Function:
def generate_launch_description():
	
    # Get the package directory
    racetrack_dir = get_package_share_directory('aws_robomaker_racetrack_world')
    jetbot_description_dir = get_package_share_directory('jetbot_description')
    
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    z_pos = LaunchConfiguration('z_pos')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(racetrack_dir, 'worlds', 'racetrack_day.world'),
        description='Full path to world model file to load')
    
    declare_x_pos_cmd = DeclareLaunchArgument(
        'x_pos',
        default_value=TextSubstitution(text="2.75"))
    
    declare_y_pos_cmd = DeclareLaunchArgument(
        'y_pos',
        default_value=TextSubstitution(text="-14.0"))
    
    declare_z_pos_cmd = DeclareLaunchArgument(
        'z_pos',
        default_value=TextSubstitution(text="0.5"))
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start the joint state publisher')
    
    urdf = os.path.join(jetbot_description_dir, 'models', 'urdf', 'jetbot.urdf')
    urdf_xacro = Command(['xacro ', os.path.join(jetbot_description_dir, 'models', 'urdf', 'jetbot.urdf')])
    
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[racetrack_dir],
        output='screen')
    
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[racetrack_dir],
        output='screen')
    
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
    
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0,
            'robot_description': urdf_xacro}],
        output='screen')
    
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': urdf_xacro}],
        output='screen')

        
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)   
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_pos_cmd)
    ld.add_action(declare_y_pos_cmd)
    ld.add_action(declare_z_pos_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_joint_state_pub_cmd)
    
    # Add all actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)    
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
        
    return ld


#                                                                              
# end of file
"""ANI717"""