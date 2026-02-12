#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    add_realsense_d405 = LaunchConfiguration('add_realsense_d405', default=False)
    add_d405_links = LaunchConfiguration('add_d405_links', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=True)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    # robot moveit gazebo launch
    # xarm_moveit_config/launch/_robot_moveit_gazebo.launch.py
    robot_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_gazebo.launch.py'])),
        launch_arguments={
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
            'add_realsense_d405': add_realsense_d405, 
            'add_d405_links': add_d405_links,
            'add_realsense_d435i': add_realsense_d435i, 
            'add_d435i_links': add_d435i_links
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_gazebo_launch
    ])
