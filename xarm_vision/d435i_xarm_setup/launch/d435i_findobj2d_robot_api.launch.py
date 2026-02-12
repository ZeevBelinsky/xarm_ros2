#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('d435i_xarm_setup'),
                'launch',
                '_findobj2d_robot_api.launch.py'
            ])
        ),
        launch_arguments={
            'camera_model': 'd435i',  # <-- key line: this file = D435i
        }.items(),
    )

    return LaunchDescription([base_launch])
