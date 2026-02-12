#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Gazebo + YOLOv12 3D demo:
# - assumes the xArm/Lite6 and D405/D435i are already running in Gazebo
#   (e.g. via xarm_moveit_config/lite6_moveit_gazebo.launch.py with add_realsense_d405:=true)
# - starts:
#       * yolo_ros (Ultralytics YOLOv8–YOLOv12 wrapper) with 3D detection enabled
#       * tf_object_to_base helper node (you can adapt it to consume YOLO detections)
#
# This is analogous in spirit to the ORK LINEMOD demo, but using YOLOv12 via yolo_ros.
#
# Copyright (c) 2025, UFACTORY, Inc.
# All rights reserved.
#
# Author: adapted for YOLOv12

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # YOLO model to use (YOLOv12; you can override from CLI if you want)
    # e.g. model:=/absolute/path/to/your/yolo12m-seg.pt
    model = LaunchConfiguration('model', default='yolo12m-seg.pt')

    # Frame in which 3D detections will be expressed (usually robot base)
    target_frame = LaunchConfiguration('target_frame', default='link_base')

    # D405/D435i topics (Gazebo or real camera); override if your topics differ
    input_image_topic = LaunchConfiguration(
        'input_image_topic',
        default='/camera/camera/color/image_raw'
    )
    input_depth_topic = LaunchConfiguration(
        'input_depth_topic',
        default='/camera/camera/aligned_depth_to_color/image_raw'
    )
    input_depth_info_topic = LaunchConfiguration(
        'input_depth_info_topic',
        default='/camera/camera/aligned_depth_to_color/camera_info'
    )

    # yolo_ros namespace and options
    namespace = LaunchConfiguration('namespace', default='yolo')
    use_tracking = LaunchConfiguration('use_tracking', default='True')
    use_3d = LaunchConfiguration('use_3d', default='True')
    use_debug = LaunchConfiguration('use_debug', default='True')

    # Optional use_sim_time for your own nodes (tf_object_to_base, etc.)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include the generic yolo_ros launch (supports YOLOv3–YOLOv12)
    # ros2 launch yolo_bringup yolo.launch.py model:=yolo12m-seg.pt use_3d:=True ...
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py',
            ])
        ),
        launch_arguments={
            'model': model,
            'target_frame': target_frame,
            'input_image_topic': input_image_topic,
            'input_depth_topic': input_depth_topic,
            'input_depth_info_topic': input_depth_info_topic,
            'namespace': namespace,
            'use_tracking': use_tracking,
            'use_3d': use_3d,
            'use_debug': use_debug,
            # model_type, device, etc. keep their defaults from yolo.launch.py
        }.items(),
    )

    # Helper node: publish an object TF based on detections
    # NOTE: the existing tf_object_to_base in d435i_xarm_setup is wired for ORK.
    # If you want to use YOLO detections, you'll likely adapt its code (or make
    # a new node) to subscribe to /yolo/detections_3d instead.
    tf_object_to_base = Node(
        package='d435i_xarm_setup',
        executable='yolo_tf_object_to_base',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return [
        yolo_launch,
        tf_object_to_base,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
