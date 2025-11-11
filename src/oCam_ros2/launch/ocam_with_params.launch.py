#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/cyjung/made_in_Korea/src/oCam_ros2/config/camera_params.yaml',
        description='Full path to the ROS2 parameters file to use'
    )

    # oCam publisher node with parameters file
    ocam_node = Node(
        package='oCam_ros2',
        executable='ocam_publisher.py',
        name='ocam_publisher',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )

    return LaunchDescription([
        params_file_arg,
        ocam_node,
    ])