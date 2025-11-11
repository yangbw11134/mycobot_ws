#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments (compatible with camera_params.yaml)
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video4',
        description='Camera device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='1280',
        description='Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='720',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Frame rate'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera4_frame',
        description='Camera frame ID'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='file:///home/cyjung/made_in_Korea/src/oCam_ros2/config/camera_info.yaml',
        description='URL to camera info yaml file'
    )

    # oCam publisher node
    ocam_node = Node(
        package='oCam_ros2',
        executable='ocam_publisher.py',
        name='ocam_publisher',
        parameters=[
            {
                'video_device': LaunchConfiguration('video_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'camera_info_url': LaunchConfiguration('camera_info_url')
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        frame_id_arg,
        camera_info_url_arg,
        ocam_node,
    ])