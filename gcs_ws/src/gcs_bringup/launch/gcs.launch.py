#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Ground Control Station Launch File

Launches all GCS nodes:
- Target Receiver
- Mission Router
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for GCS."""
    
    # Launch arguments
    log_level = LaunchConfiguration('log_level')
    auto_dispatch = LaunchConfiguration('auto_dispatch')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        DeclareLaunchArgument(
            'auto_dispatch',
            default_value='true',
            description='Automatically dispatch missions to Drone-2'
        ),
        
        # Target Receiver
        Node(
            package='gcs_target_receiver',
            executable='gcs_target_receiver_node',
            name='gcs_target_receiver_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('gcs_target_receiver'),
                    'config', 'receiver_params.yaml'
                ])
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Mission Router
        Node(
            package='gcs_mission_router',
            executable='gcs_mission_router_node',
            name='gcs_mission_router_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('gcs_mission_router'),
                    'config', 'router_params.yaml'
                ]),
                {'auto_dispatch': auto_dispatch}
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])
