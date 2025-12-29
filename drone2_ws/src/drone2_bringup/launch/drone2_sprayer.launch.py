#!/usr/bin/env python3
# Copyright 2024 Maintainer
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Sprayer System Launch File

Launches all nodes for the sprayer drone:
- GCS to D2 Downlink
- Navigation
- Local Detection
- Centering Controller
- Sprayer Control
- Mission Manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Drone-2."""
    
    log_level = LaunchConfiguration('log_level')
    
    return LaunchDescription([
        DeclareLaunchArgument('log_level', default_value='info'),
        
        # GCS Downlink
        Node(
            package='gcs_to_d2_downlink',
            executable='gcs_to_d2_downlink_node',
            name='gcs_to_d2_downlink_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('gcs_to_d2_downlink'), 'config', 'downlink_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Navigation
        Node(
            package='drone2_navigation',
            executable='drone2_navigation_node',
            name='drone2_navigation_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('drone2_navigation'), 'config', 'navigation_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Local Detection
        Node(
            package='local_detection',
            executable='local_detection_node',
            name='local_detection_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('local_detection'), 'config', 'detection_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Centering Controller
        Node(
            package='centering_controller',
            executable='centering_controller_node',
            name='centering_controller_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('centering_controller'), 'config', 'centering_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Sprayer Control
        Node(
            package='sprayer_control',
            executable='sprayer_control_node',
            name='sprayer_control_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('sprayer_control'), 'config', 'sprayer_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Mission Manager
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('mission_manager'), 'config', 'manager_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])
