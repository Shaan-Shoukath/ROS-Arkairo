#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Drone-2 Sprayer System Launch File

Launches all nodes for the sprayer drone:
- Telemetry RX (receives and dispatches geotags)
- Navigation (unified with ARM/TAKEOFF/NAV)
- Detection & Centering (MERGED NODE)
- Sprayer Control

Launch arguments:
    use_sim: Use simulation mode (default: true)
            - true: Sprayer uses PWM topic for testing
            - false: Sprayer uses MAVROS servo command for hardware relay
    log_level: Logging level (default: info)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Drone-2."""
    
    # Launch arguments
    use_sim = LaunchConfiguration('use_sim')
    log_level = LaunchConfiguration('log_level')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation mode (true=PWM topic, false=MAVROS servo)'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        
        # Telemetry RX (receives geotags from Drone-1, validates, and dispatches)
        Node(
            package='telem_rx',
            executable='telem_rx_node',
            name='telem_rx_node',
            output='screen',
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
        
        # Detection & Centering (MERGED NODE)
        Node(
            package='local_detection_and_centering',
            executable='detection_centering_node',
            name='detection_centering_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('local_detection_and_centering'), 'config', 'detection_centering_params.yaml'
            ])],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Sprayer Control (with use_sim parameter)
        Node(
            package='sprayer_control',
            executable='sprayer_control_node',
            name='sprayer_control_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('sprayer_control'), 'config', 'sprayer_params.yaml'
                ]),
                {'use_sim': use_sim}  # Override from launch argument
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])

