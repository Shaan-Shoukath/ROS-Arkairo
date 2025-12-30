#!/usr/bin/env python3
# Copyright 2024 Shaan Shoukath
# SPDX-License-Identifier: Apache-2.0

"""
Drone-1 Survey System Launch File

Launches all nodes for the survey drone:
- KML Lane Planner
- Navigation
- Image Capture
- Detection and Geotag
- GCS Uplink
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Drone-1."""
    
    # Launch arguments
    use_sim = LaunchConfiguration('use_sim')
    log_level = LaunchConfiguration('log_level')
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation mode'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'
        ),
        
        # KML Lane Planner
        Node(
            package='kml_lane_planner',
            executable='kml_lane_planner_node',
            name='kml_lane_planner_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('kml_lane_planner'),
                    'config', 'planner_params.yaml'
                ]),
                {'require_gps_home': False}  # Use default home for easier testing
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Navigation
        Node(
            package='drone1_navigation',
            executable='drone1_navigation_node',
            name='drone1_navigation_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('drone1_navigation'),
                    'config', 'navigation_params.yaml'
                ])
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Image Capture
        Node(
            package='image_capture',
            executable='image_capture_node',
            name='image_capture_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('image_capture'),
                    'config', 'camera_params.yaml'
                ]),
                {'use_usb_camera': use_sim}
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Detection and Geotag
        Node(
            package='detection_and_geotag',
            executable='detection_and_geotag_node',
            name='detection_and_geotag_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('detection_and_geotag'),
                    'config', 'detection_params.yaml'
                ])
            ],
            arguments=['--ros-args', '--log-level', log_level]
        ),
        
        # Telemetry TX (direct drone-to-drone communication)
        Node(
            package='telem_tx',
            executable='telem_tx_node',
            name='telem_tx_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])
