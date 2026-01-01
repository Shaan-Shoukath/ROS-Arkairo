#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════════════════════
Detection Test Launch File
═══════════════════════════════════════════════════════════════════════════════

Launch the disease detection test node for calibration and testing.

USAGE:
    # Simulation mode (mock GPS, no hardware needed)
    ros2 launch detection_and_geotag detection_test.launch.py use_sim:=true

    # Real mode (GPS from Orange Cube+ via MAVROS)
    ros2 launch detection_and_geotag detection_test.launch.py use_sim:=false

    # With live preview window (requires display)
    ros2 launch detection_and_geotag detection_test.launch.py show_window:=true

═══════════════════════════════════════════════════════════════════════════════
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package paths
    pkg_detection = get_package_share_directory('detection_and_geotag')
    
    # Config file path
    test_params_file = os.path.join(pkg_detection, 'config', 'test_params.yaml')
    
    # ═══════════════════════════════════════════════════════════════════
    # LAUNCH ARGUMENTS
    # ═══════════════════════════════════════════════════════════════════
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Use simulation mode with mock GPS (true) or real MAVROS GPS (false)'
    )
    
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='false',
        description='Show live OpenCV preview window (requires display, set true for testing)'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    # ═══════════════════════════════════════════════════════════════════
    # NODES
    # ═══════════════════════════════════════════════════════════════════
    detection_test_node = Node(
        package='detection_and_geotag',
        executable='detection_test_node',
        name='detection_test_node',
        output='screen',
        parameters=[
            test_params_file,
            {
                'use_sim': LaunchConfiguration('use_sim'),
                'show_gui': LaunchConfiguration('show_window'),
                'camera_topic': LaunchConfiguration('camera_topic'),
            }
        ],
    )
    
    return LaunchDescription([
        use_sim_arg,
        show_window_arg,
        camera_topic_arg,
        detection_test_node,
    ])
