"""
Launch three gscam2 nodes with NVIDIA Jetson hardware acceleration for RTSP streams.

This launch file creates a multi-threaded component container with three independent
camera nodes, each publishing to separate topics.

Usage:
    # Standard latency mode
    ros2 launch gscam2 triple_jetson_rtsp_launch.py
    
    # Low latency mode
    ros2 launch gscam2 triple_jetson_rtsp_launch.py \
        params_file:=$(ros2 pkg prefix gscam2)/share/gscam2/cfg/triple_jetson_rtsp_lowlatency_params.yaml
    
    # Custom params file
    ros2 launch gscam2 triple_jetson_rtsp_launch.py \
        params_file:=/path/to/your/params.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('gscam2')
    
    # Default params file path (standard latency mode)
    default_params_file = os.path.join(
        pkg_share, 'cfg', 'triple_jetson_rtsp_params.yaml'
    )
    
    # Declare launch argument for params file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file for triple camera setup'
    )
    
    # Get launch configuration
    params_file = LaunchConfiguration('params_file')
    
    # Create a multi-threaded component container with three camera nodes
    container = ComposableNodeContainer(
        name='gscam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Camera 1
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_camera1',
                parameters=[params_file],
                remappings=[
                    ('camera_info', 'camera1/camera_info'),
                    ('image_raw', 'camera1/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Camera 2
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_camera2',
                parameters=[params_file],
                remappings=[
                    ('camera_info', 'camera2/camera_info'),
                    ('image_raw', 'camera2/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Camera 3
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_camera3',
                parameters=[params_file],
                remappings=[
                    ('camera_info', 'camera3/camera_info'),
                    ('image_raw', 'camera3/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        container,
    ])
