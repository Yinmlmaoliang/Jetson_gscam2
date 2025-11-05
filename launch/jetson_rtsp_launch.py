"""
Launch gscam2 with NVIDIA Jetson hardware acceleration for RTSP streams.

Usage:
    ros2 launch gscam2 jetson_rtsp_launch.py
    ros2 launch gscam2 jetson_rtsp_launch.py rtsp_url:=rtsp://192.168.1.100:8554/stream
    ros2 launch gscam2 jetson_rtsp_launch.py low_latency:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    # Declare launch arguments
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://192.168.137.208:8554/camera1',
        description='RTSP stream URL'
    )

    low_latency_arg = DeclareLaunchArgument(
        'low_latency',
        default_value='false',
        description='Enable low latency mode (reduces buffering)'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='jetson_rtsp_camera',
        description='Camera name for topics'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_frame',
        description='Camera frame ID'
    )

    # Get launch configurations
    rtsp_url = LaunchConfiguration('rtsp_url')
    low_latency = LaunchConfiguration('low_latency')
    camera_name = LaunchConfiguration('camera_name')
    frame_id = LaunchConfiguration('frame_id')

    # Build GStreamer pipeline configurations
    # Standard latency mode: more buffering, stable, ~2.5s latency
    # - rtspsrc latency=2000ms: larger network buffer
    # - rtpjitterbuffer latency=500ms: larger jitter buffer
    # - skip-frames=1: skip frames if needed to maintain realtime
    # - sync_sink=True: wait for all frames to process (stable output)
    gscam_config_normal = [
        'rtspsrc location=', rtsp_url, ' latency=2000 protocols=tcp ! ',
        'rtpjitterbuffer latency=500 ! ',
        'rtph264depay ! h264parse config-interval=-1 ! ',
        'nvv4l2decoder enable-max-performance=1 skip-frames=1 ! ',
        'nvvidconv ! video/x-raw,format=BGRx ! ',
        'videoconvert'
    ]

    # Low latency mode: less buffering, lower stability, ~150ms latency
    # - rtspsrc latency=100ms: minimal network buffer
    # - rtpjitterbuffer latency=50ms: minimal jitter buffer
    # - skip-frames=0: process all frames
    # - sync_sink=False: output latest frame immediately (may drop late frames)
    gscam_config_low = [
        'rtspsrc location=', rtsp_url, ' latency=100 protocols=tcp ! ',
        'rtpjitterbuffer latency=50 ! ',
        'rtph264depay ! h264parse config-interval=-1 ! ',
        'nvv4l2decoder enable-max-performance=1 ! ',
        'nvvidconv ! video/x-raw,format=BGRx ! ',
        'videoconvert'
    ]

    # Determine which config to use based on low_latency parameter
    # This is evaluated at launch time
    gscam_config = gscam_config_low if str(low_latency).lower() == 'true' else gscam_config_normal
    sync_sink = False if str(low_latency).lower() == 'true' else True

    # Create the gscam2 node
    gscam_node = Node(
        package='gscam2',
        executable='gscam_main',
        name='gscam_publisher',
        parameters=[{
            'gscam_config': gscam_config,
            'preroll': True,
            'use_gst_timestamps': True,
            'image_encoding': 'rgb8',
            'camera_name': camera_name,
            'frame_id': frame_id,
            'sync_sink': sync_sink,  # False for low-latency mode, True for stable mode
        }],
        remappings=[
            ('camera_info', [camera_name, '/camera_info']),
            ('image_raw', [camera_name, '/image_raw']),
        ],
        output='screen',
    )

    return LaunchDescription([
        rtsp_url_arg,
        low_latency_arg,
        camera_name_arg,
        frame_id_arg,
        gscam_node,
    ])
