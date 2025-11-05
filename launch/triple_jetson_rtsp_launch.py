"""
Launch three gscam2 nodes with NVIDIA Jetson hardware acceleration for RTSP streams.

This launch file creates a multi-threaded component container with three independent
camera nodes, each publishing to separate topics.

Usage:
    ros2 launch gscam2 triple_jetson_rtsp_launch.py
    ros2 launch gscam2 triple_jetson_rtsp_launch.py \
        rtsp_url_1:=rtsp://192.168.137.208:8554/camera1 \
        rtsp_url_2:=rtsp://192.168.137.208:8554/camera2 \
        rtsp_url_3:=rtsp://192.168.137.208:8554/camera3
    ros2 launch gscam2 triple_jetson_rtsp_launch.py low_latency:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare launch arguments for camera 1
    rtsp_url_1_arg = DeclareLaunchArgument(
        'rtsp_url_1',
        default_value='rtsp://192.168.137.208:8554/camera1',
        description='RTSP stream URL for camera 1'
    )

    # Declare launch arguments for camera 2
    rtsp_url_2_arg = DeclareLaunchArgument(
        'rtsp_url_2',
        default_value='rtsp://192.168.137.208:8554/camera2',
        description='RTSP stream URL for camera 2'
    )

    # Declare launch arguments for camera 3
    rtsp_url_3_arg = DeclareLaunchArgument(
        'rtsp_url_3',
        default_value='rtsp://192.168.137.208:8554/camera3',
        description='RTSP stream URL for camera 3'
    )

    low_latency_arg = DeclareLaunchArgument(
        'low_latency',
        default_value='false',
        description='Enable low latency mode (reduces buffering)'
    )

    camera_name_1_arg = DeclareLaunchArgument(
        'camera_name_1',
        default_value='camera1',
        description='Camera name for camera 1'
    )

    camera_name_2_arg = DeclareLaunchArgument(
        'camera_name_2',
        default_value='camera2',
        description='Camera name for camera 2'
    )

    camera_name_3_arg = DeclareLaunchArgument(
        'camera_name_3',
        default_value='camera3',
        description='Camera name for camera 3'
    )

    frame_id_1_arg = DeclareLaunchArgument(
        'frame_id_1',
        default_value='camera1_frame',
        description='Camera frame ID for camera 1'
    )

    frame_id_2_arg = DeclareLaunchArgument(
        'frame_id_2',
        default_value='camera2_frame',
        description='Camera frame ID for camera 2'
    )

    frame_id_3_arg = DeclareLaunchArgument(
        'frame_id_3',
        default_value='camera3_frame',
        description='Camera frame ID for camera 3'
    )

    # Get launch configurations
    rtsp_url_1 = LaunchConfiguration('rtsp_url_1')
    rtsp_url_2 = LaunchConfiguration('rtsp_url_2')
    rtsp_url_3 = LaunchConfiguration('rtsp_url_3')
    low_latency = LaunchConfiguration('low_latency')
    camera_name_1 = LaunchConfiguration('camera_name_1')
    camera_name_2 = LaunchConfiguration('camera_name_2')
    camera_name_3 = LaunchConfiguration('camera_name_3')
    frame_id_1 = LaunchConfiguration('frame_id_1')
    frame_id_2 = LaunchConfiguration('frame_id_2')
    frame_id_3 = LaunchConfiguration('frame_id_3')

    # Helper function to build GStreamer pipeline configuration
    def build_gscam_config(rtsp_url, low_latency_mode=False):
        if low_latency_mode:
            # Low latency mode: ~150ms latency
            return [
                'rtspsrc location=', rtsp_url, ' latency=100 protocols=tcp ! ',
                'rtpjitterbuffer latency=50 ! ',
                'rtph264depay ! h264parse config-interval=-1 ! ',
                'nvv4l2decoder enable-max-performance=1 ! ',
                'nvvidconv ! video/x-raw,format=BGRx ! ',
                'videoconvert'
            ]
        else:
            # Standard latency mode: ~2.5s latency, more stable
            return [
                'rtspsrc location=', rtsp_url, ' latency=2000 protocols=tcp ! ',
                'rtpjitterbuffer latency=500 ! ',
                'rtph264depay ! h264parse config-interval=-1 ! ',
                'nvv4l2decoder enable-max-performance=1 skip-frames=1 ! ',
                'nvvidconv ! video/x-raw,format=BGRx ! ',
                'videoconvert'
            ]

    # Build GStreamer pipeline configurations for each camera
    # Note: We'll use the low_latency parameter to determine sync_sink value
    # but we need to build both configs here since LaunchConfiguration doesn't evaluate until runtime
    gscam_config_1_normal = build_gscam_config(rtsp_url_1, False)
    gscam_config_1_low = build_gscam_config(rtsp_url_1, True)

    gscam_config_2_normal = build_gscam_config(rtsp_url_2, False)
    gscam_config_2_low = build_gscam_config(rtsp_url_2, True)

    gscam_config_3_normal = build_gscam_config(rtsp_url_3, False)
    gscam_config_3_low = build_gscam_config(rtsp_url_3, True)

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
                parameters=[{
                    'gscam_config': gscam_config_1_normal,
                    'preroll': True,
                    'use_gst_timestamps': True,
                    'image_encoding': 'rgb8',
                    'camera_name': camera_name_1,
                    'frame_id': frame_id_1,
                    'sync_sink': True,
                }],
                remappings=[
                    ('camera_info', [camera_name_1, '/camera_info']),
                    ('image_raw', [camera_name_1, '/image_raw']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Camera 2
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_camera2',
                parameters=[{
                    'gscam_config': gscam_config_2_normal,
                    'preroll': True,
                    'use_gst_timestamps': True,
                    'image_encoding': 'rgb8',
                    'camera_name': camera_name_2,
                    'frame_id': frame_id_2,
                    'sync_sink': True,
                }],
                remappings=[
                    ('camera_info', [camera_name_2, '/camera_info']),
                    ('image_raw', [camera_name_2, '/image_raw']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Camera 3
            ComposableNode(
                package='gscam2',
                plugin='gscam2::GSCamNode',
                name='gscam_camera3',
                parameters=[{
                    'gscam_config': gscam_config_3_normal,
                    'preroll': True,
                    'use_gst_timestamps': True,
                    'image_encoding': 'rgb8',
                    'camera_name': camera_name_3,
                    'frame_id': frame_id_3,
                    'sync_sink': True,
                }],
                remappings=[
                    ('camera_info', [camera_name_3, '/camera_info']),
                    ('image_raw', [camera_name_3, '/image_raw']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        rtsp_url_1_arg,
        rtsp_url_2_arg,
        rtsp_url_3_arg,
        low_latency_arg,
        camera_name_1_arg,
        camera_name_2_arg,
        camera_name_3_arg,
        frame_id_1_arg,
        frame_id_2_arg,
        frame_id_3_arg,
        container,
    ])
