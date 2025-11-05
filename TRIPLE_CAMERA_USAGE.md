# 三路RTSP摄像头启动指南

本文档说明如何使用 `triple_jetson_rtsp_launch.py` 启动三个RTSP摄像头并发布视频流到独立的ROS话题。

## 功能特性

- **多摄像头支持**：同时启动三个独立的RTSP摄像头节点
- **硬件加速**：使用NVIDIA Jetson硬件解码器（nvv4l2decoder）和转换器（nvvidconv）
- **进程间通信优化**：使用多线程容器（component_container_mt）和进程内通信（IPC）
- **独立话题**：每个摄像头发布到独立的ROS话题
- **可配置延迟**：支持标准模式（~2.5s延迟，更稳定）和低延迟模式（~150ms延迟）

## 架构说明

### 容器和节点
- 使用单个 `component_container_mt` 多线程容器
- 容器内运行三个可组合节点：
  - `gscam_camera1`：处理第一个摄像头
  - `gscam_camera2`：处理第二个摄像头
  - `gscam_camera3`：处理第三个摄像头

### 话题结构
每个摄像头发布到独立的命名空间：
```
/camera1/image_raw          (sensor_msgs/Image)
/camera1/camera_info        (sensor_msgs/CameraInfo)

/camera2/image_raw          (sensor_msgs/Image)
/camera2/camera_info        (sensor_msgs/CameraInfo)

/camera3/image_raw          (sensor_msgs/Image)
/camera3/camera_info        (sensor_msgs/CameraInfo)
```

### GStreamer管道
每个摄像头使用以下管道结构：
```
rtspsrc → rtpjitterbuffer → rtph264depay → h264parse →
nvv4l2decoder → nvvidconv → videoconvert
```

## 使用方法

### 1. 基本用法（使用默认URL）
```bash
# 确保已经source工作空间
source /home/nvidia/ros/tarts_ws/install/setup.bash

# 启动三个摄像头
ros2 launch gscam2 triple_jetson_rtsp_launch.py
```

默认RTSP URLs：
- Camera 1: `rtsp://192.168.137.208:8554/camera1`
- Camera 2: `rtsp://192.168.137.208:8554/camera2`
- Camera 3: `rtsp://192.168.137.208:8554/camera3`

### 2. 自定义RTSP URLs
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py \
    rtsp_url_1:=rtsp://192.168.1.100:8554/stream1 \
    rtsp_url_2:=rtsp://192.168.1.101:8554/stream1 \
    rtsp_url_3:=rtsp://192.168.1.102:8554/stream1
```

### 3. 启用低延迟模式
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py low_latency:=true
```

**注意**：目前low_latency参数的动态切换有限制，建议在启动时设置。

### 4. 自定义摄像头名称和frame_id
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py \
    camera_name_1:=front_camera \
    camera_name_2:=left_camera \
    camera_name_3:=right_camera \
    frame_id_1:=front_camera_frame \
    frame_id_2:=left_camera_frame \
    frame_id_3:=right_camera_frame
```

这将发布到以下话题：
```
/front_camera/image_raw
/left_camera/image_raw
/right_camera/image_raw
```

## 启动参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rtsp_url_1` | `rtsp://192.168.137.208:8554/camera1` | 第一个摄像头的RTSP URL |
| `rtsp_url_2` | `rtsp://192.168.137.208:8554/camera2` | 第二个摄像头的RTSP URL |
| `rtsp_url_3` | `rtsp://192.168.137.208:8554/camera3` | 第三个摄像头的RTSP URL |
| `low_latency` | `false` | 启用低延迟模式（减少缓冲） |
| `camera_name_1` | `camera1` | 第一个摄像头的名称（用于话题命名） |
| `camera_name_2` | `camera2` | 第二个摄像头的名称 |
| `camera_name_3` | `camera3` | 第三个摄像头的名称 |
| `frame_id_1` | `camera1_frame` | 第一个摄像头的frame ID |
| `frame_id_2` | `camera2_frame` | 第二个摄像头的frame ID |
| `frame_id_3` | `camera3_frame` | 第三个摄像头的frame ID |

## 验证和监控

### 查看话题列表
```bash
ros2 topic list
```

应该看到：
```
/camera1/camera_info
/camera1/image_raw
/camera2/camera_info
/camera2/image_raw
/camera3/camera_info
/camera3/image_raw
```

### 查看话题信息
```bash
# 查看发布频率和消息类型
ros2 topic hz /camera1/image_raw
ros2 topic hz /camera2/image_raw
ros2 topic hz /camera3/image_raw

# 查看话题详细信息
ros2 topic info /camera1/image_raw -v
```

### 查看图像（使用rqt_image_view）
```bash
# 查看camera1
ros2 run rqt_image_view rqt_image_view /camera1/image_raw

# 或使用image_view
ros2 run image_view image_view --ros-args -r image:=/camera1/image_raw
```