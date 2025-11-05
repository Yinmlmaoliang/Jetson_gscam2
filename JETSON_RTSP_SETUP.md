# NVIDIA Jetson RTSP 相机配置指南

本指南说明如何在 NVIDIA Jetson 平台上使用 gscam2 发布 RTSP 摄像头流。

## 硬件加速插件

此配置使用以下 NVIDIA Jetson 专用的 GStreamer 插件：
- `nvv4l2decoder`: 硬件加速 H.264 解码器
- `nvvidconv`: NVIDIA 视频转换器

## 快速开始

### 方法 1: 使用参数文件

```bash
# 标准延迟模式（更稳定）
ros2 run gscam2 gscam_main --ros-args --params-file $(ros2 pkg prefix gscam2)/share/gscam2/cfg/jetson_rtsp_params.yaml

# 低延迟模式（延迟更低，可能不太稳定）
ros2 run gscam2 gscam_main --ros-args --params-file $(ros2 pkg prefix gscam2)/share/gscam2/cfg/jetson_rtsp_lowlatency_params.yaml
```

### 方法 2: 使用启动文件

```bash
# 使用默认 RTSP URL
ros2 launch gscam2 jetson_rtsp_launch.py

# 指定自定义 RTSP URL
ros2 launch gscam2 jetson_rtsp_launch.py rtsp_url:=rtsp://192.168.1.100:8554/stream

# 使用低延迟模式
ros2 launch gscam2 jetson_rtsp_launch.py low_latency:=true

# 自定义相机名称
ros2 launch gscam2 jetson_rtsp_launch.py camera_name:=my_camera frame_id:=my_camera_frame
```

### 方法 3: 使用环境变量

```bash
export GSCAM_CONFIG="rtspsrc location=rtsp://192.168.137.208:8554/camera1 latency=2000 protocols=tcp ! rtpjitterbuffer latency=500 ! rtph264depay ! h264parse config-interval=-1 ! nvv4l2decoder enable-max-performance=1 skip-frames=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert"

ros2 run gscam2 gscam_main --ros-args -p preroll:=true -p use_gst_timestamps:=true
```

## 验证图像发布

在另一个终端中运行以下命令来验证图像是否正确发布：

```bash
# 查看可用话题
ros2 topic list

# 查看图像话题信息
ros2 topic info /image_raw

# 查看图像发布频率
ros2 topic hz /image_raw

# 使用 rqt_image_view 查看图像
ros2 run rqt_image_view rqt_image_view
```

## GStreamer 管道说明

```
rtspsrc location=<RTSP_URL> latency=2000 protocols=tcp
  └─ 从 RTSP 服务器接收流，使用 TCP 协议

rtpjitterbuffer latency=500
  └─ RTP 抖动缓冲，减少网络抖动影响

rtph264depay
  └─ 从 RTP 包中提取 H.264 数据

h264parse config-interval=-1
  └─ 解析 H.264 流，config-interval=-1 确保定期插入 SPS/PPS

nvv4l2decoder enable-max-performance=1 skip-frames=1
  └─ NVIDIA 硬件解码器，启用最大性能模式，允许跳帧

nvvidconv
  └─ NVIDIA 视频格式转换

video/x-raw,format=BGRx
  └─ 输出 BGRx 格式的原始视频

videoconvert
  └─ 转换为 ROS 兼容的格式（通常是 RGB8）
```

## 参数调优

### 降低延迟

1. 减少 `latency` 参数：
   - `rtspsrc latency=100`（默认 2000ms）
   - `rtpjitterbuffer latency=50`（默认 500ms）

2. 设置 `sync_sink: False` 禁用同步

3. 移除 `skip-frames=1` 参数以不跳帧

### 提高稳定性

1. 增加 `latency` 参数
2. 设置 `preroll: True`
3. 设置 `sync_sink: True`

### 调整图像质量

在 `nvvidconv` 后添加分辨率限制：
```
nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1080 ! videoconvert
```