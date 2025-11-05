# NVIDIA Jetson 下通过 Jetson_gscam2 发布 RTSP 摄像头流

## 致谢

本项目部分实现基于 [clydemcqueen/gscam2](https://github.com/clydemcqueen/gscam2.git)，感谢其开源贡献。

## 安装与编译指南

适用版本：**ROS2 Foxy, Galactic, Humble, Rolling**
详细安装流程见 [Dockerfile](Dockerfile)。

### 步骤 0：克隆本仓库

```bash
# 进入 ROS2 工作空间的 src 目录
cd ~/ros2_ws/src  # 或者你的工作空间路径，如 cd ~/tarts_ws/src

# 克隆本仓库
git clone https://github.com/Yinmlmaoliang/Jetson_gscam2.git gscam2
```

### 步骤 1：安装 GStreamer 依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  libgstreamer1.0-0 \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-gl \
  gstreamer1.0-gtk3 \
  gstreamer1.0-qt5 \
  gstreamer1.0-pulseaudio \
  libgstreamer-plugins-base1.0-dev
```

### 步骤 2：克隆并编译 `ros2_shared` 依赖

```bash
# 克隆 ros2_shared 依赖到工作空间 src 目录
cd ~/ros2_ws/src  # 确保在工作空间的 src 目录
git clone https://github.com/ptrmu/ros2_shared.git -b master

# 返回工作空间根目录并编译
cd ~/ros2_ws  # 或 cd ~/tarts_ws
colcon build --packages-select ros2_shared
```

### 步骤 3：安装工作空间依赖项

```bash
# 返回工作空间根目录（如果不在的话）
cd ~/ros2_ws  # 或 cd ~/tarts_ws

# 安装依赖
rosdep install -y --from-paths . --ignore-src
```

### 步骤 4：编译 Jetson_gscam2

```bash
colcon build --packages-select gscam2 --symlink-install
```

> **注意（Jetson平台）：**  
> 确保下列两个NVIDIA加速插件可用，否则无法硬解码RTSP流：
>
> ```bash
> gst-inspect-1.0 nvv4l2decoder
> gst-inspect-1.0 nvvidconv
> ```

## Jetson平台使用

### 单相机节点启动

```bash
# 确保已经 source 工作空间
source ~/ros2_ws/install/setup.bash

# 使用默认 RTSP URL 启动
ros2 launch gscam2 jetson_rtsp_launch.py

# 指定自定义 RTSP URL
ros2 launch gscam2 jetson_rtsp_launch.py rtsp_url:=rtsp://192.168.1.100:8554/stream

# 启用低延迟模式
ros2 launch gscam2 jetson_rtsp_launch.py low_latency:=true

# 自定义相机名称和Frame ID
ros2 launch gscam2 jetson_rtsp_launch.py camera_name:=my_camera frame_id:=my_camera_frame
```

---

### 三相机节点启动

本方案使用单个 `component_container_mt` 多线程容器，容器内运行三个可组合节点：

- `gscam_camera1`：处理第一个摄像头
- `gscam_camera2`：处理第二个摄像头
- `gscam_camera3`：处理第三个摄像头

```bash
# 确保已经 source 工作空间
source ~/ros2_ws/install/setup.bash

# 启动三路摄像头
ros2 launch gscam2 triple_jetson_rtsp_launch.py
```