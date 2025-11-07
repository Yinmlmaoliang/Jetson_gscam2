# Triple Camera Configuration Guide

## Configuration Files

### 1. Standard Latency Mode
**File:** `triple_jetson_rtsp_params.yaml`
- Latency: ~2.5 seconds
- More stable connection
- Best for: Recording, monitoring applications

**Run command:**
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py
```

### 2. Low Latency Mode
**File:** `triple_jetson_rtsp_lowlatency_params.yaml`
- Latency: ~150ms
- Less stable, may drop frames
- Best for: Real-time control, robotics applications

**Run command:**
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py \
    params_file:=$(ros2 pkg prefix gscam2)/share/gscam2/cfg/triple_jetson_rtsp_lowlatency_params.yaml
```

### 3. Custom Configuration
You can create your own configuration file and specify its path:

**Run command:**
```bash
ros2 launch gscam2 triple_jetson_rtsp_launch.py \
    params_file:=/path/to/your/custom_params.yaml
```

## Configuration File Format

Each configuration file must define parameters for all three cameras:

```yaml
gscam_camera1:
  ros__parameters:
    gscam_config: 'rtspsrc location=rtsp://IP:PORT/camera1 ...'
    preroll: True
    use_gst_timestamps: True
    image_encoding: 'rgb8'
    camera_name: 'camera1'
    frame_id: 'camera1_frame'
    sync_sink: True  # Set to False for lower latency

gscam_camera2:
  ros__parameters:
    # Same structure as camera1
    ...

gscam_camera3:
  ros__parameters:
    # Same structure as camera1
    ...
```

## Published Topics

- `/camera1/image_raw` - Camera 1 image stream
- `/camera1/camera_info` - Camera 1 info
- `/camera2/image_raw` - Camera 2 image stream
- `/camera2/camera_info` - Camera 2 info
- `/camera3/image_raw` - Camera 3 image stream
- `/camera3/camera_info` - Camera 3 info

## Key Parameters to Adjust

### For Lower Latency
- Reduce `rtspsrc latency` (e.g., 100ms)
- Reduce `rtpjitterbuffer latency` (e.g., 50ms)
- Set `sync_sink: False`

### For More Stability
- Increase `rtspsrc latency` (e.g., 2000ms)
- Increase `rtpjitterbuffer latency` (e.g., 500ms)
- Set `sync_sink: True`
- Keep or add `skip-frames=1` in nvv4l2decoder

## RTSP Server Configuration

Update the RTSP URLs in the configuration file to match your camera setup:

```yaml
gscam_config: 'rtspsrc location=rtsp://YOUR_IP:YOUR_PORT/camera_path ...'
```

Current default: `rtsp://192.168.137.208:8554/camera[1-3]`

