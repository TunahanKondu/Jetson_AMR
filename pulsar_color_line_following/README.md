# pulsar_color_line_following

ROS 2 Humble package for orange-line detection, alignment-safe motion control,
encoder-distance completion and optional RTP/H.264 streaming.

Version 0.1.4 makes `LINE_START` idempotent and publishes
`LINE_START_ACCEPTED` only after the controller and detector have both started
and the first camera frame has been processed. Retried commands therefore do
not reset PID or encoder-distance state. `LINE_STOP` publishes separate
detector and controller acknowledgements after streaming and motion stop.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select pulsar_color_line_following --symlink-install
source install/setup.bash
```

## Launch with streaming

```bash
ros2 launch pulsar_color_line_following color_line_following.launch.py \
  show_image:=true \
  gstreamer_enabled:=true \
  gstreamer_host:=172.20.10.6 \
  gstreamer_port:=5000
```

The RealSense ROS driver must publish
`/camera/camera/color/image_raw`. `LINE_START` starts line detection, motion
control and GStreamer streaming. `LINE_STOP` stops all three:

```bash
ros2 topic pub --once /robot_action std_msgs/msg/String "{data: 'LINE_START'}"
```

Successful startup is acknowledged on `/amr/robot_action_result`:

```text
LINE_START_ACCEPTED
```

Successful stop produces both:

```text
LINE_DETECTOR_STOPPED
LINE_CONTROLLER_STOPPED
```

Receiver:

```bash
gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```
