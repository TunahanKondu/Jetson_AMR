# Pulsar color line following

First implementation of orange center stripe tracking for the start area in the 2026 SRU additional technical specification (Figure 5). The document states QR size as 50 x 50 mm and start segment length as 1.5 m. Individual orange/blue stripe widths are not dimensioned; HSV and contour geometry are adjustable rather than based on a fixed pixel width.

Run `ros2 launch pulsar_color_line_following color_line_following.launch.py show_image:=true` to open a local preview window after building and sourcing the workspace. Use `show_image:=false` for headless operation. Publish `/robot_action` (`std_msgs/String`) with `LINE_START` to begin and `LINE_STOP` to stop. The detector publishes `/color_line_detected` and `/color_line_error`; the controller publishes `/cmd_vel`. The debug image is `/color_line_debug_image/compressed` and is generated only when subscribed.

The orange HSV range, camera topic, ROI and control gains are ROS parameters. Tune the HSV bounds with a real camera image because arena lighting and camera white balance are unknown. The ROI ends at 88% of image height to reduce interference from the QR at the start of the stripe. The old and new controllers both publish `/cmd_vel`; run only one controller at a time.

Send the actions with `ros2 topic pub --once /robot_action std_msgs/msg/String "{data: LINE_START}"` and `ros2 topic pub --once /robot_action std_msgs/msg/String "{data: LINE_STOP}"`.

Reverse drive: the launch defaults to `linear_direction:=-1.0`, so an active track command has negative `linear.x`. The angular correction keeps the existing `steering_sign:=-1.0` until its direction is checked on the robot. Set `steering_sign:=1.0` if a line on the right causes the vehicle to steer away from it. Command sign depends on camera orientation and drivetrain configuration.
