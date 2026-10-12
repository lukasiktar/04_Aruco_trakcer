# aruco_tracker

ROS 2 (Jazzy) package that detects an ArUco marker with an Intel RealSense D435i, centres it on a
crosshair and outputs `geometry_msgs/Twist` on `/cmd_vel` (`linear.y` / `linear.z`) to steer a drone.

- Detects markers, draws their axes and shows the distance to the camera.
- Filters detections over time (consecutive-frame confirmation, outlier rejection, smoothing) and tracks the longest-lived marker.
- PID regulator on the horizontal and vertical offset between the image centre and the marker centre.
- Publishes zero velocity whenever no marker is tracked.

## Build and run

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aruco_tracker
source install/setup.bash
ros2 launch aruco_tracker aruco_tracker.launch.py marker_size:=0.10
```

Launch arguments: `marker_size` (m), `dictionary_family` (`5x5`), `dictionary` (`1000`), `view` (`true`).
Requires `realsense2_camera`, `cv_bridge` and OpenCV with the `aruco` module.

## Topics

| Topic | Type | |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.y`, `linear.z` from the regulator (body frame FLU: y left, z up) |
| `/aruco_tracker_node/image` | `sensor_msgs/Image` | annotated debug image |
| `/aruco_tracker_node/distance` | `std_msgs/Float32` | distance to the tracked marker (m) |

Node parameters: `kp`, `ki`, `kd`, `max_velocity`, `deadband_m`, `invert_y`, `invert_z`, `cmd_vel_topic`,
`min_consecutive_frames`, `lost_timeout_sec`, `smoothing_alpha`, `max_jump_m`, `qos_reliability`.

> Nothing gates the output: commands are sent as soon as a marker is confirmed. Add a safety switch
> and velocity limits downstream before flying.
