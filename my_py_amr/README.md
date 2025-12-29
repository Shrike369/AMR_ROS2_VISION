# my_py_amr

🔧 Lightweight ROS 2 utilities for a top-down camera-based AMR (Autonomous Mobile Robot)

This package contains a small set of Python nodes and launch files that were developed to support a simulated top-down camera setup. The components are intentionally simple, practical, and focused on making Gazebo-simulated cameras and robot state usable with standard ROS 2 tools (Nav2, tf, /odom, /cmd_vel, etc.).

---

## 🚀 Quick overview

- **camera_bridge** — Re-publishes bridged Gazebo camera topics into conventional ROS 2 topics (`/camera/image_raw` and `/camera/camera_info`).
- **image_to_occupancy** — Converts an RGB image into an `nav_msgs/OccupancyGrid` (white→occupied, black→free, other→unknown). Publishes `/map` and `/local_map`.
- **top_down_marker_tf** — Detects a red (front) and blue (rear) marker on the robot from a top-down camera image, computes robot (x, y, yaw) in the camera/map frame, and publishes a `map->base_link` TF (and optional `PoseStamped`).
- **republish_cmd_vel / republish_odometry / republish_joint_states** — Small helpers to forward topics or synthesize `joint_states` from odometry for systems that use non-standard topic names.

---

## ✅ Features

- Simple, robust image-based marker detection using HSV thresholds and contour centroids
- Pixel→metric conversion using camera intrinsics + fixed camera height
- Smoothing and last-pose hold options for stable TF publication
- Convert camera images directly into occupancy grids suitable for Nav2
- Pluggable parameter-based configuration for topic names and thresholds

---

## 🏗️ Architecture & design notes

The package is deliberately small and modular. The main ideas are:

- Keep vision code isolated so it can be unit-tested without a ROS runtime (see `detect_markers` in `top_down_marker_tf.py`).
- Use HSV color thresholds + simple morphological operations to detect colored markers (fast and easy to tune).
- Convert detected pixel coordinates into world coordinates using the pinhole camera model and a fixed camera height:

	- x = (u - cx) * height / fx
	- y = (v - cy) * height / fy

	Then compute yaw from the vector between the two markers: yaw = atan2(y_red - y_blue, x_red - x_blue).

- Occupancy mapping is done by classifying pixels: bright (white) → occupied (100), dark (black) → free (0), others → unknown (-1). The result is flipped vertically to convert image coordinates to map coordinates.

---

## Files & responsibilities

- `my_py_amr/camera_bridge_node.py` — Forward bridged Gazebo camera topics to conventional ROS 2 topics.
- `my_py_amr/image_to_occupancy.py` — Convert `/camera/image_raw` to `/map` and `/local_map` OccupancyGrid messages.
- `my_py_amr/top_down_marker_tf.py` — Marker detection, pose estimation, TF publication, optional debug image output.
- `my_py_amr/republish_cmd_vel.py` — Forward `/cmd_vel_nav` → `/cmd_vel` (configurable topics).
- `my_py_amr/republish_odometry.py` — Forward simulator odometry to `/odom`.
- `my_py_amr/republish_joint_states.py` — Integrate wheel angular velocities from odometry and publish `/joint_states`.
- `launch/camera_bridge_launch.py` — Launch file that starts the `ros_gz_bridge` parameter bridge, camera bridge, occupancy node, top marker TF, republish helpers, and Nav2 bringup (configured for convenience in simulation).

---

## Parameters & tuning 🔧

Key parameters (most can be passed during node startup or set in the launch file):

- `top_down_marker_tf`:
	- `camera_topic` — image topic to use (default `/camera/image_raw`)
	- `camera_info_topic` — camera info (default `/camera/camera_info`)
	- `camera_frame` — frame for TF and Pose (default `map`)
	- `base_frame` — child frame id for published transform (default `base_link`)
	- `camera_height` — height above ground (meters) used when projecting pixels to meters
	- `invert_y` — flip lateral sign if image Y axis direction differs from world (boolean)
	- `red_hsv_low1`, `red_hsv_high1`, `red_hsv_low2`, `red_hsv_high2` — HSV ranges for red (two ranges to handle hue wrap-around)
	- `blue_hsv_low`, `blue_hsv_high` — HSV range for blue marker
	- `debug` — publish annotated debug image on `/top_down/debug_image`
	- `publish_pose` — also publish `PoseStamped` on `/detected_pose`

- `image_to_occupancy`:
	- `resolution` is currently fixed in code (0.05 m/cell) but can be adapted by editing the script or adding a parameter

For quick experiments, change HSV thresholds and enable `debug` to get annotated images which help find the right thresholds.

---

## Usage examples

Build and source the workspace:

```bash
colcon build --packages-select my_py_amr
source install/setup.bash
```

Use the provided launch file to start the simulation helpers and Nav2 bringup:

```bash
ros2 launch my_py_amr camera_bridge_launch.py
```

Or run a node directly (after `source install/setup.bash`):

```bash
ros2 run my_py_amr top_down_marker_tf
ros2 run my_py_amr camera_bridge
ros2 run my_py_amr image_to_occupancy
```

You can override parameters at runtime with `--ros-args -p <name>:=<value>` or via launch files.

---

## Testing ✅

This package includes unit tests for the marker detection logic. To run tests:

```bash
python3 -m pytest -q
# or use colcon test to run tests within an ament context
colcon test --packages-select my_py_amr
```

The detection function (`detect_markers`) is written to be testable without ROS so tests can run quickly in CI.

---

## Troubleshooting & tips ⚠️

- If markers are not being detected:
	- Enable `debug` to get annotated images on `/top_down/debug_image` and inspect the masks saved to `/tmp/top_down_mask_*`.
	- Tune the HSV ranges (especially when lighting changes) and ensure `camera_height` and intrinsics (`CameraInfo`) are correct.

- If Nav2 cannot find the map, ensure `image_to_occupancy` publishes `/map` with TRANSIENT_LOCAL durability (the node does this by default).

- If images aren't appearing, verify `ros_gz_bridge` is running and that the bridge topics listed in the launch file match the Gazebo model names in your world.

---

## Contributing & extending 💡

- To add more robust detection (e.g., feature-based tracking or ArUco markers), isolate the detector interface and replace `detect_markers` while keeping the TF and publishing logic.
- Add parameters for `image_to_occupancy` resolution and threshold values if you need different map scales or color heuristics.

---

## License & authors

Maintainer: **shree**

This repository currently does not declare a specific open-source license in `package.xml` or `setup.py`. Add a license file (e.g., `LICENSE`) and update `package.xml` when you decide which license to apply.

Questions or improvements? Open an issue or send a patch — small, readable contributions are welcome.

---

Made with ❤️ — simple, pragmatic tools to bridge vision, simulation, and ROS 2 navigation.

