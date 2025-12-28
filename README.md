# src1-showcase

A small showcase repository bundling simulation scenarios and ROS 2 nodes for a prototype AMR (autonomous mobile robot) demo.

Contents
- `my_py_amr/` — ROS 2 Python package: marker detection, camera bridge, occupancy grid conversion, republish helpers, and launch files.
- `map_gazebo/a_whole_new_world.sdf` — Gazebo/Ignition SDF world used for simulation.
- `vision/` — supporting demos / docs for the showcase.

Quick start
1. Install ROS 2 (recommended: Humble or later) and Ignition Gazebo.
2. Build and source the workspace:

```bash
colcon build
. install/setup.bash
```

3. Launch camera bridge and Nav2 demo:

```bash
ros2 launch my_py_amr camera_bridge_launch.py
```

Tests
- `pytest` for unit tests, linters for code style.

License: MIT (see `LICENSE`)
