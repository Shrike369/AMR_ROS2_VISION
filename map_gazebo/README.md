# camera_bot: Capture full 100×100 ground plane

## Summary ✅
- The world uses a ground plane of size **100 × 100** (see `a_whole_new_world.sdf`).
- I raised the `camera_bot` model so the camera will cover the entire plane with the current `horizontal_fov` (1.8926 rad). The include pose is now `0 0 36.50 0 1.57 0` (sensor is at ~36.58 m due to the sensor local z=0.08).
- The camera sensor resolution was set to **1000 × 1000** (square) to give a high-resolution, more reasonable frame size for testing.

## Files changed 🔧
- `a_whole_new_world.sdf` — changed camera include pose to `z=36.50`.
- `model.sdf` — set camera image `width` and `height` to `10000` and added a performance warning.
 - `a_whole_new_world.sdf` — added `border_walls` model: four black box walls around the 100×100 plane (thickness 0.03 m, height 0.5 m).
 - `a_whole_new_world.sdf` — updated `border_walls` to match the camera coverage area (full width ≈ **24.038 m**, half ≈ **11 m**). Each wall: thickness 0.03 m, height 0.5 m, centered at ±11 m.

## Important caveats ⚠️
- 10000×10000 ≈ **100 million pixels**; uncompressed RGB is ~300 MB per image. Many GPUs/drivers limit texture sizes (commonly 8192 or 16384) and the simulator or GUI may crash or fail to allocate such a large image.
  - If you see crashes or `Out of memory` errors, use one of these alternatives:
  - Use a lower resolution (e.g., 512, 1024, or 4096) if that meets your needs.
  - Capture tiles: render several overlapping smaller images (e.g., 4096×4096) by changing camera pitch/pose or horizontal_fov and stitch them offline.
  - Use a headless/offscreen renderer if available to avoid GUI limitations.

## How to run the world and capture an image (suggested steps) 🧭
1. Start Gazebo Sim (from the project folder):

```bash
gz sim a_whole_new_world.sdf
```

2. In another terminal, list transport topics to find the camera image topic:

```bash
gz topic -l
```

Look for a topic that includes `camera` or `top_camera` in its name. The exact name depends on the simulator version and naming conventions.

3. To export a single image (example; replace `<image_topic>` with the real topic):

```bash
gz topic -e <image_topic> -n 1 --raw > image.msg
```

You may need an extra tool or script to decode the raw message to a PNG, or use the GUI's camera view and take a screenshot.

If you are using ROS2 with a bridge, you can also use `ros2 topic` tools to receive the sensor messages and convert to images.

## If you want I can:
- Add an example script to tile the map by adjusting camera pose/fov and stitch the pieces together automatically.
- Provide a small helper to convert/safe the raw image topic into a PNG using python (depending on the exact published message format).

---
If you'd like, I can now add a tiling/stitching script or test instructions tailored to your environment (headless, ROS2 bridge, or GUI). Let me know which approach you prefer and whether you want me to proceed. ✨