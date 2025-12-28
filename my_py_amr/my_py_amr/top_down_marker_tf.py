"""Top-down marker TF node.

Detects red (front) and blue (rear) markers in a top-down camera image,
computes the robot X,Y and yaw (map frame), and publishes a map->base_link TF.

Provides debug image publishing and optional PoseStamped topic.
"""
from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

import cv2
import numpy as np

# Make imports optional so the pure detection function can be unit-tested
# without a ROS runtime available in the environment running pytest.
HAS_ROS = True
try:
    import rclpy  # type: ignore
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import TransformBroadcaster
except Exception:  # pragma: no cover - exercised in test environment
    HAS_ROS = False
    # Provide minimal placeholders so module-level names exist
    CvBridge = None
    PoseStamped = object
    TransformStamped = object
    Node = object
    CameraInfo = object
    Image = object
    TransformBroadcaster = None


def detect_markers(
    rgb_img: np.ndarray,
    red_ranges: Sequence[Tuple[Tuple[int, int, int], Tuple[int, int, int]]],
    blue_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]],
):
    """Detect red and blue markers in an RGB image.

    Returns a dict with keys 'red' and 'blue' containing (u, v) pixel coordinates
    (u=column, v=row). Raises ValueError when detection fails.
    """
    if rgb_img is None:
        raise ValueError("No image")

    hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

    red_mask = None
    for low, high in red_ranges:
        l = np.array(low, dtype=np.uint8)
        h = np.array(high, dtype=np.uint8)
        m = cv2.inRange(hsv, l, h)
        red_mask = m if red_mask is None else cv2.bitwise_or(red_mask, m)

    bl_low = np.array(blue_range[0], dtype=np.uint8)
    bl_high = np.array(blue_range[1], dtype=np.uint8)
    blue_mask = cv2.inRange(hsv, bl_low, bl_high)

    # Apply small morphological filtering to reduce noise and fill holes
    try:
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
    except Exception:
        # If morphological ops fail for any reason, continue without them
        pass

    # Debug: counts for masks (helps diagnose why detection may fail)
    try:
        red_count = int(np.count_nonzero(red_mask))
        blue_count = int(np.count_nonzero(blue_mask))
        print(f"detect_markers: red_count={red_count}, blue_count={blue_count}")
    except Exception:
        pass

    def center_from_mask(mask: np.ndarray):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        return float(cx), float(cy)

    red_center = center_from_mask(red_mask)
    blue_center = center_from_mask(blue_mask)

    if red_center is None or blue_center is None:
        raise ValueError("Markers not detected")

    return {"red": red_center, "blue": blue_center}


# Diagnostic mask computation and disk-dump helpers removed to simplify the node.


if HAS_ROS:  # only define ROS node when rclpy is present
    class TopDownMarkerTF(Node):
        def __init__(self):
            super().__init__("top_down_marker_tf")
            # Parameters
            self.declare_parameter("camera_topic", "/camera/image_raw")
            self.declare_parameter("camera_info_topic", "/camera/camera_info")
            self.declare_parameter("camera_frame", "map")
            self.declare_parameter("base_frame", "base_link")
            self.declare_parameter("camera_height", 10.13)
            self.declare_parameter("camera_info_timeout", 2.0)
            self.declare_parameter("hold_last_pose", True)
            self.declare_parameter("smoothing_alpha", 0.0)
            self.declare_parameter("publish_pose", True)
            # debug parameter and debug image publishing removed for simplicity
            # Allow fixing axis sign mismatches between image coords and world
            self.declare_parameter("invert_y", True)
            # Logging throttles
            self.declare_parameter("detection_log_interval", 1.0)

            # HSV ranges (two for red to handle hue wrap)
            self.declare_parameter("red_hsv_low1", [0, 100, 100])
            self.declare_parameter("red_hsv_high1", [10, 255, 255])
            self.declare_parameter("red_hsv_low2", [160, 100, 100])
            self.declare_parameter("red_hsv_high2", [179, 255, 255])

            self.declare_parameter("blue_hsv_low", [100, 150, 50])
            self.declare_parameter("blue_hsv_high", [140, 255, 255])

            # Intrinsics
            self.fx = None
            self.fy = None
            self.cx = None
            self.cy = None

            # Load parameters
            self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
            self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
            self.camera_height = float(self.get_parameter("camera_height").get_parameter_value().double_value)
            self.camera_info_timeout = float(self.get_parameter("camera_info_timeout").get_parameter_value().double_value)
            self.hold_last_pose = bool(self.get_parameter("hold_last_pose").get_parameter_value().bool_value)
            self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").get_parameter_value().double_value)
            self.publish_pose = bool(self.get_parameter("publish_pose").get_parameter_value().bool_value)
            self.invert_y = bool(self.get_parameter("invert_y").get_parameter_value().bool_value)
        
            # last time we logged a detection (seconds since epoch)
            self._last_detection_log_time = 0.0

            # HSV values
            self.red_ranges = [
                (tuple(self.get_parameter("red_hsv_low1").get_parameter_value().integer_array_value),
                 tuple(self.get_parameter("red_hsv_high1").get_parameter_value().integer_array_value)),
                (tuple(self.get_parameter("red_hsv_low2").get_parameter_value().integer_array_value),
                 tuple(self.get_parameter("red_hsv_high2").get_parameter_value().integer_array_value)),
            ]
            self.blue_range = (
                tuple(self.get_parameter("blue_hsv_low").get_parameter_value().integer_array_value),
                tuple(self.get_parameter("blue_hsv_high").get_parameter_value().integer_array_value),
            )

            self.bridge = CvBridge()
            self.br = TransformBroadcaster(self)

            # Publishers
            # Debug image topic removed; keep pose publisher if requested
            if self.publish_pose:
                self.pose_pub = self.create_publisher(PoseStamped, "/detected_pose", 1)

            # Subscribers
            camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
            self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
            self.sub = self.create_subscription(Image, camera_topic, self._image_cb, 10)

            # Wait for CameraInfo up to timeout then fallback
            self.camera_info_got = False
            self._start_time = self.get_clock().now()
            self.create_timer(0.1, self._camera_info_wait_timer)
            # Periodically re-publish last known pose (if requested) so TF remains available
            self.create_timer(0.2, self._hold_pose_timer)

            # (diagnostic mask dumping removed)

            # last pose for smoothing / hold
            self._last_pose = None

            self.get_logger().info("Top-down marker TF node started")
            self.get_logger().info(f"invert_y={self.invert_y}")

        def _camera_info_cb(self, msg: CameraInfo):
            if self.fx is None:
                self.fx = float(msg.k[0])
                self.fy = float(msg.k[4])
                self.cx = float(msg.k[2])
                self.cy = float(msg.k[5])
                self.camera_info_got = True
                self.get_logger().info("CameraInfo received")

        def _camera_info_wait_timer(self):
            if self.camera_info_got:
                return
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            if elapsed >= self.camera_info_timeout:
                # fallback defaults (user requested values)
                self.fx = self.fx or 201.4
                self.fy = self.fy or 201.4
                self.cx = self.cx or 240.0
                self.cy = self.cy or 240.0
                self.get_logger().warn("Using fallback intrinsics")
                self.camera_info_got = True

        def _image_cb(self, msg: Image):
            if self.fx is None:
                self.get_logger().debug("No intrinsics yet; skipping image")
                return
            # Log image encoding once for debugging color order issues
            if not hasattr(self, '_image_encoding_logged'):
                try:
                    self.get_logger().info(f"Image encoding: {msg.encoding}")
                except Exception:
                    pass
                self._image_encoding_logged = True
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            except Exception:
                try:
                    cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
                    if msg.encoding in ("bgr8", "bgr"):
                        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                except Exception as exc:
                    self.get_logger().error(f"Image conversion failed: {exc}")
                    return

            try:
                centers = detect_markers(cv_img, self.red_ranges, self.blue_range)
            except ValueError:
                self.get_logger().debug("Markers not detected in image")
                # If requested, keep publishing last known pose so TF stays available
                if self._last_pose and self.hold_last_pose:
                    lx, ly, lyaw = self._last_pose
                    self._publish_transform_and_pose(lx, ly, lyaw)
                else:
                    if not self.hold_last_pose:
                        self._last_pose = None
                return

            u_red, v_red = centers['red']
            u_blue, v_blue = centers['blue']

            x_red = (u_red - self.cx) * (self.camera_height) / self.fx
            y_red = (v_red - self.cy) * (self.camera_height) / self.fy
            x_blue = (u_blue - self.cx) * (self.camera_height) / self.fx
            y_blue = (v_blue - self.cy) * (self.camera_height) / self.fy

            # Some camera/image coordinate systems require the lateral axis to be inverted
            # to match the world frame orientation (image v increases downward).
            if getattr(self, "invert_y", False):
                y_red = -y_red
                y_blue = -y_blue

            x = (x_red + x_blue) / 2.0
            y = (y_red + y_blue) / 2.0
            yaw = math.atan2(y_red - y_blue, x_red - x_blue)

            # smoothing
            if self._last_pose and (self.smoothing_alpha > 0.0):
                la = self.smoothing_alpha
                x = la * x + (1.0 - la) * self._last_pose[0]
                y = la * y + (1.0 - la) * self._last_pose[1]
                dy = yaw - self._last_pose[2]
                dy = math.atan2(math.sin(dy), math.cos(dy))
                yaw = self._last_pose[2] + la * dy

            self._last_pose = (x, y, yaw)
            # Throttle detection logging to avoid spamming logs every frame
            interval = float(self.get_parameter("detection_log_interval").get_parameter_value().double_value)
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_detection_log_time >= interval:
                self._last_detection_log_time = now
                self.get_logger().info(f"Markers detected: red=({u_red:.1f},{v_red:.1f}) blue=({u_blue:.1f},{v_blue:.1f})")

            # Publish TF and Pose
            t = self._publish_transform_and_pose(x, y, yaw)

            # Debug image publishing removed to keep runtime simple

        def _publish_transform_and_pose(self, x: float, y: float, yaw: float):
            """Publish map->base_link transform and optional PoseStamped, return TransformStamped."""
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.camera_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(x)
            t.transform.translation.y = float(y)
            t.transform.translation.z = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.br.sendTransform(t)

            if self.publish_pose:
                ps = PoseStamped()
                ps.header.stamp = t.header.stamp
                ps.header.frame_id = self.camera_frame
                ps.pose.position.x = float(x)
                ps.pose.position.y = float(y)
                ps.pose.position.z = 0.0
                ps.pose.orientation = t.transform.rotation
                self.pose_pub.publish(ps)

            return t

        def _hold_pose_timer(self):
            """Republish the last known pose periodically when hold_last_pose is enabled."""
            if (self._last_pose is None) or (not self.hold_last_pose):
                return
            x, y, yaw = self._last_pose
            self._publish_transform_and_pose(x, y, yaw)


def main(args=None):
    if not HAS_ROS:
        raise RuntimeError("ROS python libraries (rclpy) are not available in this environment")
    rclpy.init(args=args)
    node = TopDownMarkerTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
