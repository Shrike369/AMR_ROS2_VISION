import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image

import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


class ImageToOccupancyGrid(Node):

    def __init__(self):
        super().__init__('image_to_occupancy')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Global map publisher
        # QoS: Nav2 expects TRANSIENT_LOCAL durability for the map topic
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Global map publisher (transient-local so Nav2 can latch the map)
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            qos
        )

        # Local map publisher (also transient-local to be discoverable by Nav2 components)
        self.local_map_pub = self.create_publisher(
            OccupancyGrid,
            '/local_map',
            qos
        )

        # --- Map parameters ---
        self.image_width = 480
        self.image_height = 480
        self.resolution = 0.05  # meters per cell

        self.map_size_m = self.image_width * self.resolution

        self.get_logger().info('Optimized image->occupancy node started')

    def image_callback(self, msg: Image):

        # Convert ROS Image → OpenCV (try rgb8, fall back to msg.encoding)
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            except Exception as exc:  # pragma: no cover - runtime error
                self.get_logger().error(f'Image conversion failed: {exc}')
                return

        # Convert to uint8 numpy array
        img = cv_img.astype(np.uint8)

        # Handle dynamic image sizes
        h, w = img.shape[:2]
        if (h != self.image_height) or (w != self.image_width):
            self.image_height = int(h)
            self.image_width = int(w)
            self.map_size_m = self.image_width * self.resolution
            self.get_logger().info(f'Image size changed to {w}x{h}, map_size_m={self.map_size_m:.2f} m')


        # Initialize grid as UNKNOWN (-1)
        grid = np.full((self.image_height, self.image_width), -1, dtype=np.int8)

        # BLACK => FREE
        grid[(img == [0, 0, 0]).all(axis=2)] = 0

        # OCCUPIED COLORS
        occupied = [
            (100, 100, 100),
            (120, 120, 120),
            (177, 177, 177),
            (178, 178, 178),
            (217, 217, 217),
            (216, 216, 216),
            (121, 121, 121),
            (219, 219, 219),
            (118, 118, 118),
            (179, 179, 179),
            (215, 215, 215),
            (218, 218, 218),
            (119, 119, 119),
            (198, 198, 198),
            (123, 123, 123),
            (164, 164, 164),
            (199, 199, 199),
            (122, 122, 122),
            (165, 165, 165),
            (200, 200, 200),
            (122, 122, 122),
            (166, 166, 166),
            (201, 201, 201),
            (124, 124, 124),
            (167, 167, 167),
            (202, 202, 202),
            (125, 125, 125),
            (168, 168, 168),
            (203, 203, 203),
            (126, 126, 126),
            (169, 169, 169),
            (204, 204, 204),
            (127, 127, 127),
            (170, 170, 170),
            (205, 205, 205),
            (128, 128, 128),
            (171, 171, 171),
            (206, 206, 206),
            (129, 129, 129),
            (172, 172, 172),
            (207, 207, 207),
            (130, 130, 130),
            (173, 173, 173),
        ]
        for color in occupied:
            grid[(img == color).all(axis=2)] = 100
        
        # Flip Y-axis (image -> map coordinates)
        grid = np.flipud(grid)

        # Create OccupancyGrid message
        og = OccupancyGrid()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'

        og.info.resolution = float(self.resolution)
        og.info.width = int(self.image_width)
        og.info.height = int(self.image_height)

        # Map origin (bottom-left)
        og.info.origin.position.x = -self.map_size_m / 2.0
        og.info.origin.position.y = -self.map_size_m / 2.0
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0

        # Ensure plain Python ints for ROS sequence
        og.data = [int(x) for x in grid.flatten()]

        # Publish both maps
        self.map_pub.publish(og)
        self.local_map_pub.publish(og)


def main():
    rclpy.init()
    node = ImageToOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
