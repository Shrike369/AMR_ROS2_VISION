"""
Camera bridge node.

Subscribes to the Gazebo image topic (bridged into ROS by ``ros_gz_bridge``)
and re-publishes it on a conventional ROS 2 image topic (``/camera/image_raw``).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class CameraBridge(Node): 
    def __init__(self):
        super().__init__('camera_bridge')
        # Topic published by ros_gz_bridge after bridging from Gazebo
        self.src_topic = (
            '/world/empty/model/camera_bot/link/base_link/sensor/top_camera/image'
        )
        # Standard ROS 2 camera topic we want to provide
        self.dst_topic = '/camera/image_raw'
        # Also forward camera_info to a standard topic
        self.src_info_topic = (
            '/world/empty/model/camera_bot/link/base_link/sensor/top_camera/camera_info'
        )
        self.dst_info_topic = '/camera/camera_info'

        self.pub = self.create_publisher(Image, self.dst_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.dst_info_topic, 10)
        self.sub = self.create_subscription(Image, self.src_topic, self.callback, 10)
        self.sub_info = self.create_subscription(
            CameraInfo, self.src_info_topic, self.callback_info, 10
        )
        self.get_logger().info(
            f"Subscribed to '{self.src_topic}' -> '{self.dst_topic}', "
            f"and '{self.src_info_topic}' -> '{self.dst_info_topic}'"
        )

    def callback(self, msg: Image) -> None:
        # Forward message as-is. Could add header/timestamp adjustment here if needed.
        self.pub.publish(msg)

    def callback_info(self, msg: CameraInfo) -> None:
        # Forward CameraInfo as-is
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
