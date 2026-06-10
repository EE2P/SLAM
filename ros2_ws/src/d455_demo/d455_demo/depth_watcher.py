import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class DepthWatcher(Node):
    """订阅 D455 深度图，每秒打印一次画面中心点的距离。"""

    def __init__(self):
        super().__init__("depth_watcher")
        self.sub = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.on_depth, 10
        )
        self.last_log = self.get_clock().now()

    def on_depth(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds < 1e9:
            return
        self.last_log = now
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        center_mm = int(depth[msg.height // 2, msg.width // 2])
        self.get_logger().info(
            f"{msg.width}x{msg.height} center depth = {center_mm} mm"
        )


def main():
    rclpy.init()
    node = DepthWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
