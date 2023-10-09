import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageListener(Node):
    def __init__(self):
        super().__init__('image_listener')

        video_qos = rclpy.qos.QoSProfile(depth=10)
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.listener = self.create_subscription(
            Image, '/color/image_raw', self.on_subscribe, video_qos)

    def on_subscribe(self, msg):
        img = CvBridge().imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow('Image', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
