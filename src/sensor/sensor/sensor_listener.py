import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from mycobot_msg.msg import MyCobotMsg
from cv_bridge import CvBridge
import cv2


class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.img_sub = Subscriber(
            self, Image, '/color/image_raw', qos_profile=qos)
        self.joint_sub = Subscriber(
            self, MyCobotMsg, '/mc_joints', qos_profile=qos)

        queue_size = 30

        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.joint_sub],
            queue_size,
            0.01,
        )
        self.ts.registerCallback(self.callback)

    def callback(self, img_msg, joint_msg):
        self.get_logger().info(f'{joint_msg.joints}, {joint_msg.gripper}')
        img = CvBridge().imgmsg_to_cv2(img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        cv2.imshow('Image', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
