import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from mycobot_msg.msg import MyCobotMsg


class JointCoordListener(Node):
    def __init__(self):
        super().__init__('joint_coord_listener')

        self.joints = []
        self.imgs = []

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.coords_sub = Subscriber(
            self, MyCobotMsg, '/mc_coords', qos_profile=qos)
        self.joint_sub = Subscriber(
            self, MyCobotMsg, '/mc_joints', qos_profile=qos)
        self.radian_sub = Subscriber(
            self, MyCobotMsg, '/mc_radian', qos_profile=qos)

        queue_size = 30

        self.ts = ApproximateTimeSynchronizer(
            [self.coords_sub, self.joint_sub, self.radian_sub],
            queue_size,
            0.01,
            allow_headerless=True
        )
        self.ts.registerCallback(self.callback)

    def callback(self, coords_msg, joint_msg, radian_msg):
        self.get_logger().info(
            f'joint: {np.array(joint_msg.joints)}\ncoords: {np.array(coords_msg.joints)}\nradian: {np.array(radian_msg.joints)}')


def main(args=None):
    rclpy.init(args=args)
    node = JointCoordListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
