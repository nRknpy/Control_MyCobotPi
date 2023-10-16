import numpy as np
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from mycobot_msg.msg import MyCobotMsg
from NLinkArm import NLinkArm
import math


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

        self.sim = NLinkArm([
            [0., math.pi / 2, 0, 0.13156],
            [0., 0., -0.1104, 0.],
            [0., 0., -0.096, 0.],
            [0., math.pi / 2, 0., 0.06639],
            [0., -math.pi / 2, 0., 0.07318],
            [0., 0., 0., 0.0436]
        ])

    def callback(self, coords_msg, joint_msg, radian_msg):
        self.get_logger().info(
            f'joint: {np.array(joint_msg.joints)}\ncoords: {np.array(coords_msg.joints)}\nradian: {np.array(radian_msg.joints)}')
        self.sim.send_angles(
            self.sim.convert_joint_angles_mc_to_sim(radian_msg.joints))
        coords = self.sim.forward_kinematics()

        pred_angles = self.sim.inverse_kinematics(coords)
        self.sim.send_angles(pred_angles)
        # self.sim.forward_kinematics()

        self.sim.plot(realtime=True)


def main(args=None):
    rclpy.init(args=args)
    node = JointCoordListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
