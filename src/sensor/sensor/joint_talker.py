import rclpy
from rclpy.node import Node
from mycobot_msg.msg import MyCobotMsg
from pymycobot import MyCobot, PI_PORT, PI_BAUD


class JointTalker(Node):
    def __init__(self):
        super().__init__('joint_talker')

        self.mc = MyCobot(PI_PORT, PI_BAUD)

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(MyCobotMsg, '/mc_joints', qos)
        self.timer = self.create_timer(0.1, self.callback)

    def callback(self):
        msg = MyCobotMsg()
        msg.joints = self.mc.get_angles()
        msg.gripper = self.mc.get_gripper_value()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
