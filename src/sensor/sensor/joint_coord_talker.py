import rclpy
from rclpy.node import Node
from mycobot_msg.msg import MyCobotMsg
from pymycobot import MyCobot, PI_PORT, PI_BAUD


class JointCoordTalker(Node):
    def __init__(self):
        super().__init__('joint_coord_talker')

        self.mc = MyCobot(PI_PORT, PI_BAUD)
        self.mc.release_all_servos()

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(MyCobotMsg, '/mc_joints', qos)

        self.joint_pub = self.create_publisher(MyCobotMsg, '/mc_joints', qos)
        self.coord_pub = self.create_publisher(MyCobotMsg, '/mc_coords', qos)
        self.radian_pub = self.create_publisher(MyCobotMsg, '/mc_radian', qos)

        self.timer = self.create_timer(0.5, self.callback)

    def callback(self):
        joints = self.mc.get_angles()
        coords = self.mc.get_coords()
        radian = self.mc.get_radians()
        gripper = self.mc.set_gripper_value()

        joints_msg = MyCobotMsg()
        coords_msg = MyCobotMsg()
        radian_msg = MyCobotMsg()

        joints_msg.joints = joints
        joints_msg.gripper = gripper
        coords_msg.joints = coords
        coords_msg.gripper = gripper
        radian_msg.joints = radian
        radian_msg.gripper = gripper

        self.joint_pub.publish(joints_msg)
        self.coord_pub.publish(coords_msg)
        self.radian_pub.publish(radian_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointCoordTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
