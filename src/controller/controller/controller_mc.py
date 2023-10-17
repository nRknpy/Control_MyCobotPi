import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mycobot_msg.msg import MyCobotMsg
from pymycobot import MyCobot, PI_PORT, PI_BAUD
import time
import atexit


class ControllerMc(Node):
    def __init__(self):
        super().__init__('controller_mc')

        self.mc = MyCobot(PI_PORT, PI_BAUD)
        self.angles = self.mc.get_angles()

        self.speed = 100
        self.gripper_value = 0

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.listener = self.create_subscription(
            MyCobotMsg, '/radians', self.on_subscribe)

        self.publisher = self.create_publisher(MyCobotMsg, '/mc_joints', qos)

    def on_subscribe(self, msg):
        radians = msg.joints
        gripper = msg.gripper
        self.mc.send_radians(radians, self.speed)
        self.mc.set_gripper_value(gripper, 20)
        # time.sleep(0.1)

        self.angles = self.mc.get_angles()
        self.gripper_value = self.mc.get_gripper_value()

        pub_msg = MyCobotMsg()
        # pub_msg.header.stamp = self.get_clock().now().to_msg()
        # pub_msg.joints = self.mc.get_radians()
        pub_msg.joints = self.angles
        pub_msg.gripper = self.mc.get_gripper_value()
        self.publisher.publish(pub_msg)

        print(f'radians; {radians}, gripper: {gripper}')
        self.get_logger().info(f'radians; {radians}, gripper: {gripper}')


def end(node: ControllerMc):
    node.mc.release_all_servos()
    node.mc.set_gripper_state(254, 10)


def main():
    rclpy.init()
    node = ControllerMc()
    node.mc.set_gripper_calibration()
    atexit.register(end, node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
