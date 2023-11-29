import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mycobot_msg.msg import MyCobotMsg
import pymycobot
from pymycobot import MyCobot, PI_PORT, PI_BAUD
import time
import atexit


class ControllerMc(Node):
    def __init__(self):
        super().__init__('controller_mc')
        self.get_logger().info(f"{pymycobot.__version__}")

        self.mc = MyCobot(PI_PORT, PI_BAUD)
        self.angles = self.mc.get_angles()

        self.speed = 100
        self.gripper_value = 0
        self.mc.set_encoder(7, 4000, 100)

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.listener = self.create_subscription(
            MyCobotMsg, '/radians', self.on_subscribe, 10)

        self.publisher = self.create_publisher(MyCobotMsg, '/mc_joints', qos)

    def on_subscribe(self, msg):
        radians = msg.joints
        gripper = msg.gripper
        self.mc.send_radians(radians, self.speed)
        if gripper > 50:
            # gripper = 3000
            gripper = 0
        else:
            # gripper = 5
            gripper = 1
        # self.mc.set_encoder(7, gripper, 100)
        self.mc.set_gripper_state(gripper, 0)
        # time.sleep(0.1)

        # while True:
        #     try:
        #         self.angles = self.mc.get_angles()
        #     except:
        #         self.get_logger().info('except' * 10)
        #     else:
        #         if self.angles:
        #             break
        #         else:
        #             self.get_logger().info('failed!' * 10)
        self.angles = radians
        # self.gripper_value = self.mc.get_gripper_value()
        self.gripper_value = gripper

        pub_msg = MyCobotMsg()
        pub_msg.joints = self.angles
        pub_msg.gripper = self.gripper_value
        self.publisher.publish(pub_msg)

        print(f'radians; {self.angles}, gripper: {self.gripper_value}')
        self.get_logger().info(
            f'radians; {self.angles}, gripper: {self.gripper_value}')


def main():
    rclpy.init()
    node = ControllerMc()
    node.mc.set_gripper_calibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
