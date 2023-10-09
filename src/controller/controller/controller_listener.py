import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mycobot_msg.msg import MyCobotMsg
from pymycobot import MyCobot, PI_PORT, PI_BAUD
import time
import atexit


class ControllerListener(Node):
    def __init__(self):
        super().__init__('controller_listener')

        self.mc = MyCobot(PI_PORT, PI_BAUD)
        self.angles = self.mc.get_angles()
        self.coords = self.mc.get_coords()

        self.speed = 80
        self.gripper_value = 0
        self.prev_action = 'stop'

        self.listener = self.create_subscription(
            Joy, 'joy', self.on_subscribe, 10)

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.publisher = self.create_publisher(MyCobotMsg, '/mc_joints', qos)

    def joy2action(self, joy):
        if joy.buttons[5]:
            return 'move_to_default'
        if joy.buttons[10]:
            return 'move_to_home'
        if joy.buttons[0]:
            return 'grip_off'
        if joy.buttons[1]:
            return 'grip_on'
        if joy.axes[0] > 0.2 and -joy.axes[0] < joy.axes[1] < joy.axes[0]:
            return 'x+'
        if joy.axes[0] < -0.2 and joy.axes[0] < joy.axes[1] < -joy.axes[0]:
            return 'x-'
        if joy.axes[1] > 0.2 and -joy.axes[1] < joy.axes[0] < joy.axes[1]:
            return 'y+'
        if joy.axes[1] < -0.2 and joy.axes[1] < joy.axes[0] < -joy.axes[1]:
            return 'y-'
        if joy.axes[4] > 0.2 and -joy.axes[4] < joy.axes[3] < joy.axes[4]:
            return 'z+'
        if joy.axes[4] < -0.2 and joy.axes[4] < joy.axes[3] < -joy.axes[4]:
            return 'z-'
        if joy.axes[3] > 0.2 and -joy.axes[3] < joy.axes[4] < joy.axes[3]:
            return 'rx+'
        if joy.axes[3] < -0.2 and joy.axes[3] < joy.axes[4] < -joy.axes[3]:
            return 'rx-'
        # if joy.axes[0] > 0.2:
        #     return 'x+'
        # if joy.axes[0] < -0.2:
        #     return 'x-'
        return 'stop'

    def on_subscribe(self, msg):
        action = self.joy2action(msg)
        # self.get_logger().info(f'{action}')

        pub_msg = MyCobotMsg()
        # pub_msg.header.stamp = self.get_clock().now().to_msg()
        pub_msg.joints = self.mc.get_angles()
        pub_msg.gripper = self.mc.get_gripper_value()

        self.publisher.publish(pub_msg)

        print(self.mc.get_angles())

        if action == 'stop':
            self.mc.send_angles(self.angles, self.speed)
            self.prev_action = 'stop'

        elif action == 'move_to_default':
            self.prev_action = action
            # self.mc.send_angles([-0.17, -10, -133.24, 60.99, 0.17, 50.36], self.speed)
            self.mc.send_coords(
                [186.9, -51.0, 116.6, -177.44, 20, -136.52], self.speed)
            time.sleep(3)
            self.angles = self.mc.get_angles()
            self.coords = self.mc.get_coords()
        elif action == 'move_to_home':
            self.prev_action = action
            self.mc.sync_send_angles(
                [1.49, 123.48, -148.09, -32.78, 1.84, 55.45], 70)
            time.sleep(3)
            self.angles = self.mc.get_angles()
            self.coords = self.mc.get_coords()

        elif action == 'grip_on':
            # self.mc.set_gripper_state(1, self.speed)
            self.mc.set_gripper_value(100, self.speed)
        elif action == 'grip_off':
            # self.mc.set_gripper_state(0, self.speed)
            self.mc.set_gripper_value(0, self.speed)

        elif action == 'x-':
            self.prev_action = action
            # self.angles[0] += 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[1] -= 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        elif action == 'x+':
            self.prev_action = action
            # self.angles[0] -= 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[1] += 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        elif action == 'y+':
            self.prev_action = action
            # self.angles[0] += 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[0] += 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        elif action == 'y-':
            self.prev_action = action
            # self.angles[0] += 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[0] -= 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        elif action == 'z+':
            self.prev_action = action
            # self.angles[0] += 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[2] += 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        elif action == 'z-':
            self.prev_action = action
            # self.angles[0] += 2
            # self.mc.send_angles(self.angles, self.speed)
            self.coords[2] -= 4
            self.mc.send_coords(self.coords, self.speed)
            self.angles = self.mc.get_angles()
        # elif action == 'rx+':
        #     self.coords[3] += 2
        #     self.mc.send_coords(self.angles, self.speed)
        #     self.angles = self.mc.get_angles()
        # elif action == 'rx-':
        #     self.coords[3] -= 2
        #     self.mc.send_coords(self.angles, self.speed)
        #     self.angles = self.mc.get_angles()


def end(node: ControllerListener):
    node.mc.release_all_servos()
    node.mc.set_gripper_state(10, 10)


def main():
    rclpy.init()
    node = ControllerListener()
    node.mc.set_gripper_calibration()
    atexit.register(end, node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
