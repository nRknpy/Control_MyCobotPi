import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mycobot_msg.msg import MyCobotMsg
from .NLinkArm import NLinkArm
import math
import time


class ControllerSim(Node):
    def __init__(self):
        super().__init__('controller_sim')

        self.sim = NLinkArm([
            [0., math.pi / 2, 0, 0.13156],
            [0., 0., -0.1104, 0.],
            [0., 0., -0.096, 0.],
            [0., math.pi / 2, 0., 0.06639],
            [0., -math.pi / 2, 0., 0.07318],
            [0., 0., 0., 0.0436]
        ])

        # self.mc = MyCobot(PI_PORT, PI_BAUD)
        # self.angles = self.mc.get_radians()

        self.angles = self.sim.convert_joint_angles_mc_to_sim(
            [0.024, 2.172, -2.6, -0.604, -0.143, 0.928])
        self.sim.send_angles(self.angles)
        self.coords = self.sim.forward_kinematics()
        self.gripper_value = 0

        self.delta = 0.05

        self.listener = self.create_subscription(
            Joy, 'joy', self.on_subscribe, 10)

        self.publisher = self.create_publisher(MyCobotMsg, '/radians')

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
        self.get_logger().info(f'{action}')

        # print(self.mc.get_radians())
        # print(self.coords)

        if action == 'stop':
            # self.sim.send_angles(self.angles)
            mc_angles = self.sim.convert_joint_angles_sim_to_mc(self.angles)

        elif action == 'move_to_default':
            # self.mc.send_angles([-0.17, -10, -133.24, 60.99, 0.17, 50.36], self.speed)
            self.sim.send_angles(self.sim.convert_joint_angles_mc_to_sim(
                [0.052, -0.534, -1.934, 0.923, -0.031, -2.428]))
            self.angles = self.sim.get_angles()
            mc_angles = self.sim.convert_joint_angles_sim_to_mc(self.angles)
            self.coords = self.sim.forward_kinematics()
        elif action == 'move_to_home':
            self.sim.send_angles(self.sim.convert_joint_angles_mc_to_sim(
                [0.024, 2.172, -2.6, -0.604, -0.143, 0.928]))
            self.angles = self.sim.get_angles()
            mc_angles = self.sim.convert_joint_angles_sim_to_mc(self.angles)
            self.coords = self.sim.forward_kinematics()

        elif action == 'grip_on':
            self.gripper_value = 100
            mc_angles = self.sim.convert_joint_angles_sim_to_mc(self.angles)
        elif action == 'grip_off':
            self.gripper_value = 0
            mc_angles = self.sim.convert_joint_angles_sim_to_mc(self.angles)

        elif action == 'x-':
            cache = self.coords.copy()
            self.coords[1] -= self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        elif action == 'x+':
            cache = self.coords.copy()
            self.coords[1] += self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        elif action == 'y+':
            cache = self.coords.copy()
            self.coords[0] += self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        elif action == 'y-':
            cache = self.coords.copy()
            self.coords[0] -= self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        elif action == 'z+':
            cache = self.coords.copy()
            self.coords[2] += self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        elif action == 'z-':
            cache = self.coords.copy()
            self.coords[2] -= self.delta
            if self.validate_coords(self.coords):
                self.angles = self.sim.inverse_kinematics(self.coords)
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
                self.coords = self.sim.forward_kinematics()
            else:
                self.coords = cache
                mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                    self.angles)
        # elif action == 'rx+':
        #     self.coords[3] += 2
        #     self.mc.send_radians(self.angles, self.speed)
        #     self.angles = self.mc.get_radians()
        # elif action == 'rx-':
        #     self.coords[3] -= 2
        #     self.mc.send_radians(self.angles, self.speed)
        #     self.angles = self.mc.get_radians()
        else:
            mc_angles = mc_angles = self.sim.convert_joint_angles_sim_to_mc(
                self.angles)

        pub_msg = MyCobotMsg()
        pub_msg.joints = mc_angles
        pub_msg.gripper = self.gripper_value

        self.publisher.publish(pub_msg)

        self.sim.plot(realtime=True)

    @staticmethod
    def validate_coords(coords):
        print(coords[:3])
        if not (coords[2] >= 0. and math.dist([0, 0, 0], coords[:3]) <= 0.37):
            return False
        return True


def main():
    rclpy.init()
    node = ControllerSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
