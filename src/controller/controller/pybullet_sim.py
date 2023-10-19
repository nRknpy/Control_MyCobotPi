import rclpy
from rclpy.node import Node
from mycobot_msg.msg import MyCobotMsg
import pybullet as p
import math


class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim')

        self.pub = self.create_publisher(MyCobotMsg, '/radians', 10)
        self.timer = self.create_timer(0.02, self.run)

        clid = p.connect(p.GUI)
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.mycobot_id = p.loadURDF("./urdf/mycobot_npaka.urdf",
                                     start_pos, start_orientation)

        default_pos = [0., 0.052, -0.534, -1.934, 0.923, -0.031, -2.428]
        for i in range(6):
            p.resetJointState(self.mycobot_id, i, default_pos[i])
        self.t = 0
        self.hasPrevPose = 0
        self.prevPose = None
        self.prevPose1 = None

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

    def run(self):
        self.t = self.t + 0.01

        p.stepSimulation()

        for i in range(1):
            pos = [0.15 * math.cos(4*self.t), 0.2, 0.2 +
                   0.1 * math.sin(4*self.t)]
            orn = p.getQuaternionFromEuler(
                [-math.pi / 2.3, math.pi / 8., -math.pi / 4.])

            joint_angles = p.calculateInverseKinematics(self.mycobot_id,
                                                        6,
                                                        pos,
                                                        orn)
            # print('joint_angles:', joint_angles)
            for i in range(6):
                p.setJointMotorControl2(bodyIndex=self.mycobot_id,
                                        jointIndex=i + 1,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=joint_angles[i],
                                        targetVelocity=0,
                                        force=500,
                                        positionGain=0.2,
                                        velocityGain=1.5)
        ls = p.getLinkState(self.mycobot_id, 6)
        radians = []
        for i in range(6):
            radians.append(p.getJointState(self.mycobot_id, i+1)[0])
        print(radians)
        if (self.hasPrevPose):
            p.addUserDebugLine(self.prevPose, pos, [0, 0, 0.3], 1, 15)
            p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, 15)
        self.prevPose = pos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1

        if pos[2] < 0.2:
            gripper = 0
        else:
            gripper = 100

        msg = MyCobotMsg()
        msg.joints = radians
        msg.gripper = gripper
        self.pub.publish(msg)
        # print(ls)


def main():
    rclpy.init()
    node = PyBulletSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
