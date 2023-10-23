import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mycobot_msg.msg import MyCobotMsg
import pandas
import pybullet as p
import math
from datetime import datetime


class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim')

        self.listener = self.create_subscription(
            Joy, 'joy', self.on_subscribe, 10)

        self.pub = self.create_publisher(MyCobotMsg, '/radians', 10)
        # self.timer = self.create_timer(0.02, self.run)

        clid = p.connect(p.GUI)
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.mycobot_id = p.loadURDF("./urdf/mycobot_npaka.urdf",
                                     start_pos, start_orientation)

        # self.orn = p.getQuaternionFromEuler(
        #     [-math.pi / 2.3, math.pi / 8., -math.pi / 4.])
        self.orn = p.getQuaternionFromEuler(
            [-math.pi / 2, math.pi / 16, -math.pi / 4.])
        self.default_pos = [-0.012, -0.331, -2.419, 1.239, 0.032, 0.747]
        self.home_pos = [-0.005, 1.916, -2.655, -0.561, -0.006, 0.745]
        for i in range(6):
            p.resetJointState(self.mycobot_id, i+1, self.default_pos[i])
        self.angles = self.default_pos
        self.pos = list(p.getLinkState(self.mycobot_id, 6)[4])
        self.r = -math.pi / 4.
        self.gripper_value = 100

        self.delta = 0.01
        self.prev_time = datetime.now()

        self.t = 0
        self.hasPrevPose = 0
        self.prevPose = None
        self.prevPose1 = None

    def joy2action(self, joy):
        if joy.buttons[5]:
            return 'move_to_default', False
        if joy.buttons[10]:
            return 'move_to_home', False
        if joy.buttons[0]:
            return 'grip_off', False
        if joy.buttons[1]:
            return 'grip_on', False
        if joy.axes[0] > 0.2 and -joy.axes[0] < joy.axes[1] < joy.axes[0]:
            if joy.axes[0] > 0.5:
                return 'x-', True
            return 'x-', False
        if joy.axes[0] < -0.2 and joy.axes[0] < joy.axes[1] < -joy.axes[0]:
            if joy.axes[0] < -0.5:
                return 'x+', True
            return 'x+', False
        if joy.axes[1] > 0.2 and -joy.axes[1] < joy.axes[0] < joy.axes[1]:
            if joy.axes[1] > 0.5:
                return 'y+', True
            return 'y+', False
        if joy.axes[1] < -0.2 and joy.axes[1] < joy.axes[0] < -joy.axes[1]:
            if joy.axes[1] < -0.5:
                return 'y-', True
            return 'y-', False
        if joy.axes[4] > 0.2 and -joy.axes[4] < joy.axes[3] < joy.axes[4]:
            if joy.axes[4] > 0.5:
                return 'z+', True
            return 'z+', False
        if joy.axes[4] < -0.2 and joy.axes[4] < joy.axes[3] < -joy.axes[4]:
            if joy.axes[4] < -0.5:
                return 'z-', True
            return 'z-', False
        # if joy.axes[3] > 0.2 and -joy.axes[3] < joy.axes[4] < joy.axes[3]:
        #     return 'r+'
        # if joy.axes[3] < -0.2 and joy.axes[3] < joy.axes[4] < -joy.axes[3]:
        #     return 'r-'
        return 'stop', False

    def on_subscribe(self, msg):
        now = datetime.now()
        d = (now - self.prev_time).microseconds * 1e-3
        self.prev_time = now

        action, fast = self.joy2action(msg)

        delta = self.delta * d / 10.
        if not fast:
            delta *= 0.2

        p.stepSimulation()

        # self.get_logger().info(f'{action}')
        # print(self.pos)

        if action == 'stop':
            for i in range(6):
                p.setJointMotorControl2(bodyIndex=self.mycobot_id,
                                        jointIndex=i + 1,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.angles[i],
                                        targetVelocity=0,
                                        force=500,
                                        positionGain=0.2,
                                        velocityGain=1.5)
            self.pos = list(p.getLinkState(self.mycobot_id, 6)[4])
            msg = MyCobotMsg()
            msg.joints = self.angles
            msg.gripper = self.gripper_value
            self.pub.publish(msg)
            return

        if action == 'move_to_default':
            for i in range(6):
                p.setJointMotorControl2(bodyIndex=self.mycobot_id,
                                        jointIndex=i + 1,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.default_pos[i],
                                        targetVelocity=0,
                                        force=500,
                                        positionGain=0.2,
                                        velocityGain=1.5)
            self.angles = self.default_pos
            self.gripper = 100
            self.pos = list(p.getLinkState(self.mycobot_id, 6)[4])
            msg = MyCobotMsg()
            msg.joints = self.angles
            msg.gripper = self.gripper_value
            self.pub.publish(msg)
            return
        elif action == 'move_to_home':
            for i in range(6):
                p.setJointMotorControl2(bodyIndex=self.mycobot_id,
                                        jointIndex=i + 1,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=self.home_pos[i],
                                        targetVelocity=0,
                                        force=500,
                                        positionGain=0.2,
                                        velocityGain=1.5)
            self.angles = self.home_pos
            self.gripper = 100
            self.pos = list(list(p.getLinkState(self.mycobot_id, 6)[4]))
            msg = MyCobotMsg()
            msg.joints = self.angles
            msg.gripper = self.gripper_value
            self.pub.publish(msg)
            return

        elif action == 'grip_on':
            self.gripper_value = 100
            msg = MyCobotMsg()
            msg.joints = self.angles
            msg.gripper = self.gripper_value
            self.pub.publish(msg)
        elif action == 'grip_off':
            self.gripper_value = 0
            msg = MyCobotMsg()
            msg.joints = self.angles
            msg.gripper = self.gripper_value
            self.pub.publish(msg)

        elif action == 'x-':
            self.pos[0] -= delta
        elif action == 'x+':
            self.pos[0] += delta
        elif action == 'y+':
            self.pos[1] += delta
        elif action == 'y-':
            self.pos[1] -= delta
        elif action == 'z+':
            self.pos[2] += delta
        elif action == 'z-':
            self.pos[2] -= delta

        angles = p.calculateInverseKinematics(self.mycobot_id,
                                              6,
                                              self.pos,
                                              self.orn)
        for i in range(6):
            p.setJointMotorControl2(bodyIndex=self.mycobot_id,
                                    jointIndex=i + 1,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=angles[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.2,
                                    velocityGain=1.5)
        self.angles = []
        for i in range(6):
            self.angles.append(p.getJointState(self.mycobot_id, i+1)[0])
        self.pos = list(p.getLinkState(self.mycobot_id, 6)[4])

        msg = MyCobotMsg()
        msg.joints = self.angles
        msg.gripper = self.gripper_value
        self.pub.publish(msg)

    def run(self):
        self.t = self.t + 0.01

        p.stepSimulation()

        for i in range(1):
            pos = [0.15 * math.cos(4*self.t), 0.17, 0.25 +
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
        # print(radians)
        if (self.hasPrevPose):
            p.addUserDebugLine(self.prevPose, pos, [0, 0, 0.3], 1, 15)
            p.addUserDebugLine(self.prevPose1, ls[4], [1, 0, 0], 1, 15)
        self.prevPose = pos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1

        if pos[2] < 0.25:
            gripper = 0
        else:
            gripper = 100

        msg = MyCobotMsg()
        msg.joints = radians
        msg.gripper = gripper
        self.pub.publish(msg)
        print(ls[4])


def main():
    rclpy.init()
    node = PyBulletSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
