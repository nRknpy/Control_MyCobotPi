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

    def run(self):
        self.t = self.t + 0.01

        p.stepSimulation()

        for i in range(1):
            pos = [0.1 * math.cos(self.t), 0.2, 0.2 + 0.1 * math.sin(self.t)]
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

        msg = MyCobotMsg()
        msg.joints = radians
        msg.gripper = 100
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
