import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random

PLOT_AREA = 0.5


class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self):
        # theta = self.joint_angle_
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]

        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])

        return trans

    @staticmethod
    def basic_jacobian(trans_prev, ee_pos):
        pos_prev = np.array(
            [trans_prev[0, 3], trans_prev[1, 3], trans_prev[2, 3]])
        z_axis_prev = np.array(
            [trans_prev[0, 2], trans_prev[1, 2], trans_prev[2, 2]])

        basic_jacobian = np.hstack(
            (np.cross(z_axis_prev, ee_pos - pos_prev), z_axis_prev))
        return basic_jacobian


class NLinkArm:
    def __init__(self, dh_params_list):
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection='3d')

    @staticmethod
    def convert_joint_angles_mc_to_sim(joint_angles):
        """convert joint angles simulator to mycobot

        Args:
            joint_angles ([float]): [joint angles(radian)]

        Returns:
            [float]: [joint angles calculated(radian)]
        """
        conv_mul = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        conv_add = [0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, 0.0]

        joint_angles = [
            joint_angles * conv_mul for (joint_angles, conv_mul) in zip(joint_angles, conv_mul)]
        joint_angles = [
            joint_angles + conv_add for (joint_angles, conv_add) in zip(joint_angles, conv_add)]

        joint_angles_lim = []
        for joint_angle in joint_angles:
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles_lim.append(joint_angle)

        return joint_angles_lim

    @staticmethod
    def convert_joint_angles_sim_to_mc(joint_angles):
        conv_mul = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        conv_add = [0.0, math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, 0.0]

        joint_angles = [
            joint_angles * conv_mul for (joint_angles, conv_mul) in zip(joint_angles, conv_mul)]
        joint_angles = [
            joint_angles + conv_add for (joint_angles, conv_add) in zip(joint_angles, conv_add)]

        joint_angles_lim = []
        for joint_angle in joint_angles:
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles_lim.append(joint_angle)

        return joint_angles_lim

    def transformation_matrix(self):
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
        return trans

    def forward_kinematics(self):
        trans = self.transformation_matrix()

        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        alpha, beta, gamma = self.euler_angle()

        return [x, y, z, alpha, beta, gamma]

    def basic_jacobian(self):
        ee_pos = self.forward_kinematics()[0:3]
        basic_jacobian_mat = []

        trans = np.identity(4)
        for i in range(len(self.link_list)):
            basic_jacobian_mat.append(
                self.link_list[i].basic_jacobian(trans, ee_pos))
            trans = np.dot(trans, self.link_list[i].transformation_matrix())

        return np.array(basic_jacobian_mat).T

    def inverse_kinematics(self, ref_ee_pose):
        for cnt in range(10):
            ee_pose = self.forward_kinematics()
            diff_pose = np.array(ref_ee_pose) - ee_pose

            basic_jacobian_mat = self.basic_jacobian()
            alpha, beta, gamma = self.euler_angle()

            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            theta_dot = np.dot(
                np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha),
                np.array(diff_pose))
            self.update_joint_angles(theta_dot / 100.)
        return self.get_angles()

    def euler_angle(self):
        trans = self.transformation_matrix()

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))

        return alpha, beta, gamma

    def send_angles(self, joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] = joint_angle_list[i]

    def update_joint_angles(self, diff_joint_angle_list):
        for i in range(len(self.link_list)):
            self.link_list[i].dh_params_[0] += diff_joint_angle_list[i]

    def get_angles(self):
        joint_angles = []
        for i in range(len(self.link_list)):
            joint_angle = self.link_list[i].dh_params_[0]
            while joint_angle > math.pi:
                joint_angle -= 2 * math.pi

            while joint_angle < -math.pi:
                joint_angle += 2 * math.pi

            joint_angles.append(joint_angle)

        return joint_angles

    def plot(self, realtime=False):
        if realtime:
            plt.cla()
        else:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(projection='3d')

        x_list = []
        y_list = []
        z_list = []

        trans = np.identity(4)

        x_list.append(trans[0, 3])
        y_list.append(trans[1, 3])
        z_list.append(trans[2, 3])
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix())
            x_list.append(trans[0, 3])
            y_list.append(trans[1, 3])
            z_list.append(trans[2, 3])

        self.ax.plot(x_list, y_list, z_list, "o-", color="#00aa00", ms=4,
                     mew=0.5)
        self.ax.plot([0], [0], [0], "o")

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.set_xlim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_ylim(-PLOT_AREA, PLOT_AREA)
        self.ax.set_zlim(-PLOT_AREA, PLOT_AREA)

        if realtime:
            plt.pause(.1)
        else:
            plt.show()


def random_val(min_val, max_val):
    return min_val + random.random() * (max_val - min_val)


if __name__ == "__main__":
    # myCobot DH parameters
    # [theta, alpha, a, d]
    mycobot_sim = NLinkArm([[0., math.pi / 2, 0, 0.13156],
                            [0., 0., -0.1104, 0.],
                            [0., 0., -0.096, 0.],
                            [0., math.pi / 2, 0., 0.06639],
                            [0., -math.pi / 2, 0., 0.07318],
                            [0., 0., 0., 0.0436]])

    mycobot_sim.send_angles(
        mycobot_sim.convert_joint_angles_mc_to_sim
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    mycobot_sim.forward_kinematics()
