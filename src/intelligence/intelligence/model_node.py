import sys
from collections import deque
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
from mycobot_msg.msg import MyCobotMsg
from cv_bridge import CvBridge
import cv2
from PIL import Image as PImage
import numpy as np
import matplotlib.pyplot as plt
import pickle
from sklearn.decomposition import PCA
from eipl.utils import deprocess_img, tensor2numpy, normalization

from model.AEMTRNN import AEMTRNN


class ModelNode(Node):
    def __init__(self, model, img_size, joint_bounds, device, input_param=0.6):
        super().__init__('model_node')

        self.on = False
        self.img_msg = None

        self.model = model
        self.img_size = img_size
        self.joint_bounds = joint_bounds
        self.device = device
        self.input_param = input_param

        self.state = None
        self.joints = np.array(
            [-0.005, 1.916, -2.655, -0.561, -0.006, 0.745, 100])
        self.joints = torch.from_numpy(normalization(
            self.joints.astype(np.float32), self.joint_bounds, (0., 1.))).float().unsqueeze(0)
        self.prev_img = torch.zeros(
            (1, 3, self.img_size, self.img_size))

        self.joints_queue = deque([], 100)
        self.hidden_queue = deque([], 100)

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.img_lis = self.create_subscription(
            Image, '/color/image_raw', self.img_callback, qos_profile=qos)
        self.joy_lis = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.10, self.callback)

        self.pub = self.create_publisher(MyCobotMsg, '/radians', 10)
        msg = MyCobotMsg()
        msg.joints = [-0.005, 1.916, -2.655, -0.561, -0.006, 0.745]
        msg.gripper = 100
        self.pub.publish(msg)

        self.fig = plt.figure(figsize=(12, 5))
        self.ax = [self.fig.add_subplot(2, 2, 1),
                   self.fig.add_subplot(2, 2, 2),
                   self.fig.add_subplot(2, 2, 3),
                   self.fig.add_subplot(2, 2, 4)]
        with open(f'./models/pca.pkl', 'rb') as file:
            self.decomp: PCA = pickle.load(file)

    def callback(self):
        if self.img_msg is None:
            return
        if not self.on:
            return

        img = self.img_msg
        img = CvBridge().imgmsg_to_cv2(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_norm, img_raw = self._img_process(img)
        img = torch.from_numpy(img_norm).float()
        img = img.unsqueeze(0)

        self.img = self.input_param * img + \
            (1 - self.input_param) * self.prev_img

        if self.state is not None:
            self.state = (state.to(self.device) for state in self.state)
        prev_img, joints, states = self.model(
            self.img.to(self.device), self.joints.to(self.device), self.state)
        self.prev_img = prev_img.cpu().detach()
        self.joints = joints.cpu().detach()
        self.state = tuple(state.cpu().detach() for state in states)

        self.joints_queue.append(self.joints)
        self.hidden_queue.append(self.state[2])

        # self.get_logger().info(f'{self.joints}')

        joints = self._joint_deprocess()
        self._publish_action(joints)

        # self.plot()

    def img_callback(self, img):
        self.img_msg = img

    def _img_process(self, img):
        height, width, _ = img.shape
        size = min(height, width)
        x = (width - size) // 2
        y = (height - size) // 2
        img = img[y:y + size, x:x + size]
        # img = cv2.rotate(img, cv2.ROTATE_180)
        img = PImage.fromarray(img)
        img = img.resize(self.img_size, self.img_size)
        img = np.array(img)
        img = np.transpose(img, (2, 0, 1))
        img_norm = normalization(img.astype(np.float32), (0., 255.), (0., 1.))
        return img_norm, img

    def _joint_deprocess(self):
        joints = self.joints.squeeze()
        joints = tensor2numpy(joints)
        joints[6] = 0 if joints[6] < 0.5 else 1
        joints = normalization(joints, (0., 1.), self.joint_bounds)
        joints = joints.tolist()
        joints[6] = int(joints[6])
        return joints

    def _publish_action(self, joints):
        self.get_logger().info(f'{joints}')
        msg = MyCobotMsg()
        msg.joints = joints[:6]
        msg.gripper = joints[6]
        self.pub.publish(msg)

    def joy_callback(self, joy):
        if joy.buttons[0]:
            self.on = False
        if joy.buttons[1]:
            self.on = True

    def plot(self):
        plt.cla()

        img = tensor2numpy(self.img.permute(0, 2, 3, 1).squeeze())
        self.ax[0].imshow(img)
        self.ax[0].axis('off')
        self.ax[0].set_title('input image')

        pred_img = tensor2numpy(self.prev_img.permute(0, 2, 3, 1).squeeze())
        pred_img = deprocess_img(pred_img, 0., 1.)
        self.ax[1].imshow(pred_img)
        self.ax[1].axis('off')
        self.ax[1].set_title('predicted image')

        joints_list = np.stack(list(self.joints_queue)).reshape(-1, 7)
        self.ax[2].plot(np.arange(joints_list.shape[0]), joints_list)
        self.ax[2].set_title('joint angles')
        self.ax[2].set_xlabel('step')
        self.ax[2].set_xlim(0, self.joints_queue.maxlen)

        hidden_list = np.stack(list(self.hidden_queue))
        hidden_list = hidden_list.reshape(-1, hidden_list.shape[2])
        print(hidden_list.shape)
        feature = self.decomp.transform(hidden_list)
        feature1, feature2 = feature[:, 0], feature[:, 1]
        self.ax[3].scatter(feature1, feature2, alpha=0.2, s=6, c='b')
        self.ax[3].scatter(feature1[-1], feature2[-1], alpha=1, s=6, c='r')
        self.ax[3].set_xlabel('feature1')
        self.ax[3].set_ylabel('feature2')
        self.ax[3].set_title('hidden state')
        self.ax[3].set_box_aspect(1)
        self.ax[3].set_xlim((-0.3, 0.3))
        self.ax[3].set_ylim(-0.2, 0.4)

        plt.pause(.1)


def main(args=None):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    ckpt = torch.load(args[1])
    joint_bounds = ckpt['joint_bounds']
    model_state_dict = ckpt['model_state_dict']

    model = AEMTRNN(rnn_hidden_dims=[64, 32, 32],
                    image_feat_dim=32,
                    decoder_input_dim=64,
                    joint_dim=7,
                    rnn_slow_tau=100).to(device)
    model.load_state_dict(model_state_dict)
    model.eval()

    rclpy.init(args=args)
    node = ModelNode(model=model,
                     img_size=256,
                     joint_bounds=joint_bounds,
                     device=device)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    args = sys.argv
    main(args)
