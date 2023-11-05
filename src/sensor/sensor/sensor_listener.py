import os
import sys
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from mycobot_msg.msg import MyCobotMsg
from cv_bridge import CvBridge
import cv2
import numpy as np


def crop_center(pil_img: Image, crop_width, crop_height):
    img_width, img_height = pil_img.size
    return pil_img.crop(((img_width - crop_width) // 2,
                         (img_height - crop_height) // 2,
                         (img_width + crop_width) // 2,
                         (img_height + crop_height) // 2))


def crop_max_square(pil_img: Image):
    return crop_center(pil_img, min(pil_img.size), min(pil_img.size))


class SensorListener(Node):
    def __init__(self, data_dir):
        super().__init__('sensor_listener')

        self.data_dir = data_dir

        self.joints = []
        self.imgs = []

        self.rec = False
        self.end_rec = False
        self.idx = -1
        self.img_idx = 0

        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT

        self.img_sub = Subscriber(
            self, Image, '/color/image_raw', qos_profile=qos)
        self.joint_sub = Subscriber(
            self, MyCobotMsg, '/mc_joints', qos_profile=qos)

        queue_size = 30

        self.ts = ApproximateTimeSynchronizer(
            [self.img_sub, self.joint_sub],
            queue_size,
            0.01,
            allow_headerless=True
        )
        self.ts.registerCallback(self.listener_callback)

        self.joy_listener = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.timer = self.create_timer(0.10, self.callback)
        self.img_msg = None
        self.joint_msg = None

    def joy_callback(self, joy):
        if joy.buttons[9]:
            self.get_logger().info("----------record START----------\n"*10)
            if self.rec == False:
                self.idx += 1
                self.img_idx = 0
                os.makedirs(os.path.join(
                    self.data_dir, 'joints'), exist_ok=True)
                os.makedirs(os.path.join(self.data_dir,
                            f'images/{self.idx}'), exist_ok=True)
            self.rec = True
        if joy.buttons[8]:
            self.get_logger().info("----------record will STOP----------\n"*10)
            self.end_rec = True

    def listener_callback(self, img_msg, joint_msg):
        self.img_msg = img_msg
        self.joint_msg = joint_msg

    def callback(self):
        if self.img_msg is None or self.joint_msg is None:
            return

        joints = list(self.joint_msg.joints) + [self.joint_msg.gripper]
        img = CvBridge().imgmsg_to_cv2(self.img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        height, width, _ = img.shape
        size = min(height, width)
        x = (width - size) // 2
        y = (height - size) // 2
        img = img[y:y+size, x:x+size]
        cv2.rotate(img, cv2.ROTATE_180)

        if self.rec:
            self.get_logger().info(str(joints))
            self.joints.append(joints)
            cv2.imwrite(os.path.join(self.data_dir,
                        f'images/{self.idx}/{self.img_idx}.png'), img)
            self.img_idx += 1
            if self.end_rec and self.img_idx == 200:
                np_joints = np.array(self.joints)
                np.save(os.path.join(self.data_dir,
                        f'joints/{self.idx}.npy'), np_joints)
                self.joints = []
                self.rec = False
                self.end_rec = False
                self.get_logger().info("----------record STOP----------\n"*10)
            elif self.img_idx == 200:
                np_joints = np.array(self.joints)
                np.save(os.path.join(self.data_dir,
                        f'joints/{self.idx}.npy'), np_joints)
                self.joints = []
                self.idx += 1
                os.makedirs(os.path.join(self.data_dir,
                            f'images/{self.idx}'), exist_ok=True)
                self.img_idx = 0

        cv2.imshow('Image', img)
        cv2.waitKey(1)


# def end(node: SensorListener):
#     joints = np.array(node.joints)
#     imgs = np.array(node.imgs)
#     np.save('./data/joints.npy', joints)
#     np.save('./data/images.npy', imgs)


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener(args[1])
    # atexit.register(end, node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    args = sys.argv
    main(args)
