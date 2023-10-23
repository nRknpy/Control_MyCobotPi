import os
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from mycobot_msg.msg import MyCobotMsg
from cv_bridge import CvBridge
import cv2
import numpy as np
# from PIL import Image
import atexit


def crop_center(pil_img: Image, crop_width, crop_height):
    img_width, img_height = pil_img.size
    return pil_img.crop(((img_width - crop_width) // 2,
                         (img_height - crop_height) // 2,
                         (img_width + crop_width) // 2,
                         (img_height + crop_height) // 2))


def crop_max_square(pil_img: Image):
    return crop_center(pil_img, min(pil_img.size), min(pil_img.size))


class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')

        self.joints = []
        self.imgs = []

        self.rec = False
        self.idx = 0
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
        self.ts.registerCallback(self.callback)

        self.joy_listener = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy):
        if joy.buttons[9]:
            self.get_logger().info("----------record START----------")
            if self.rec == False:
                self.idx += 1
                self.img_idx = 0
                os.makedirs(f'./data/images/{self.idx}', exist_ok=True)
            self.rec = True
        if joy.buttons[8]:
            self.get_logger().info("----------record STOP----------")
            if self.rec == True:
                np_joints = np.array(self.joints)
                np.save(f'./data/joints/{self.idx}.npy', np_joints)
            self.rec = False

    def callback(self, img_msg, joint_msg):
        joints = list(joint_msg.joints) + [joint_msg.gripper]
        img = CvBridge().imgmsg_to_cv2(img_msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        self.get_logger().info(str(joints))

        height, width, _ = img.shape
        size = min(height, width)
        x = (width - size) // 2
        y = (height - size) // 2
        img = img[y:y+size, x:x+size]
        cv2.rotate(img, cv2.ROTATE_180)

        if self.rec:
            self.joints.append(joints)
            cv2.imwrite(f'./data/images/{self.idx}/{self.img_idx}.png', img)
            self.img_idx += 1

        cv2.imshow('Image', img)
        cv2.waitKey(1)


# def end(node: SensorListener):
#     joints = np.array(node.joints)
#     imgs = np.array(node.imgs)
#     np.save('./data/joints.npy', joints)
#     np.save('./data/images.npy', imgs)


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    # atexit.register(end, node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
