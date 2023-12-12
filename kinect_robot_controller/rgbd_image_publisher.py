import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class RgbdImagePublisher(Node):
    def __init__(self):
        super().__init__('rgbd_image_publisher')
        self.declare_parameter('window_width', 640)
        self.declare_parameter('window_height', 480)
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_')
        self.br = CvBridge()
        self.rgbd_image = np.zeros((self.window_width, self.window_height, 4))
        self.rgb_subscription = self.create_subscription(Image, '/image_raw', self.rgb_callback, 10)
        # self.depth_subscription = self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)

    def rgb_callback(self, rgb_image):
        rgb_frame = self.br.imgmsg_to_cv2(rgb_image)
        width = rgb_image.width
        height = rgb_image.height
        rgb_frame = cv2.resize(rgb_frame, (width, height))
        self.rgbd_image[:, :, 0:3] = rgb_frame
        cv2.imshow("image", rgb_frame)
        cv2.waitKey(1)

    def depth_callback(self, depth_image):
        depth_frame = self.br.imgmsg_to_cv2(depth_image)
        width = depth_image.width
        height = depth_image.height


def main(args=None):
    rclpy.init(args=args)
    rgbd_image_publisher = RgbdImagePublisher()
    rclpy.spin(rgbd_image_publisher)
    rgbd_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()