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
        self.window_height = self.get_parameter('window_height').value

        self.br = CvBridge()

        self.rgbd_image = np.zeros((self.window_height, self.window_width, 4), dtype=np.uint8)
        self.rgb_subscription = self.create_subscription(Image, '/image_raw', self.rgb_callback, 10)
        self.depth_subscription = self.create_subscription(Image, '/depth/image_raw', self.depth_callback, 10)

        self.publisher_ = self.create_publisher(Image, 'rgbd_image', 10)
        self.timer = self.create_timer(0.04, self.publish_rgbd_image)

        self.get_logger().info("RGB image and depth image acquisition started")


    def rgb_callback(self, rgb_image):
        rgb_frame = self.br.imgmsg_to_cv2(rgb_image, "bgr8")
        rgb_frame = cv2.resize(rgb_frame, (self.window_width, self.window_height))
        rgb_frame = np.flip(rgb_frame, 1)
        self.rgbd_image[:, :, 0:3] = rgb_frame
        # cv2.imshow("image", rgb_frame)
        # cv2.waitKey(1)

    def depth_callback(self, depth_image):
        depth_frame = self.br.imgmsg_to_cv2(depth_image, "16UC1")
        depth_frame = cv2.resize(depth_frame, (self.window_width, self.window_height))
        depth_frame = np.flip(depth_frame, 1)
        cv_image_array = np.array(depth_frame, dtype=np.dtype('f8'))
        cv_image_norm = cv2.normalize(cv_image_array, None, 0, 255, cv2.NORM_MINMAX)
        cv_image_norm = cv_image_norm.astype(np.uint8)
        self.rgbd_image[:, :, 3] = cv_image_norm
        # cv2.imshow("depth", cv_image_norm)
        # print(cv_image_norm)
        # cv2.waitKey(10)
        self.publish_rgbd_image()

    def publish_rgbd_image(self):
        msg = self.br.cv2_to_imgmsg(self.rgbd_image, 'rgba8')
        self.publisher_.publish(msg)
        return msg


def main(args=None):
    rclpy.init(args=args)
    rgbd_image_publisher = RgbdImagePublisher()
    rclpy.spin(rgbd_image_publisher)
    rgbd_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()