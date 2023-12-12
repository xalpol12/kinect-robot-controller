import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class KinectImageProcessor(Node):
    def __init__(self):
        super().__init__('kinect_image_processor')
        self.cv_bridge = CvBridge()
        self.rgbd_subscription = self.create_subscription(Image, "/rgbd_image", self.rgbd_callback,10)
        self.rgb_image = np.array(0)
        self.depth_image = np.array(0)

    def rgbd_callback(self, rgbd_image: Image):
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 0:3]
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 3]
        cv2.imshow("rgb", self.rgb_image)
        cv2.imshow("depth", self.depth_image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    kinect_image_processor = KinectImageProcessor()
    rclpy.spin(kinect_image_processor)
    kinect_image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()