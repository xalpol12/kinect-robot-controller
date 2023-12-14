import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

MIN_THRESHOLD_TRACKBAR_VALUE = 0
MAX_THRESHOLD_TRACKBAR_VALUE = 200
DEPTH_DETECTION_THRESHOLD = 40

def on_detection_threshold_change(value):
    global DEPTH_DETECTION_THRESHOLD
    DEPTH_DETECTION_THRESHOLD = value

class Window():
    def __init__(self, window_name) -> None:
        self.window_name = window_name
        self.image = None

    def create_trackbar(self, trackbar_name, slider_min, slider_max, on_change):
        cv2.namedWindow(self.window_name) 
        this_trackbar_name = f'{trackbar_name}'
        cv2.createTrackbar(this_trackbar_name, self.window_name, slider_min, slider_max, on_change)

    def update_image(self, image):
        self.image = image
        cv2.imshow(self.window_name, image)
        cv2.waitKey(10)


class KinectImageProcessor(Node):
    def __init__(self):
        super().__init__('kinect_image_processor')
        self.cv_bridge = CvBridge()
        self.rgbd_subscription = self.create_subscription(Image, "/rgbd_image", self.rgbd_callback, 10)
        
        self.rgb_image = np.array(0)
        self.depth_image = np.array(0)
        self.thresh_away = np.array(0)
        self.threshed_depth_image = np.array(0)

        self.rgb_window = Window("rgb")
        self.depth_window = Window("threshed_depth")
        # self.depth_window.create_trackbar("threshold", MIN_THRESHOLD_TRACKBAR_VALUE, MAX_THRESHOLD_TRACKBAR_VALUE, on_detection_threshold_change)

    def rgbd_callback(self, rgbd_image: Image):
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 0:3]
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 3]
        self.depth_image = cv2.blur(self.depth_image, (7, 7))
        _, thresh_away = cv2.threshold(self.depth_image, DEPTH_DETECTION_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        _, thresh_close = cv2.threshold(self.depth_image, 2, 255, cv2.THRESH_BINARY)
        self.threshed_depth_image = cv2.bitwise_xor(thresh_close, thresh_away)
        self.update_windows()

    def update_windows(self):
        self.depth_window.update_image(self.threshed_depth_image)
        self.rgb_window.update_image(self.rgb_image)


def main(args=None):
    rclpy.init(args=args)
    kinect_image_processor = KinectImageProcessor()
    rclpy.spin(kinect_image_processor)
    kinect_image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
