import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import numpy.typing as npt

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
        self.cnts = []
        self.x, self.y = (0, 0)
        self.hand_size_scaling = 0.001
        self.image_queue = np.zeros((1, 1, 3))

        self.rgb_window = Window("rgb")
        self.depth_window = Window("threshed_depth")
        self.depth_window.create_trackbar("threshold", MIN_THRESHOLD_TRACKBAR_VALUE, MAX_THRESHOLD_TRACKBAR_VALUE, on_detection_threshold_change)

    def rgbd_callback(self, rgbd_image: Image):
        self.load_parameters(rgbd_image)
        self.initial_in_range_threshing(rgbd_image)
        cnts, _ = cv2.findContours(self.threshed_depth_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        self.cnts = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area > self.x * self.y * self.hand_size_scaling:
                self.cnts.append(cnt)
        cv2.drawContours(self.rgb_image, self.cnts, -1, (255, 0, 0), 3)
        self.update_windows()

    def load_parameters(self, image: Image) -> None:
        if self.x == 0:
            self.x = image.width
            self.y = image.height
            self.image_queue = np.zeros((self.y, self.x, 3))

    def initial_in_range_threshing(self, rgbd_image: Image):
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 0:3]
        self.rgb_image = cv2.UMat(self.rgb_image)
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(rgbd_image, 'rgba8')[:, :, 3]
        self.threshed_depth_image = cv2.inRange(self.depth_image, 2, DEPTH_DETECTION_THRESHOLD)
        self.threshed_depth_image = cv2.dilate(self.threshed_depth_image, (13, 13))
        self.push_image_queue(self.threshed_depth_image)
        self.threshed_depth_image = np.min(self.image_queue, axis=2).astype(np.uint8)
        # self.threshed_depth_image = cv2.morphologyEx(self.threshed_depth_image, cv2.MORPH_CLOSE, (7, 7))

    def update_windows(self):
        self.depth_window.update_image(self.threshed_depth_image)
        self.rgb_window.update_image(self.rgb_image)

    def push_image_queue(self, frame: npt.NDArray[any]):
        self.image_queue[:, :, 1:] = self.image_queue[:, :, 0:-1]
        self.image_queue[:, :, 0] = frame

def main(args=None):
    rclpy.init(args=args)
    kinect_image_processor = KinectImageProcessor()
    rclpy.spin(kinect_image_processor)
    kinect_image_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
