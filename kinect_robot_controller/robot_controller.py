import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
import numpy.typing as npt
import json

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.cv_bridge = CvBridge()
        self.rgb_subscription = self.create_subscription(Image, "/image_raw", self.rgb_callback, 10)
        self.rect_subscription = self.create_subscription(String, "/rect_pos", self.rect_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "vel", 10)

        self.rgb_image = np.array(0)
        self.rect_pos = {}
        self.x_window_size = 0
        self.y_window_size = 0
        cv2.namedWindow('dupa')

    def rgb_callback(self, image: Image):
        self.load_params(image)
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        self.rgb_image = cv2.UMat(np.flip(self.rgb_image, 1))
        self.draw_sections(self.rgb_image)


    def rect_callback(self, rect_params: String):
        self.rect_pos = json.loads(rect_params.data)
        self.rect_pos['center'] = (int(self.rect_pos['x'] + self.rect_pos['w'] / 2), int(self.rect_pos['y'] + self.rect_pos['h'] / 2))
        self.draw_rect_and_center_point()

    def find_section(self):
        print("a - mimimimi")

    def load_params(self, image: Image):
        if self.x_window_size == 0:
            self.x_window_size = image.width
            self.y_window_size = image.height

    def draw_rect_and_center_point(self):
        x1 = self.rect_pos['x']
        y1 = self.rect_pos['y']
        x2 = x1 + self.rect_pos['w']
        y2 = y1 + self.rect_pos['h']
        cv2.rectangle(self.rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.circle(self.rgb_image, self.rect_pos['center'], 3, (0, 0, 255), cv2.FILLED)
        cv2.imshow('dupa', self.rgb_image)
        cv2.waitKey(10)

    def draw_sections(self, image):
        cv2.rectangle(image, (int(self.x_window_size / 3), 0), (int(self.x_window_size / 3) * 2, self.y_window_size), (0, 255, 0), 2)
        cv2.rectangle(image, (0, int(self.y_window_size / 3)), (self.x_window_size, int(self.y_window_size / 3) * 2), (0, 255, 0), 2)
        cv2.rectangle(image, (0, 0, self.x_window_size, self.y_window_size), (0, 255, 0), 2)



def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()