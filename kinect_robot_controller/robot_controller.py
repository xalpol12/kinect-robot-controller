import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import numpy as np
import json


MOVE_SPEED: float = 0.0
ROTATION_SPEED: float = 0.0

def on_move_speed_trackbar_value_changed(value):
    global MOVE_SPEED
    MOVE_SPEED = value / 10


def on_rotation_speed_trackbar_value_changed(value):
    global ROTATION_SPEED
    ROTATION_SPEED = value / 1000

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self.cv_bridge = CvBridge()
        self.rgb_subscription = self.create_subscription(Image, "/rgbd_image", self.rgb_callback, 10)
        self.rect_subscription = self.create_subscription(String, "/rect_pos", self.rect_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.rgb_image = np.array(0)
        self.rect_pos = {}
        self.x_window_size = 0
        self.y_window_size = 0
        self.sectors_boundaries = {}
        self.robot_control_msg = Twist()

        self.get_logger().info("robot_controller node initialized")
        cv2.namedWindow('rgb_image')
        cv2.createTrackbar('Linear speed', 'rgb_image', 0, 100, on_move_speed_trackbar_value_changed)
        cv2.createTrackbar('Rotational speed', 'rgb_image', 0, 2000, on_rotation_speed_trackbar_value_changed)

    def rgb_callback(self, image: Image):
        self.load_params(image)
        self.rgb_image = self.cv_bridge.imgmsg_to_cv2(image, "rgba8")[:, :, 0:3]
        self.rgb_image = cv2.UMat(self.rgb_image)
        self.draw_sections(self.rgb_image)
        self.publisher_.publish(self.robot_control_msg)

    def rect_callback(self, rect_params: String):
        self.rect_pos = json.loads(rect_params.data)
        self.rect_pos['center'] = (int(self.rect_pos['x'] + self.rect_pos['w'] / 2), int(self.rect_pos['y'] + self.rect_pos['h'] / 2))
        self.draw_rect_and_center_point()
        self.robot_control_msg = self.check_boundaries(self.rect_pos["center"], self.robot_control_msg, MOVE_SPEED, ROTATION_SPEED)

    def load_params(self, image: Image):
        if self.x_window_size == 0:
            self.x_window_size = image.width
            self.y_window_size = image.height
            self.sectors_boundaries["x1"] = int(self.x_window_size / 3)
            self.sectors_boundaries["x2"] = int(self.x_window_size / 3) * 2
            self.sectors_boundaries["y1"] = int(self.y_window_size / 3)
            self.sectors_boundaries["y2"] = int(self.y_window_size / 3) * 2
            self.get_logger().info("Parameters loaded")

    def draw_rect_and_center_point(self):
        x1 = self.rect_pos['x']
        y1 = self.rect_pos['y']
        x2 = x1 + self.rect_pos['w']
        y2 = y1 + self.rect_pos['h']
        cv2.rectangle(self.rgb_image, (x1, y1), (x2, y2), (255, 0, 255), 3)
        cv2.circle(self.rgb_image, self.rect_pos['center'], 3, (0, 0, 255), cv2.FILLED)
        cv2.imshow('rgb_image', self.rgb_image)
        cv2.waitKey(10)

    def draw_sections(self, image):
        cv2.rectangle(image, (self.sectors_boundaries["x1"], 0), (self.sectors_boundaries["x2"], self.y_window_size), (0, 255, 0), 2)
        cv2.rectangle(image, (0, self.sectors_boundaries["y1"]), (self.x_window_size, self.sectors_boundaries["y2"]), (0, 255, 0), 2)
        cv2.rectangle(image, (0, 0, self.x_window_size, self.y_window_size), (0, 255, 0), 2)

    def check_boundaries(self, point: tuple, twist_msg: Twist, move_speed: float, rotation_speed: float) -> Twist:
        twist_msg = self.vertical_switch_case(point, twist_msg, move_speed)
        twist_msg = self.horizontal_switch_case(point, twist_msg, rotation_speed)
        return twist_msg

    def vertical_switch_case(self, point: tuple, twist_msg: Twist, move_speed: float) -> Twist:
        if point[1] < self.sectors_boundaries["y1"]:
            twist_msg.linear.x = move_speed
        elif point[1] > self.sectors_boundaries["y2"]:
            twist_msg.linear.x = - move_speed
        else:
            twist_msg.linear.x = 0.0
        return twist_msg

    def horizontal_switch_case(self, point: tuple, twist_msg: Twist, rotation_speed: float) -> Twist:
        if point[0] < self.sectors_boundaries["x1"]:
            twist_msg.angular.z = rotation_speed
        elif point[0] > self.sectors_boundaries["x2"]:
            twist_msg.angular.z = - rotation_speed
        else:
            twist_msg.angular.z = 0.0
        return twist_msg


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
