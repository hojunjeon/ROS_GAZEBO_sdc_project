#car_controller_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from .pedestrian_detector import PedestrianDetector

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller_node')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Utils
        self.bridge = CvBridge()
        self.detector = PedestrianDetector()

        # States
        self.pedestrian_detected = False
        self.current_y = 0.0
        self.timer = self.create_timer(0.1, self.control_loop)

        # Crosswalk positions
        self.crosswalk_positions = [40.0]

    def odom_callback(self, msg):
        self.current_y = msg.pose.pose.position.y

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.pedestrian_detected = self.detector.detect(cv_image, visualize=True)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def get_nearest_crosswalk(self):
        for y in self.crosswalk_positions:
            if y > self.current_y:
                return y
        return None

    def control_loop(self):
        twist_msg = Twist()

        # 1. 보행자 인식 여부에 따른 조건 분기
        if self.pedestrian_detected:
            nearest_crosswalk = self.get_nearest_crosswalk()

            if nearest_crosswalk is not None:
                stop_line = nearest_crosswalk - 6.0

                # 자동차 절대 좌표가 정지선보다 작으면 직진
                if self.current_y < stop_line:
                    twist_msg.linear.x = 2.0
                    self.get_logger().info(f"보행자 인식됨. 현재 위치 {self.current_y:.2f} < 정지선 {stop_line:.2f}, 계속 직진")
                else:
                    twist_msg.linear.x = 0.0
                    self.get_logger().info(f"보행자 인식됨. 현재 위치 {self.current_y:.2f} >= 정지선 {stop_line:.2f}, 정지")
            else:
                twist_msg.linear.x = 2.0  # 더 이상 횡단보도 없음 -> 직진 유지
        else:
            twist_msg.linear.x = 2.0
            self.get_logger().info("보행자 인식 안됨. 직진 유지")

        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
