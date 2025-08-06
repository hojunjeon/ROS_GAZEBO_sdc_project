import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller_node')

        # 카메라 구독
        self.image_sub = self.create_subscription(
            Image,
            '/prius_hybrid/front_camera/image_raw',
            self.image_callback,
            10
        )

        # 자동차 속도 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, '/prius_hybrid/cmd_vel', 10)

        # OpenCV용 브리지
        self.bridge = CvBridge()

        # HOG 기반 보행자 감지기
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # 제어 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 내부 상태
        self.current_y = 0.0
        self.velocity = 2.0  # m/s
        self.stop_at_y = None  # 정지 지점
        self.crosswalk_positions = [20.0, 40.0, 70.0]  # 횡단보도 y 위치
        self.detected = False

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 오류: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects, _ = self.hog.detectMultiScale(gray, winStride=(4,4), padding=(8,8), scale=1.05)

        self.detected = len(rects) > 0

        # 보행자 감지되면 가장 가까운 횡단보도 앞 4m 지점 설정
        if self.detected:
            if self.stop_at_y is None:  # 처음 감지되었을 때만 설정
                for crosswalk_y in self.crosswalk_positions:
                    if self.current_y < crosswalk_y - 4.0:
                        self.stop_at_y = crosswalk_y - 4.0
                        self.get_logger().info(f'보행자 감지! 정지 위치: y = {self.stop_at_y}')
                        break
        else:
            # 보행자가 사라지면 다음 횡단보도 진입 가능하게 초기화
            if self.stop_at_y is not None and self.current_y > self.stop_at_y + 4.0:
                self.get_logger().info('보행자 사라짐 - 주행 재개')
                self.stop_at_y = None

    def control_loop(self):
        msg = Twist()

        # 정지 지점이 설정되어 있다면
        if self.stop_at_y is not None:
            if self.current_y < self.stop_at_y:
                msg.linear.y = self.velocity
                self.current_y += self.velocity * 0.1  # 0.1초 간격 이동
            else:
                msg.linear.y = 0.0  # 정지
        else:
            # 보행자 없으면 계속 전진
            msg.linear.y = self.velocity
            self.current_y += self.velocity * 0.1

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
