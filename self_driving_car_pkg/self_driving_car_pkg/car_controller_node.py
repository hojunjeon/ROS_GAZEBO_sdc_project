# car_controller_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from .pedestrian_detector import PedestrianDetector

class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller_node')

        self.bridge = CvBridge()
        self.pedestrian_detector = PedestrianDetector()

        # 카메라 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # 자동차 명령 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 보행자 탐지 상태 플래그
        self.pedestrian_detected = False

        # 10Hz 제어 루프 타이머
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Car Controller Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 보행자 탐지 수행
        self.pedestrian_detected = self.pedestrian_detector.detect(cv_image)

    def control_loop(self):
        twist_msg = Twist()

        if self.pedestrian_detected:
            # 보행자 감지 시 정지
            twist_msg.linear.x = 0.0
            self.get_logger().info("Pedestrian detected: Stopping car.")
        else:
            # 보행자 미감지 시 전진 (속도 1.0 m/s, 필요 시 조정)
            twist_msg.linear.x = 1.0

        # 회전은 하지 않음 (직진)
        twist_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
